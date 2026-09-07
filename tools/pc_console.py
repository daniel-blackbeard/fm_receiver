#!/usr/bin/env python3
"""
fm_receiver PC-side command console.

Talks the same 8-byte command protocol the board's UART1 console and its
UDP command path both share (see sw/main.c's dispatch_command() and the
protocol doc-comment right above it, and sw/eth0.h for the device/port
constants) -- this tool only ever exercises the UDP path.

Wire format, one UDP request:
    preamble (4 bytes): ASCII "COM" + 0x00        -- UDP_CMD_PREAMBLE
    one or more 8-byte commands, back-to-back:
        byte 0   = device select (CMD_DEV_*)
        byte 1   = 0x00 read, 0x01 write
        bytes 2-3 = 16-bit address, big-endian
        bytes 4-7 = 32-bit data, big-endian (write value; ignored on read)
    stop sentinel (8 bytes): all 0xFF               -- only needed if more
                                                        commands could follow;
                                                        harmless to always
                                                        send it, so this tool
                                                        always does

Reply (only if at least one command in the request produces one): a
sequence of 4-byte big-endian values, one per replying command, in the
same order the commands were sent -- packed into a single UDP datagram,
never one datagram per command.

No dependencies beyond the Python standard library (socket + tkinter).
Run: python pc_console.py
"""

import socket
import struct
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Reuse the sample-stream receiver/unpacking logic verbatim rather than
# duplicating it -- see sample_stream_view.py's own docstring for the full
# wire-format derivation (AXI byte-lane mapping -> per-sample field order).
# Binds a *different* UDP port (SAMPLE_PORT=5556) than this console's own
# command socket (which never binds at all, just sends from an ephemeral
# port and reads the reply back on it) -- the two can't collide, and
# sample_stream_view.py can still be run standalone alongside this tab, or
# instead of it, freely.
from sample_stream_view import (
    SampleReceiver, CHANNEL_NAMES, SAMPLE_PORT, SAMPLE_RATE_HZ, DEFAULT_FFT_SIZE,
)

# --- Protocol constants (mirrors sw/eth0.h / sw/main.c exactly) -----------

BOARD_IP = "192.168.3.50"
UDP_CMD_PORT = 5555
UDP_CMD_PREAMBLE = b"COM\x00"
STOP_SENTINEL = b"\xff" * 8

CMD_RW_READ = 0x00
CMD_RW_WRITE = 0x01

# name -> (device select byte, produces a reply on write?)
# Every device replies on a read except SYS, which is write-only (mode
# select) and never produces a reply either way -- see dispatch_command()
# in sw/main.c. "replies on write" is only relevant for the write+readback
# pairing below: SYS is the one device where pairing a read after the
# write is meaningless, since it would never come back anyway.
DEVICES = {
    "AXI (axi_registers)":  0x00,
    "SYS (mode select)":    0x04,
    "SPI (AD9361)":         0x08,
    "CDC (axi_cdc_status)": 0x0C,
    "GEM (GEM0 core)":      0x10,
    "DESC (TX descriptor)": 0x14,
    "SLCR":                 0x18,
    "DESC_RX":              0x1C,
    "RXBUF":                0x20,
}
CMD_DEV_SYS = 0x04

# Known AXI (axi_registers) offsets -- source of truth is fm_receiver.sv's
# own "RM 0x.." comments next to each regmap assignment. Extend this list
# if more regmap words get defined later; nothing else needs to change.
AXI_REGMAP_FIELDS = [
    # (offset, register label, [(bit_or_range_label, mask, shift), ...])
    (0x00, "CTRL",      [("Blink Enable (bit0)", 0x1, 0)]),
    (0x04, "DIV",       [("Blink Speed (bits[4:0])", 0x1F, 0)]),
    (0x08, "AD9361 Pins", [
        ("Enable (bit0)",       0x1, 0),
        ("TXNRX (bit1)",        0x1, 1),
        ("RESETB released (bit2)", 0x1, 2),
        ("R1_MODE (bit3)",      0x1, 3),
    ]),
]


class BoardLink:
    """One UDP socket, one board. Blocking with a short timeout -- this
    tool only ever sends on manual user action, never polls, so a brief
    GUI pause during a send is an acceptable trade for not needing a
    background thread."""

    def __init__(self, ip=BOARD_IP, port=UDP_CMD_PORT, timeout=1.0):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def pack_command(self, dev, rw, addr, data):
        return bytes([dev & 0xFF, rw & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]) \
            + struct.pack(">I", data & 0xFFFFFFFF)

    def send_commands(self, commands, expected_replies):
        """commands: list of already-packed 8-byte commands.
        expected_replies: how many of them produce a reply value, in order.
        Returns a list of that many 32-bit ints, or raises socket.timeout /
        OSError on failure -- caller decides how to surface that."""
        payload = UDP_CMD_PREAMBLE + b"".join(commands) + STOP_SENTINEL
        self.sock.sendto(payload, self.addr)
        if expected_replies == 0:
            return []
        data, _ = self.sock.recvfrom(4096)
        if len(data) < expected_replies * 4:
            raise ValueError(
                f"expected {expected_replies} reply word(s) "
                f"({expected_replies * 4} bytes), got {len(data)} bytes"
            )
        return [struct.unpack_from(">I", data, i * 4)[0] for i in range(expected_replies)]

    def close(self):
        self.sock.close()


class ConsoleApp:
    def __init__(self, root):
        self.root = root
        root.title("fm_receiver PC console")

        self.link = BoardLink()
        # (dev, addr) -> last known 32-bit value, updated from every
        # response this tool has ever seen. Only ever written from a real
        # reply, never guessed -- see the "no reason to mistrust it" call.
        self.state = {}

        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        self.console_tab = ttk.Frame(notebook)
        self.regmap_tab = ttk.Frame(notebook)
        self.sample_tab = ttk.Frame(notebook)
        notebook.add(self.console_tab, text="Command Console")
        notebook.add(self.regmap_tab, text="AXI Regmap")
        notebook.add(self.sample_tab, text="RX Sample Stream (FFT)")

        self._build_console_tab()
        self._build_regmap_tab()
        self._build_sample_tab()

        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    def _build_console_tab(self):
        f = self.console_tab

        send_frame = ttk.LabelFrame(f, text="Send command")
        send_frame.pack(fill="x", padx=8, pady=8)

        ttk.Label(send_frame, text="Device:").grid(row=0, column=0, sticky="e", padx=4, pady=4)
        self.dev_var = tk.StringVar(value=list(DEVICES.keys())[0])
        dev_combo = ttk.Combobox(send_frame, textvariable=self.dev_var,
                                  values=list(DEVICES.keys()), state="readonly", width=22)
        dev_combo.grid(row=0, column=1, padx=4, pady=4)
        dev_combo.bind("<<ComboboxSelected>>", self._on_device_change)

        ttk.Label(send_frame, text="Address (hex):").grid(row=0, column=2, sticky="e", padx=4, pady=4)
        self.addr_var = tk.StringVar(value="0000")
        ttk.Entry(send_frame, textvariable=self.addr_var, width=10).grid(row=0, column=3, padx=4, pady=4)

        ttk.Label(send_frame, text="Data (hex):").grid(row=0, column=4, sticky="e", padx=4, pady=4)
        self.data_var = tk.StringVar(value="00000000")
        ttk.Entry(send_frame, textvariable=self.data_var, width=12).grid(row=0, column=5, padx=4, pady=4)

        self.rw_var = tk.StringVar(value="read")
        ttk.Radiobutton(send_frame, text="Read", variable=self.rw_var, value="read").grid(row=1, column=1, sticky="w")
        self.write_radio = ttk.Radiobutton(send_frame, text="Write (+ readback)", variable=self.rw_var, value="write")
        self.write_radio.grid(row=1, column=2, columnspan=2, sticky="w")

        ttk.Button(send_frame, text="Send", command=self._on_send).grid(row=1, column=5, padx=4, pady=4, sticky="e")

        log_frame = ttk.LabelFrame(f, text="Command log")
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log = tk.Listbox(log_frame, font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, command=self.log.yview)
        scroll.pack(fill="y", side="right")
        self.log.config(yscrollcommand=scroll.set)

    def _on_device_change(self, _event=None):
        # SYS is write-only and never replies -- reading it is meaningless,
        # so just steer the user away from it rather than sending a
        # request that can never produce anything.
        is_sys = DEVICES[self.dev_var.get()] == CMD_DEV_SYS
        if is_sys:
            self.rw_var.set("write")

    def _log(self, text):
        self.log.insert("end", f"[{time.strftime('%H:%M:%S')}] {text}")
        self.log.see("end")

    def _on_send(self):
        dev_name = self.dev_var.get()
        dev = DEVICES[dev_name]
        try:
            addr = int(self.addr_var.get(), 16)
            data = int(self.data_var.get(), 16)
        except ValueError:
            self._log(f"ERROR: address/data must be hex (got addr='{self.addr_var.get()}' data='{self.data_var.get()}')")
            return

        is_write = self.rw_var.get() == "write"

        try:
            if is_write:
                write_cmd = self.link.pack_command(dev, CMD_RW_WRITE, addr, data)
                if dev == CMD_DEV_SYS:
                    # No readback -- SYS never replies, pairing one would
                    # just wait out a timeout for nothing.
                    self.link.send_commands([write_cmd], expected_replies=0)
                    self._log(f"WRITE {dev_name} addr=0x{addr:04X} data=0x{data:08X} (no readback for SYS)")
                    return
                read_cmd = self.link.pack_command(dev, CMD_RW_READ, addr, 0)
                (readback,) = self.link.send_commands([write_cmd, read_cmd], expected_replies=1)
                self.state[(dev, addr)] = readback
                self._log(f"WRITE {dev_name} addr=0x{addr:04X} data=0x{data:08X} -> readback 0x{readback:08X}")
            else:
                read_cmd = self.link.pack_command(dev, CMD_RW_READ, addr, 0)
                (value,) = self.link.send_commands([read_cmd], expected_replies=1)
                self.state[(dev, addr)] = value
                self._log(f"READ  {dev_name} addr=0x{addr:04X} -> 0x{value:08X}")
        except socket.timeout:
            self._log(f"TIMEOUT: no reply from board for {dev_name} addr=0x{addr:04X}")
            return
        except (OSError, ValueError) as exc:
            self._log(f"ERROR: {exc}")
            return

        if dev == DEVICES["AXI (axi_registers)"]:
            self._refresh_regmap_display()

    # ------------------------------------------------------------------
    def _build_regmap_tab(self):
        f = self.regmap_tab
        ttk.Button(f, text="Bulk read whole regmap", command=self._on_bulk_read) \
            .pack(anchor="w", padx=8, pady=8)

        self.regmap_labels = {}
        for offset, reg_label, fields in AXI_REGMAP_FIELDS:
            box = ttk.LabelFrame(f, text=f"0x{offset:02X} -- {reg_label}")
            box.pack(fill="x", padx=8, pady=4)
            raw_var = tk.StringVar(value="(never read)")
            ttk.Label(box, textvariable=raw_var, font=("Consolas", 9, "bold")).pack(anchor="w", padx=4)
            self.regmap_labels[offset] = {"raw": raw_var, "fields": []}
            for field_label, mask, shift in fields:
                fv = tk.StringVar(value="--")
                ttk.Label(box, text=f"  {field_label}:").pack(side="left", padx=(4, 2))
                lbl = ttk.Label(box, textvariable=fv)
                lbl.pack(side="left", padx=(0, 12))
                self.regmap_labels[offset]["fields"].append((fv, mask, shift))

    def _refresh_regmap_display(self):
        dev = DEVICES["AXI (axi_registers)"]
        for offset, entry in self.regmap_labels.items():
            value = self.state.get((dev, offset))
            if value is None:
                continue
            entry["raw"].set(f"raw = 0x{value:08X}")
            for fv, mask, shift in entry["fields"]:
                fv.set(str((value >> shift) & mask))

    # ------------------------------------------------------------------
    def _build_sample_tab(self):
        f = self.sample_tab
        self.sample_recv = None       # SampleReceiver, only while listening
        self.sample_after_id = None   # root.after() handle for the redraw loop

        ctrl = ttk.Frame(f)
        ctrl.pack(fill="x", padx=8, pady=8)
        self.sample_status_var = tk.StringVar(value="Not listening")
        ttk.Label(ctrl, textvariable=self.sample_status_var).pack(side="left")
        self.sample_toggle_btn = ttk.Button(ctrl, text="Start listening", command=self._toggle_sample_stream)
        self.sample_toggle_btn.pack(side="right")

        # Editable sample rate driving the FFT frequency axis, defaulting
        # to the current hardware-measured decimated rate (SAMPLE_RATE_HZ,
        # from sample_stream_view.py -- see its own comment for how that
        # figure was derived). Not read from the board; purely a display
        # setting, so it's safe to change freely if the decimation ratio
        # or clk_dsp ever changes.
        self.sample_rate_hz = SAMPLE_RATE_HZ
        ttk.Label(ctrl, text="Sample rate (Hz):").pack(side="left", padx=(16, 4))
        self.sample_rate_var = tk.StringVar(value=f"{SAMPLE_RATE_HZ:.0f}")
        rate_entry = ttk.Entry(ctrl, textvariable=self.sample_rate_var, width=12)
        rate_entry.pack(side="left")
        rate_entry.bind("<Return>", self._on_sample_rate_change)
        rate_entry.bind("<FocusOut>", self._on_sample_rate_change)

        fig = Figure(figsize=(9, 6))
        axes = fig.subplots(2, 2)
        self.sample_freqs_mhz = np.fft.rfftfreq(DEFAULT_FFT_SIZE, d=1.0 / SAMPLE_RATE_HZ) / 1e6
        self.sample_window = np.hanning(DEFAULT_FFT_SIZE)
        self.sample_axes = axes
        self.sample_lines = {}
        for ax, name in zip(axes.flat, CHANNEL_NAMES):
            (line,) = ax.plot(self.sample_freqs_mhz, np.zeros_like(self.sample_freqs_mhz))
            ax.set_title(name)
            ax.set_xlabel("Freq (MHz)")
            ax.set_ylabel("Magnitude (dB)")
            ax.set_ylim(-20, 100)
            self.sample_lines[name] = line
        fig.tight_layout()

        self.sample_canvas = FigureCanvasTkAgg(fig, master=f)
        self.sample_canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=(0, 8))

    def _on_sample_rate_change(self, _event=None):
        try:
            rate_hz = float(self.sample_rate_var.get())
            if rate_hz <= 0:
                raise ValueError
        except ValueError:
            self.sample_rate_var.set(f"{self.sample_rate_hz:.0f}")  # revert to last-good
            return
        self.sample_rate_hz = rate_hz
        self.sample_freqs_mhz = np.fft.rfftfreq(DEFAULT_FFT_SIZE, d=1.0 / rate_hz) / 1e6
        for line in self.sample_lines.values():
            line.set_xdata(self.sample_freqs_mhz)
        for ax in self.sample_axes.flat:
            ax.set_xlim(self.sample_freqs_mhz[0], self.sample_freqs_mhz[-1])
        self.sample_canvas.draw_idle()

    def _toggle_sample_stream(self):
        if self.sample_recv is None:
            try:
                self.sample_recv = SampleReceiver(fft_size=DEFAULT_FFT_SIZE)
            except OSError as exc:
                # Most likely cause: sample_stream_view.py (or another copy
                # of this tab) already has SAMPLE_PORT bound -- only one
                # listener can hold a given UDP port at a time.
                self.sample_status_var.set(f"ERROR binding :{SAMPLE_PORT}: {exc}")
                return
            self.sample_recv.start()
            self.sample_toggle_btn.config(text="Stop listening")
            self._sample_update()
        else:
            self._stop_sample_stream()

    def _stop_sample_stream(self):
        if self.sample_after_id is not None:
            self.root.after_cancel(self.sample_after_id)
            self.sample_after_id = None
        if self.sample_recv is not None:
            self.sample_recv.stop()
            self.sample_recv = None
        self.sample_toggle_btn.config(text="Start listening")
        self.sample_status_var.set("Not listening")

    def _sample_update(self):
        if self.sample_recv is None:
            return
        data = self.sample_recv.snapshot()
        for name, line in self.sample_lines.items():
            buf = data[name]
            if len(buf) < DEFAULT_FFT_SIZE:
                continue
            spectrum = np.fft.rfft(buf[-DEFAULT_FFT_SIZE:] * self.sample_window)
            line.set_ydata(20 * np.log10(np.abs(spectrum) + 1e-9))
        self.sample_status_var.set(
            f"Listening on :{SAMPLE_PORT} -- {self.sample_recv.packets_received} pkts, "
            f"{self.sample_recv.bytes_received / 1024:.0f} KB, "
            f"{self.sample_recv.packets_dropped} dropped"
        )
        self.sample_canvas.draw_idle()
        self.sample_after_id = self.root.after(200, self._sample_update)

    def _on_close(self):
        self._stop_sample_stream()
        self.root.destroy()

    # ------------------------------------------------------------------
    def _on_bulk_read(self):
        dev = DEVICES["AXI (axi_registers)"]
        offsets = [offset for offset, _, _ in AXI_REGMAP_FIELDS]
        cmds = [self.link.pack_command(dev, CMD_RW_READ, off, 0) for off in offsets]
        try:
            values = self.link.send_commands(cmds, expected_replies=len(cmds))
        except socket.timeout:
            self._log("TIMEOUT: bulk regmap read got no reply")
            return
        except (OSError, ValueError) as exc:
            self._log(f"ERROR: bulk regmap read failed: {exc}")
            return
        for offset, value in zip(offsets, values):
            self.state[(dev, offset)] = value
        self._log(f"BULK READ regmap ({len(offsets)} words) -> " +
                   ", ".join(f"0x{o:02X}=0x{v:08X}" for o, v in zip(offsets, values)))
        self._refresh_regmap_display()


def main():
    root = tk.Tk()
    ConsoleApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
