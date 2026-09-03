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
        notebook.add(self.console_tab, text="Command Console")
        notebook.add(self.regmap_tab, text="AXI Regmap")

        self._build_console_tab()
        self._build_regmap_tab()

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
