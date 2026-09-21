#!/usr/bin/env python3
"""
fm_receiver PC-side command console.

Talks the same 8-byte command protocol the board's UART1 console and its
UDP command path both share (see sw/main.c's dispatch_command() and the
protocol doc-comment right above it, and sw/eth0.h for the device/port
constants). Either transport can be selected live from the UI (top bar) --
useful for working around UDP command-reply unreliability under heavy
sample-stream load without losing register access entirely.

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

UART wire format: no preamble/stop-sentinel -- each 8-byte command is sent
alone, and if it's a read (the only thing that ever produces a reply; SYS
never does) its 4-byte big-endian reply comes back before the next command
goes out. Same byte order as the UDP reply (uart1_put32() in sw/main.c is
MSB-first, matching UDP's big-endian framing exactly).

The RX Sample Stream (FFT) tab is always UDP -- it's a receive-only,
high-throughput broadcast path with no UART equivalent in firmware, so the
transport selector doesn't affect it.

Dependencies beyond the Python standard library: numpy, matplotlib,
pyserial (`pip install pyserial` or, on this project's MSYS2 UCRT Python,
`pacman -S mingw-w64-ucrt-x86_64-python-pyserial`).
Run: python pc_console.py
"""

import os
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# sounddevice's ctypes.util.find_library() lookup doesn't honor
# os.add_dll_directory() on Windows -- it needs the DLL's directory on
# PATH itself (confirmed 2026-09-15: add_dll_directory alone still
# raised "PortAudio library not found"). Prepend UCRT64's bin/ (where
# `pacman -S mingw-w64-ucrt-x86_64-python-sounddevice` puts
# libportaudio.dll) before importing, so this works regardless of
# whether the launching shell already has it on PATH. Optional feature:
# if sounddevice genuinely isn't installed, the audio checkbox below
# just stays disabled rather than crashing the whole console.
try:
    _ucrt_bin = r"C:\msys64\ucrt64\bin"
    if os.path.isdir(_ucrt_bin) and _ucrt_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = _ucrt_bin + os.pathsep + os.environ.get("PATH", "")
    import sounddevice as sd
    AUDIO_AVAILABLE = True
except Exception as _exc:  # noqa: BLE001 -- genuinely any failure here should just disable audio, not crash the console
    sd = None
    AUDIO_AVAILABLE = False
    _AUDIO_IMPORT_ERROR = str(_exc)

# Reuse the sample-stream receiver/unpacking logic verbatim rather than
# duplicating it -- see sample_stream_view.py's own docstring for the full
# wire-format derivation (AXI byte-lane mapping -> per-sample field order).
# Binds a *different* UDP port (SAMPLE_PORT=5556) than this console's own
# command socket (which never binds at all, just sends from an ephemeral
# port and reads the reply back on it) -- the two can't collide, and
# sample_stream_view.py can still be run standalone alongside this tab, or
# instead of it, freely.
from sample_stream_view import (
    SampleReceiver, CHANNEL_NAMES, SAMPLE_PORT, DEFAULT_FFT_SIZE, cast_signed,
)

# Display-only labels for CHANNEL_NAMES -- CHANNEL_NAMES itself stays as-is
# (it's also the dict key into SampleReceiver's buffers), this only renames
# what's shown in plot titles.
#
# 2026-09-19: dsp.sv's debug bus is back to real R/L audio (mono/ster
# diagnostic swap reverted), and ch1_i now carries rds.sv's properly
# decimated+scaled RDS output instead of the old raw/unfiltered tap.
#
# 2026-09-21: ch1_i no longer carries a signal at all -- dsp.sv sends the
# decoded RDS registers as {addr, byte} words on it (see RDS_ADDR_* below),
# so it has no FFT panel (and the eye-diagram tab is gone). The audio
# channels arrive already separated by the hardware (R and L).
CHANNEL_DISPLAY_NAMES = {
    "ch0_i": "Audio R",
    "ch0_q": "Audio L",
    "ch1_i": "RDS data (addr:byte words)",
    "ch1_q": "VCO1 (sin)",  # dsp.sv's raw pll_vco1_sin, a direct PLL sanity check.
}

# Channels that get an FFT panel (everything except the RDS data channel).
FFT_CHANNELS = ("ch0_i", "ch0_q", "ch1_q")

# --- Decoded RDS over ch1_i (2026-09-21; mirrors src/dsp.sv's RDS_SCAN) ------
# Every 16-bit sample on that channel is {1'b0, addr[6:0], byte[7:0]}: a
# free-running scan of the FPGA's decoded registers. The PC keeps a byte
# table indexed by addr, so no framing/sync is needed and lost samples are
# simply refreshed on the next sweep.
RDS_CHANNEL        = "ch1_i"
RDS_ADDR_PI        = 0    # 2 bytes: PI high, PI low
RDS_ADDR_PS        = 2    # 8 bytes: Programme Service name
RDS_ADDR_RT        = 10   # 64 bytes: RadioText
RDS_ADDR_STATUS    = 74   # bit 0 = block-sync lock
RDS_N_ENTRIES      = RDS_ADDR_STATUS + 1
RDS_PS_LEN         = 8
RDS_RT_LEN         = 64


def rds_apply_words(table, seen, words):
    """Write a batch of raw 16-bit samples ({addr, byte}) into the byte
    `table` and mark `seen`, in order (the newest sample for an address
    wins). Samples whose address is outside the map are ignored."""
    for w in words:
        w = int(w)
        addr = (w >> 8) & 0xFF
        if addr < RDS_N_ENTRIES:
            table[addr] = w & 0xFF
            seen[addr] = True


def rds_text(chars):
    """Printable view of decoded characters: 0x00 (not received yet) shows
    as '_', other non-printables as '?'."""
    out = []
    for b in chars:
        b = int(b)
        out.append("_" if b == 0 else (chr(b) if 32 <= b < 127 else "?"))
    return "".join(out)


def rds_summary(table, seen):
    """(pi_text, ps_text, rt_text, status_text) for the display panel."""
    n_seen = sum(1 for i in range(RDS_N_ENTRIES) if seen[i])
    if n_seen < RDS_N_ENTRIES:
        waiting = f"waiting for the RDS data channel ({n_seen}/{RDS_N_ENTRIES} entries seen)"
        return "----", "--------", "", waiting
    pi = (int(table[RDS_ADDR_PI]) << 8) | int(table[RDS_ADDR_PI + 1])
    ps = rds_text(table[RDS_ADDR_PS:RDS_ADDR_PS + RDS_PS_LEN])
    rt = rds_text(table[RDS_ADDR_RT:RDS_ADDR_RT + RDS_RT_LEN]).rstrip("_")
    locked = bool(table[RDS_ADDR_STATUS] & 1)
    return f"0x{pi:04X}", ps, rt if rt else "(none yet)", "LOCKED" if locked else "not locked"

# Which sample-stream channel carries the raw pll_vco1_sin tap -- the FFT
# tab annotates this one panel with the pilot tone's measured frequency
# and purity (2026-09-18), since that's the one channel that's supposed to
# be a single clean sinusoid and nothing else.
TONE_MONITOR_CHANNEL = "ch1_q"


def tone_peak_and_purity(freqs_khz, spectrum):
    """Peak frequency (kHz, parabolic-interpolated for sub-bin accuracy)
    and purity (dB) of the dominant tone in `spectrum` (complex rfft
    output, DC-first). Purity is 10*log10(tone power / everything-else
    power): tone power is the peak bin plus its immediate neighbors (to
    capture the Hanning window's own main-lobe spread, not just the
    single tallest bin), everything-else is the rest of the spectrum
    excluding DC. High dB = a clean single tone; low dB = a noisy/
    unlocked one."""
    power = np.abs(spectrum) ** 2
    power = power.copy()
    power[0] = 0.0  # exclude DC from both the peak search and the totals
    peak_idx = int(np.argmax(power))

    if 0 < peak_idx < len(power) - 1:
        # Standard 3-point parabolic interpolation on log-magnitude.
        y0 = np.log(power[peak_idx - 1] + 1e-30)
        y1 = np.log(power[peak_idx] + 1e-30)
        y2 = np.log(power[peak_idx + 1] + 1e-30)
        denom = y0 - 2 * y1 + y2
        delta = 0.5 * (y0 - y2) / denom if denom != 0 else 0.0
        delta = float(np.clip(delta, -0.5, 0.5))
        bin_width = freqs_khz[1] - freqs_khz[0]
        peak_freq_khz = freqs_khz[peak_idx] + delta * bin_width
    else:
        peak_freq_khz = float(freqs_khz[peak_idx])

    lo = max(0, peak_idx - 1)
    hi = min(len(power), peak_idx + 2)
    tone_power = float(np.sum(power[lo:hi]))
    total_power = float(np.sum(power))
    noise_power = max(total_power - tone_power, 1e-30)
    purity_db = 10.0 * np.log10(tone_power / noise_power)
    return peak_freq_khz, purity_db

# --- Stereo (L/R) audio playback (2026-09-18) -------------------------------
#
# ch0_i/ch0_q now carry dsp.sv's real stereo audio directly: mpx_fir.sv +
# mpx_demod.sv do the lowpass filtering, decimation to 48kHz, and L/R
# matrix combine in hardware (ch0_i=audio_r, ch0_q=audio_l, dsp.sv's
# debug-bus packing), gated by mpx_demod_valid -- no software filtering
# or decimation needed here anymore, just playback of what's already
# arriving at the right rate.
AUDIO_OUT_RATE_HZ = 48_000

# --- Protocol constants (mirrors sw/eth0.h / sw/main.c exactly) -----------

BOARD_IP = "192.168.3.50"
UDP_CMD_PORT = 5555
UDP_CMD_PREAMBLE = b"COM\x00"
STOP_SENTINEL = b"\xff" * 8

UART_PORT_DEFAULT = "COM5"
UART_BAUD = 115200

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
    "RXLO (RX LO freq, Hz)": 0x28,
    "GAIN (RX gain control)": 0x2C,
}
CMD_DEV_SYS = 0x04
CMD_DEV_RXLO = 0x28
CMD_DEV_GAIN = 0x2C

# SYS_MODE_* values sw/main.c's enter_test_mode()/enter_mission_mode()
# expect on a CMD_DEV_SYS write (see sw/main.c's own #defines) -- only the
# two normal-operation modes get buttons; the BIST/debug SYS_MODE_* values
# stay reachable via the generic Send command below.
SYS_MODE_TEST = 0x01
SYS_MODE_MISSION = 0x02

# CMD_DEV_GAIN sub-addresses (see sw/main.c's dispatch_command()) -- addr
# 0x00 selects the gain control mode, addr 0x04 sets the manual gain table
# index. Mode values are ADI's rf_gain_ctrl_mode enum (ad9361.h): only mode
# select + the manual index are wired up (sw/main.c's
# ad9361_set_rx_gain_mode()/ad9361_set_rx_manual_gain()) -- picking an AGC
# mode here selects the *behavior*, not a tuned AGC response (no step-size/
# overload-threshold setup exists yet, see sw/main.c's own comment).
GAIN_ADDR_MODE = 0x00
GAIN_ADDR_LEVEL = 0x04
GAIN_MODES = {
    "Manual (MGC)": 0,
    "Fast-attack AGC": 1,
    "Slow-attack AGC": 2,
    "Hybrid AGC": 3,
}
GAIN_LEVEL_MAX = 76  # AD9361_GAIN_TABLE_SIZE-1, sw/main.c -- ~73dB at this index for the 0-1.3GHz band

# PLL lock status -- raw AD9361 SPI register reads (via CMD_DEV_SPI, same
# path the generic Send command panel uses), not anything sw/main.c
# exposes specially. Added 2026-09-14 to have a direct, on-demand answer
# ("is the chip's own PLL actually unlocked right now") the next time the
# sample-stream rate collapses mid-session, instead of re-theorizing from
# scratch -- see ad9361.h for both bit definitions.
PLL_LOCK_REGS = {
    "BBPLL_LOCK":  (0x05E, 0x80),  # REG_CH_1_OVERFLOW bit7
    "RX VCO_LOCK": (0x247, 0x02),  # REG_RX_CP_OVERRANGE_VCO_LOCK bit1
}

# RX LO tuning range accepted by ad9361_rx_lo_synth_set() (sw/main.c) --
# a wider guard band than the actual FM broadcast band (78-108MHz-ish),
# not itself a tuning limit.
RX_LO_MIN_MHZ = 60.0
RX_LO_MAX_MHZ = 130.0

# --- Station tuning through the NCO phase step (AXI RM 0x0C), 2026-09-21 -----
# The AD9361 RX LO stays fixed at 98MHz (nothing in this tool touches it any
# more), and the station is selected digitally: dsp.sv's mixer computes
# (i + jq) * (cos - j sin) of the NCO phase, so a POSITIVE phase step moves a
# station above the LO down to DC. With the phase accumulator 32 bits wide
# and clocked at dsp_clk (30.000MHz, i.e. 1.2MHz x 25 -- the measured
# 9.6MB/s stream), a station offset from the LO maps to
#     phase_step = round(offset_hz / DSP_CLK_HZ * 2^32)   (mod 2^32)
# Offset 0 (station == LO, 98MHz) is phase_step 0, which is the startup state
# this tool assumes -- it never reads or writes 0x0C until a button is pressed.
STATION_CENTER_HZ = 98_000_000
DSP_CLK_HZ = 30_000_000
AXI_ADDR_PHASE_STEP = 0x0C
# Fine/medium/coarse steps, in kHz, shown left to right as the six buttons.
TUNE_STEPS_KHZ = (-250, -25, -10, 10, 25, 250)
# The mixer sees +-dsp_clk/2 around the LO (83-113MHz). +15000kHz would land
# exactly on Nyquist where the phase word wraps onto -15000kHz, so the upper
# limit stops one step short of it.
TUNE_MIN_OFFSET_KHZ = -DSP_CLK_HZ // 2000
TUNE_MAX_OFFSET_KHZ = DSP_CLK_HZ // 2000 - 10


def station_phase_step(offset_hz):
    """Two's-complement 32-bit NCO phase step for a station `offset_hz`
    away from the (fixed) RX LO."""
    return round(offset_hz * (1 << 32) / DSP_CLK_HZ) & 0xFFFFFFFF


def phase_step_to_offset_hz(word):
    """Inverse of station_phase_step(): the frequency a raw 32-bit phase
    step (as read back from 0x0C) actually represents."""
    word &= 0xFFFFFFFF
    if word & 0x80000000:
        word -= 1 << 32
    return word * DSP_CLK_HZ / (1 << 32)

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
    (0x0C, "Phase Step (NCO)", [
        ("Phase step (bits[31:0])", 0xFFFFFFFF, 0),
    ]),
    # dsp.sv's cfg0 -- keep this in sync with dsp.sv's own top-of-file
    # "cfg0[31:0] documentation" comment, the source of truth for what
    # each bit does.
    (0x10, "DSP Config (cfg0)", [
        ("AFC loop enable (bit0)",        0x1, 0),
        ("Discriminator reset (bit28)",   0x1, 28),
        ("Decimator reset (bit29)",       0x1, 29),
        ("PLL reset (bit30)",             0x1, 30),
        ("Demodulator reset (bit31)",     0x1, 31),
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


class UartLink:
    """Same command protocol, over UART1 instead of UDP (sw/main.c's
    process_uart_command()) -- no preamble/stop-sentinel, no reply
    batching: each 8-byte command is written and, if it's a read (the
    only thing that ever produces a reply -- SYS never does, even though
    it's always a write), its 4-byte big-endian reply is read back before
    the next command goes out. TimeoutError here is the same class as
    socket.timeout on this Python version, so existing `except
    socket.timeout` call sites already catch it with no changes needed."""

    def __init__(self, port=UART_PORT_DEFAULT, baud=UART_BAUD, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        # main()'s boot sequence blasts two unsolicited 4-byte words over
        # UART1 with no framing (phy_get_rtl_identifier(), phy_get_link_
        # status()) -- harmless to a terminal emulator, but if they're
        # still sitting in the OS receive buffer when a fresh connection
        # opens (i.e. this client connects after boot already happened),
        # the first send_commands() read would consume them as part of
        # the expected reply, permanently offsetting every reply after
        # that for the life of this connection -- every register would
        # look like it reads back wrong/unwritable, with no error ever
        # raised. Confirmed reproducible 2026-09-13: a fresh connection
        # read back garbage-but-plausible register values; reopening the
        # connection (this fix) made every read/write correct again.
        self.ser.reset_input_buffer()

    def pack_command(self, dev, rw, addr, data):
        return bytes([dev & 0xFF, rw & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]) \
            + struct.pack(">I", data & 0xFFFFFFFF)

    def send_commands(self, commands, expected_replies):
        results = []
        for cmd in commands:
            self.ser.write(cmd)
            dev, rw = cmd[0], cmd[1]
            produces_reply = (rw == CMD_RW_READ) and (dev != CMD_DEV_SYS)
            if produces_reply:
                data = self.ser.read(4)
                if len(data) < 4:
                    raise TimeoutError(f"no UART reply (got {len(data)} of 4 bytes)")
                results.append(struct.unpack(">I", data)[0])
        if len(results) != expected_replies:
            raise ValueError(
                f"expected {expected_replies} reply word(s), got {len(results)}"
            )
        return results

    def close(self):
        self.ser.close()


class ConsoleApp:
    def __init__(self, root):
        self.root = root
        root.title("fm_receiver PC console")

        # self.link is shared by the console tab's Send button and the
        # regmap tab's bulk read -- both go through whichever transport is
        # currently selected (see _build_transport_bar). The sample-stream
        # tab is unaffected: it's a UDP-only, receive-only path with no
        # UART equivalent in firmware, always independent of this choice.
        self.link = BoardLink()
        # (dev, addr) -> last known 32-bit value, updated from every
        # response this tool has ever seen. Only ever written from a real
        # reply, never guessed -- see the "no reason to mistrust it" call.
        self.state = {}

        # Toggle buttons across every tab that shows sample-stream data
        # (the FFT tab; the eye-diagram tab was removed 2026-09-21) -- all
        # drive the one shared SampleReceiver (self.sample_recv), so their
        # labels are kept in sync together rather than each tab owning its
        # own receiver/socket.
        self._sample_toggle_btns = []
        self._rate_last_t = 0.0
        self._rate_last_pkts = 0
        self._rate_last_bytes = 0
        # Cumulative-since-listening-started counterparts (2026-09-18):
        # the per-tick instantaneous rate above is too noisy (~200ms of
        # packet/OS-scheduling jitter) to tell a real sample-rate
        # mismatch (e.g. true rate vs. the 48kHz the audio path assumes)
        # from normal jitter -- averaging over a long, growing window
        # cancels that jitter out. See _sample_update_body().
        self._rate_start_t = 0.0
        self._rate_start_bytes = 0

        self._build_transport_bar(root)

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
    def _build_transport_bar(self, root):
        bar = ttk.LabelFrame(root, text="Command transport (applies to Send + bulk regmap read)")
        bar.pack(fill="x", padx=8, pady=(8, 0))

        self.transport_var = tk.StringVar(value="udp")
        ttk.Radiobutton(bar, text="Ethernet (UDP)", variable=self.transport_var,
                         value="udp", command=self._on_transport_change).pack(side="left", padx=(4, 4), pady=4)
        ttk.Radiobutton(bar, text="UART", variable=self.transport_var,
                         value="uart", command=self._on_transport_change).pack(side="left", padx=4, pady=4)

        ttk.Label(bar, text="COM port:").pack(side="left", padx=(16, 4))
        self.uart_port_var = tk.StringVar(value=UART_PORT_DEFAULT)
        ttk.Entry(bar, textvariable=self.uart_port_var, width=8).pack(side="left", padx=(0, 4))

        self.transport_status_var = tk.StringVar(value="Using UDP (Ethernet)")
        ttk.Label(bar, textvariable=self.transport_status_var).pack(side="left", padx=(16, 4))

    def _on_transport_change(self):
        want = self.transport_var.get()
        old_link = self.link
        try:
            if want == "uart":
                new_link = UartLink(port=self.uart_port_var.get())
            else:
                new_link = BoardLink()
        except (OSError, serial.SerialException) as exc:
            # Revert the radio selection -- old_link is still open and
            # usable, so don't tear it down over a failed switch.
            self.transport_var.set("uart" if want == "udp" else "udp")
            self.transport_status_var.set(f"ERROR opening {want}: {exc}")
            return
        old_link.close()
        self.link = new_link
        self.transport_status_var.set(
            f"Using UART ({self.uart_port_var.get()})" if want == "uart" else "Using UDP (Ethernet)"
        )

    # ------------------------------------------------------------------
    def _build_console_tab(self):
        f = self.console_tab

        # Station tuning (2026-09-21): replaces the old "Tune RX LO" frame (the
        # AD9361 LO is fixed at 98MHz now, so that control is no longer shown;
        # _on_tune_rx_lo() below is kept only as history). The six buttons move
        # the station by a fixed step and write the matching NCO phase step to
        # AXI 0x0C -- see the STATION_*/TUNE_* constants for the conversion.
        tune_frame = ttk.LabelFrame(f, text="Station tuning")
        tune_frame.pack(fill="x", padx=8, pady=(8, 0))
        self.station_offset_khz = 0          # startup state: station == LO, phase step 0
        self.station_freq_var = tk.StringVar(value=self._station_text())
        ttk.Label(tune_frame, textvariable=self.station_freq_var,
                  font=("Consolas", 16, "bold")).grid(row=0, column=0, columnspan=8, sticky="w", padx=8, pady=(4, 0))
        for col, step in enumerate(TUNE_STEPS_KHZ):
            ttk.Button(tune_frame, text=f"{step:+d}", width=7,
                       command=lambda s=step: self._on_station_step(s)).grid(row=1, column=col, padx=4, pady=6)
        ttk.Label(tune_frame, text="kHz").grid(row=1, column=len(TUNE_STEPS_KHZ), sticky="w", padx=(2, 8))
        self.station_status_var = tk.StringVar(value="phase step 0x00000000 (not written yet)")
        ttk.Label(tune_frame, textvariable=self.station_status_var).grid(
            row=2, column=0, columnspan=8, sticky="w", padx=8, pady=(0, 4))

        mode_frame = ttk.LabelFrame(f, text="System mode")
        mode_frame.pack(fill="x", padx=8, pady=(8, 0))
        ttk.Button(mode_frame, text="Mission mode", command=self._on_mission_mode) \
            .grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(mode_frame, text="Test mode", command=self._on_test_mode) \
            .grid(row=0, column=1, padx=4, pady=4)
        self.mode_status_var = tk.StringVar(value="")
        ttk.Label(mode_frame, textvariable=self.mode_status_var).grid(row=0, column=2, sticky="w", padx=8)

        gain_frame = ttk.LabelFrame(f, text="RX gain control")
        gain_frame.pack(fill="x", padx=8, pady=(8, 0))
        ttk.Label(gain_frame, text="Mode:").grid(row=0, column=0, sticky="e", padx=4, pady=4)
        self.gain_mode_var = tk.StringVar(value=list(GAIN_MODES.keys())[0])
        gain_mode_combo = ttk.Combobox(gain_frame, textvariable=self.gain_mode_var,
                                        values=list(GAIN_MODES.keys()), state="readonly", width=18)
        gain_mode_combo.grid(row=0, column=1, padx=4, pady=4)
        gain_mode_combo.bind("<<ComboboxSelected>>", self._on_gain_mode_change)

        ttk.Label(gain_frame, text="Manual level (0-76):").grid(row=0, column=2, sticky="e", padx=(16, 4), pady=4)
        self.gain_level_var = tk.IntVar(value=60)
        gain_level_scale = ttk.Scale(gain_frame, from_=0, to=GAIN_LEVEL_MAX, orient="horizontal",
                                      variable=self.gain_level_var, length=160,
                                      command=self._on_gain_level_drag)
        gain_level_scale.grid(row=0, column=3, padx=4, pady=4)
        # Send only on release, not on every drag tick -- a live-dragged
        # Scale's command fires continuously, which would otherwise flood
        # the board with a SPI write (and gain-table lookup) per pixel of
        # mouse movement.
        gain_level_scale.bind("<ButtonRelease-1>", self._on_gain_level_release)
        self.gain_level_label_var = tk.StringVar(value="60")
        ttk.Label(gain_frame, textvariable=self.gain_level_label_var, width=4).grid(row=0, column=4, padx=(0, 8))

        self.gain_status_var = tk.StringVar(value="")
        ttk.Label(gain_frame, textvariable=self.gain_status_var).grid(row=1, column=0, columnspan=5, sticky="w", padx=4)

        pll_frame = ttk.LabelFrame(f, text="PLL lock status (on demand -- not polled automatically)")
        pll_frame.pack(fill="x", padx=8, pady=(8, 0))
        ttk.Button(pll_frame, text="Check now", command=self._on_check_pll_lock).grid(row=0, column=0, padx=4, pady=4)
        self.pll_lock_var = tk.StringVar(value="Not checked yet")
        ttk.Label(pll_frame, textvariable=self.pll_lock_var).grid(row=0, column=1, sticky="w", padx=8)

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

    def _station_text(self):
        """Current station shown in the tuning frame, from the tracked offset."""
        mhz = (STATION_CENTER_HZ + self.station_offset_khz * 1000) / 1e6
        return f"{mhz:.3f} MHz"

    def _on_station_step(self, delta_khz):
        """One of the six tuning buttons: move the station by delta_khz, write
        the new NCO phase step to AXI 0x0C (write + readback, like the Send
        panel) and only then update the shown frequency. If the board does not
        answer, the shown frequency is left alone and the status line says the
        board may no longer match it."""
        new_khz = max(TUNE_MIN_OFFSET_KHZ, min(TUNE_MAX_OFFSET_KHZ, self.station_offset_khz + delta_khz))
        if new_khz == self.station_offset_khz:
            self.station_status_var.set("At the limit of the tunable range (83-113 MHz)")
            return
        word = station_phase_step(new_khz * 1000)
        dev = DEVICES["AXI (axi_registers)"]
        try:
            write_cmd = self.link.pack_command(dev, CMD_RW_WRITE, AXI_ADDR_PHASE_STEP, word)
            read_cmd = self.link.pack_command(dev, CMD_RW_READ, AXI_ADDR_PHASE_STEP, 0)
            (readback,) = self.link.send_commands([write_cmd, read_cmd], expected_replies=1)
        except socket.timeout:
            self.station_status_var.set("No reply from the board -- shown frequency may not match it")
            self._log(f"TIMEOUT tuning by {delta_khz:+d} kHz (phase step 0x{word:08X})")
            return
        except (OSError, ValueError) as exc:
            self.station_status_var.set(f"ERROR: {exc}")
            self._log(f"ERROR tuning by {delta_khz:+d} kHz: {exc}")
            return
        self.station_offset_khz = new_khz
        self.state[(dev, AXI_ADDR_PHASE_STEP)] = readback
        self.station_freq_var.set(self._station_text())
        ok = "readback OK" if readback == word else f"READBACK MISMATCH 0x{readback:08X}"
        self.station_status_var.set(f"{delta_khz:+d} kHz -> phase step 0x{word:08X} ({ok})")
        self._log(f"TUNE {delta_khz:+d} kHz -> {self._station_text()} "
                  f"(offset {new_khz:+d} kHz, phase step 0x{word:08X}, readback 0x{readback:08X})")
        self._refresh_regmap_display()

    def _on_tune_rx_lo(self):
        # UI removed 2026-09-21 (RX LO is fixed at 98MHz); kept for history and
        # NOT wired to anything -- its rx_lo_* variables no longer exist.
        # ad9361_rx_lo_synth_set() (sw/main.c) is NOT YET hardware-verified
        # for any frequency other than the 98MHz boot default -- see
        # private/ad9361_registers.md's "RX LO frequency tuning" section.
        try:
            mhz = float(self.rx_lo_mhz_var.get())
        except ValueError:
            self.rx_lo_status_var.set(f"ERROR: '{self.rx_lo_mhz_var.get()}' is not a number")
            return
        if not (RX_LO_MIN_MHZ <= mhz <= RX_LO_MAX_MHZ):
            self.rx_lo_status_var.set(
                f"ERROR: {mhz}MHz outside accepted range [{RX_LO_MIN_MHZ}, {RX_LO_MAX_MHZ}]MHz"
            )
            return
        freq_hz = round(mhz * 1_000_000)
        try:
            cmd = self.link.pack_command(CMD_DEV_RXLO, CMD_RW_WRITE, 0, freq_hz)
            self.link.send_commands([cmd], expected_replies=0)
        except (OSError, socket.timeout) as exc:
            self.rx_lo_status_var.set(f"ERROR: {exc}")
            self._log(f"ERROR tuning RX LO to {mhz}MHz: {exc}")
            return
        self.rx_lo_status_var.set(f"Sent: {mhz}MHz ({freq_hz}Hz). Check REG 0x247 bit1 (VCO_LOCK) via SPI read.")
        self._log(f"TUNE RX LO -> {mhz}MHz ({freq_hz}Hz)")

    def _set_mode(self, mode, label):
        try:
            cmd = self.link.pack_command(CMD_DEV_SYS, CMD_RW_WRITE, 0, mode)
            self.link.send_commands([cmd], expected_replies=0)
        except (OSError, socket.timeout) as exc:
            self.mode_status_var.set(f"ERROR: {exc}")
            self._log(f"ERROR setting {label}: {exc}")
            return
        self.mode_status_var.set(f"Sent: {label}")
        self._log(f"SYS MODE -> {label}")

    def _on_mission_mode(self):
        self._set_mode(SYS_MODE_MISSION, "Mission mode")

    def _on_test_mode(self):
        self._set_mode(SYS_MODE_TEST, "Test mode")

    def _on_gain_mode_change(self, _event=None):
        mode_val = GAIN_MODES[self.gain_mode_var.get()]
        try:
            cmd = self.link.pack_command(CMD_DEV_GAIN, CMD_RW_WRITE, GAIN_ADDR_MODE, mode_val)
            self.link.send_commands([cmd], expected_replies=0)
        except (OSError, socket.timeout) as exc:
            self.gain_status_var.set(f"ERROR: {exc}")
            self._log(f"ERROR setting gain mode {self.gain_mode_var.get()}: {exc}")
            return
        self.gain_status_var.set(f"Mode: {self.gain_mode_var.get()}")
        self._log(f"GAIN MODE -> {self.gain_mode_var.get()} ({mode_val})")

    def _on_gain_level_drag(self, value_str):
        # Label-only update while dragging -- the actual send happens on
        # release (_on_gain_level_release), see the Scale's binding above.
        self.gain_level_label_var.set(str(int(float(value_str))))

    def _on_gain_level_release(self, _event=None):
        level = int(self.gain_level_var.get())
        try:
            cmd = self.link.pack_command(CMD_DEV_GAIN, CMD_RW_WRITE, GAIN_ADDR_LEVEL, level)
            self.link.send_commands([cmd], expected_replies=0)
        except (OSError, socket.timeout) as exc:
            self.gain_status_var.set(f"ERROR: {exc}")
            self._log(f"ERROR setting manual gain level {level}: {exc}")
            return
        self.gain_status_var.set(f"Manual level: {level}")
        self._log(f"GAIN LEVEL -> {level}")

    def _on_check_pll_lock(self):
        dev = DEVICES["SPI (AD9361)"]
        results = []
        try:
            for name, (addr, mask) in PLL_LOCK_REGS.items():
                cmd = self.link.pack_command(dev, CMD_RW_READ, addr, 0)
                (val,) = self.link.send_commands([cmd], expected_replies=1)
                locked = bool(val & mask)
                results.append(f"{name}={'LOCKED' if locked else 'UNLOCKED'} (0x{val:02X})")
        except (OSError, socket.timeout) as exc:
            self.pll_lock_var.set(f"ERROR: {exc}")
            self._log(f"ERROR checking PLL lock: {exc}")
            return
        summary = "  ".join(results)
        self.pll_lock_var.set(summary)
        self._log(f"PLL LOCK CHECK -> {summary}")

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

        # Audio pipeline state (2026-09-15) -- see _on_audio_toggle()/
        # _audio_worker(). audio_queue is created once and handed to
        # whatever SampleReceiver exists at the moment audio gets
        # enabled (see set_audio_queue() on the receiver side); the
        # worker thread itself only runs while audio_enabled_var is set.
        self.audio_queue = queue.Queue(maxsize=200)  # ~200 packets worth --a few seconds' slack before dropping
        self._audio_thread = None
        self._audio_stop = threading.Event()

        ctrl = ttk.Frame(f)
        ctrl.pack(fill="x", padx=8, pady=8)
        self.sample_status_var = tk.StringVar(value="Not listening")
        ttk.Label(ctrl, textvariable=self.sample_status_var).pack(side="left")
        sample_toggle_btn = ttk.Button(ctrl, text="Start listening", command=self._toggle_sample_stream)
        sample_toggle_btn.pack(side="right")
        self._sample_toggle_btns.append(sample_toggle_btn)

        # Editable sample rate driving the FFT frequency axis, defaulting
        # to 48kHz (2026-09-18) -- the debug bus now carries real stereo
        # audio at AUDIO_OUT_RATE_HZ, not the old raw decimated sample
        # rate (SAMPLE_RATE_HZ, from sample_stream_view.py), so that's
        # the more useful default now. Not read from the board; purely a
        # display setting, so it's safe to change freely.
        self.sample_rate_hz = AUDIO_OUT_RATE_HZ
        ttk.Label(ctrl, text="Sample rate (Hz):").pack(side="left", padx=(16, 4))
        self.sample_rate_var = tk.StringVar(value=f"{AUDIO_OUT_RATE_HZ:.0f}")
        rate_entry = ttk.Entry(ctrl, textvariable=self.sample_rate_var, width=12)
        rate_entry.pack(side="left")
        rate_entry.bind("<Return>", self._on_sample_rate_change)
        rate_entry.bind("<FocusOut>", self._on_sample_rate_change)

        # Reinterprets the same raw 16-bit wire bits as signed (two's
        # complement, the real format) or unsigned -- a diagnostic toggle,
        # not a "pick whichever looks right" setting. Applied fresh every
        # _sample_update() tick (see cast_signed()), never baked into the
        # receiver's stored buffer -- so it takes effect immediately and
        # identically across every consumer of the buffers, with no risk of
        # a buffer holding a stale mix of both interpretations. No command
        # callback needed: the next poll tick (<=200ms) just reads this
        # StringVar fresh.
        ttk.Label(ctrl, text="Cast as:").pack(side="left", padx=(16, 4))
        self.sample_signed_var = tk.StringVar(value="signed")
        ttk.Radiobutton(ctrl, text="Signed", variable=self.sample_signed_var,
                         value="signed").pack(side="left")
        ttk.Radiobutton(ctrl, text="Unsigned", variable=self.sample_signed_var,
                         value="unsigned").pack(side="left")

        # Audio (2026-09-15): off by default on purpose -- opening this
        # tab (or starting the console at all) must never start making
        # noise unprompted. See _on_audio_toggle() for the pipeline.
        self.audio_enabled_var = tk.BooleanVar(value=False)
        audio_check = ttk.Checkbutton(
            ctrl, text=f"Audio (stereo, {AUDIO_OUT_RATE_HZ//1000}kHz)",
            variable=self.audio_enabled_var, command=self._on_audio_toggle,
        )
        audio_check.pack(side="left", padx=(16, 4))
        if not AUDIO_AVAILABLE:
            audio_check.state(["disabled"])
            self.audio_status_var = tk.StringVar(value=f"Audio unavailable: {_AUDIO_IMPORT_ERROR}")
        else:
            self.audio_status_var = tk.StringVar(value="")
        ttk.Label(ctrl, textvariable=self.audio_status_var).pack(side="left")

        # Decoded RDS (2026-09-21): packed at the bottom BEFORE the canvas so it
        # always keeps its size and the plots take whatever is left. Filled by
        # _rds_update() from the ch1_i {addr, byte} words (see RDS_ADDR_*).
        rds_box = ttk.LabelFrame(f, text="RDS (decoded on the FPGA)")
        rds_box.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        self.rds_table = bytearray(256)
        self.rds_seen = [False] * 256
        self.rds_pi_var = tk.StringVar(value="----")
        self.rds_ps_var = tk.StringVar(value="--------")
        self.rds_rt_var = tk.StringVar(value="")
        self.rds_status_var = tk.StringVar(value="not listening")
        mono = ("Courier New", 11)
        for row, (label, var) in enumerate((("PI", self.rds_pi_var), ("PS", self.rds_ps_var),
                                            ("RT", self.rds_rt_var), ("Status", self.rds_status_var))):
            ttk.Label(rds_box, text=f"{label}:", font=mono, width=7).grid(row=row, column=0, sticky="w", padx=(8, 0))
            ttk.Label(rds_box, textvariable=var, font=mono).grid(row=row, column=1, sticky="w", padx=8)

        fig = Figure(figsize=(10, 4))
        axes = fig.subplots(1, len(FFT_CHANNELS))
        self.sample_freqs_khz = np.fft.rfftfreq(DEFAULT_FFT_SIZE, d=1.0 / AUDIO_OUT_RATE_HZ) / 1e3
        self.sample_window = np.hanning(DEFAULT_FFT_SIZE)
        self.sample_axes = axes
        self.sample_lines = {}
        self.sample_tone_text = None
        for ax, name in zip(axes.flat, FFT_CHANNELS):
            (line,) = ax.plot(self.sample_freqs_khz, np.zeros_like(self.sample_freqs_khz))
            ax.set_title(CHANNEL_DISPLAY_NAMES[name])
            ax.set_xlabel("Freq (kHz)")
            ax.set_ylabel("Magnitude (dB)")
            ax.set_ylim(-20, 100)
            self.sample_lines[name] = line
            if name == TONE_MONITOR_CHANNEL:
                self.sample_tone_text = ax.text(
                    0.98, 0.95, "", transform=ax.transAxes, ha="right", va="top",
                    fontsize=9, family="monospace",
                    bbox=dict(boxstyle="round", facecolor="white", alpha=0.75),
                )
        fig.tight_layout()

        self.sample_canvas = FigureCanvasTkAgg(fig, master=f)
        self.sample_canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=(0, 8))

    # ------------------------------------------------------------------
    def _on_sample_rate_change(self, _event=None):
        try:
            rate_hz = float(self.sample_rate_var.get())
            if rate_hz <= 0:
                raise ValueError
        except ValueError:
            self.sample_rate_var.set(f"{self.sample_rate_hz:.0f}")  # revert to last-good
            return
        self.sample_rate_hz = rate_hz
        self.sample_freqs_khz = np.fft.rfftfreq(DEFAULT_FFT_SIZE, d=1.0 / rate_hz) / 1e3
        for line in self.sample_lines.values():
            line.set_xdata(self.sample_freqs_khz)
        for ax in self.sample_axes.flat:
            ax.set_xlim(self.sample_freqs_khz[0], self.sample_freqs_khz[-1])
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
            self.rds_table[:] = bytes(256)      # fresh RDS mirror for this listening session
            self.rds_seen = [False] * 256
            self._rate_last_t = time.monotonic()
            self._rate_last_pkts = 0
            self._rate_last_bytes = 0
            self._rate_start_t = self._rate_last_t
            self._rate_start_bytes = 0
            for btn in self._sample_toggle_btns:
                btn.config(text="Stop listening")
            self._sample_update()
        else:
            self._stop_sample_stream()

    def _stop_sample_stream(self):
        if self.sample_after_id is not None:
            self.root.after_cancel(self.sample_after_id)
            self.sample_after_id = None
        if self.audio_enabled_var.get():
            # No receiver left to feed the audio queue -- turn audio off
            # too rather than leave the worker thread spinning on an
            # empty queue with nothing coming.
            self.audio_enabled_var.set(False)
            self._on_audio_toggle()
        if self.sample_recv is not None:
            self.sample_recv.stop()
            self.sample_recv = None
        for btn in self._sample_toggle_btns:
            btn.config(text="Start listening")
        self.sample_status_var.set("Not listening")
        self.rds_status_var.set("not listening")

    def _on_audio_toggle(self):
        """Checkbox callback. Off by default -- see _build_sample_tab().
        Just taps the already-streaming ch0_i/ch0_q (real stereo audio,
        see the AUDIO_* constants' header comment) for playback."""
        if self.audio_enabled_var.get():
            if self.sample_recv is None:
                self.audio_status_var.set("Start listening first")
                self.audio_enabled_var.set(False)
                return
            # Drop any backlog so audio starts from "now" rather than
            # racing to catch up through several stale seconds first.
            while True:
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break
            self.sample_recv.set_audio_queue(self.audio_queue)
            self._audio_stop.clear()
            self._audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
            self._audio_thread.start()
            self.audio_status_var.set("Playing")
        else:
            self._audio_stop.set()
            if self.sample_recv is not None:
                self.sample_recv.set_audio_queue(None)
            self.audio_status_var.set("")

    def _audio_worker(self):
        """Runs on its own thread (never the Tk main thread) for the
        whole time audio is enabled: drains audio_queue (stereo [L, R]
        chunks, raw unsigned samples per UDP packet, already at
        AUDIO_OUT_RATE_HZ -- see the AUDIO_* header comment) and streams
        it out via sounddevice. No filtering or decimation needed here
        anymore, that's all done in hardware now."""
        peak_est = 1.0
        stream = None
        try:
            stream = sd.OutputStream(samplerate=AUDIO_OUT_RATE_HZ, channels=2, dtype="float32")
            stream.start()
            while not self._audio_stop.is_set():
                try:
                    chunk = self.audio_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                y = cast_signed(chunk).astype(np.float64)  # shape (N, 2), [L, R]

                # Slowly-adapting peak normalization + hard safety clip:
                # audio_l/audio_r's real-world amplitude was never
                # calibrated to any fixed reference (see project memory,
                # cordic_vector_hardware_stall.md's gain-chain
                # investigation), so this is "don't be silent, don't
                # clip harshly" -- not a precise loudness target. Decays
                # slowly (0.999/chunk) so it doesn't audibly "pump" on
                # every loud transient, but jumps up instantly to a new,
                # louder peak so a sudden loud passage doesn't clip
                # before the estimate catches up.
                chunk_peak = float(np.max(np.abs(y))) if y.size else 0.0
                peak_est = max(chunk_peak, peak_est * 0.999)
                scale = 0.6 / peak_est if peak_est > 1e-6 else 0.0
                out = np.clip(y * scale, -1.0, 1.0).astype(np.float32)
                stream.write(out)
        except Exception as exc:  # noqa: BLE001 -- report and exit cleanly, don't take the console down
            self.audio_status_var.set(f"Audio error: {exc}")
        finally:
            if stream is not None:
                try:
                    stream.stop()
                    stream.close()
                except Exception:
                    pass

    def _sample_update(self):
        if self.sample_recv is None:
            return
        # Whole body wrapped try/finally (2026-09-18): this reschedules
        # itself at the end, so any uncaught exception here previously
        # killed the entire redraw loop silently (FFT, RDS panel, rate
        # display, everything) -- not just whatever pane actually broke.
        # finally guarantees the next tick still fires regardless; the
        # except surfaces the error in the status bar instead of losing
        # it, so a real bug is still visible, just not fatal to the tool.
        try:
            self._sample_update_body()
        except Exception as exc:  # noqa: BLE001 -- must never kill the redraw loop
            self.sample_status_var.set(f"Redraw error (see console): {exc}")
            import traceback
            traceback.print_exc()
        finally:
            self.sample_after_id = self.root.after(200, self._sample_update)

    def _sample_update_body(self):
        # FFT re-enabled 2026-09-13: dsp_clk fix + rx_clk margin confirmed,
        # and rate_counter.py proved the earlier rate cap/jitter was this
        # tool's own GIL/rendering contention, not FPGA/firmware throughput
        # -- so the redraw is no longer suspected as a source of packet loss.
        raw = self.sample_recv.snapshot()
        data = raw
        # Cast applied here, once, fresh every tick, for the FFT loop below.
        # The RDS decoder always works on `raw` (the untouched unsigned
        # words): its samples are {addr, byte} pairs, not signed signals.
        if self.sample_signed_var.get() == "signed":
            data = {name: cast_signed(buf) for name, buf in data.items()}

        for name, line in self.sample_lines.items():
            buf = data[name]
            if len(buf) < DEFAULT_FFT_SIZE:
                continue
            windowed = buf[-DEFAULT_FFT_SIZE:].astype(np.float64)
            windowed = windowed * self.sample_window
            spectrum = np.fft.rfft(windowed)
            line.set_ydata(20 * np.log10(np.abs(spectrum) + 1e-9))
            if name == TONE_MONITOR_CHANNEL and self.sample_tone_text is not None:
                peak_freq_khz, purity_db = tone_peak_and_purity(self.sample_freqs_khz, spectrum)
                self.sample_tone_text.set_text(
                    f"peak: {peak_freq_khz:.3f} kHz\npurity: {purity_db:.1f} dB"
                )
        # Self-adjusting Y limits: a fixed range clips PRBS's much higher
        # noise floor compared to a clean tone. relim() recomputes data
        # limits from each line's just-updated ydata; scalex=False leaves
        # the frequency axis alone (that's driven by the sample-rate field,
        # not the data).
        for ax in self.sample_axes.flat:
            # set_autoscaley_on(True) every tick: the initial ax.set_ylim()
            # call at tab-build time (below) turns autoscale off, and
            # autoscale_view() silently no-ops on an axis with it off.
            ax.set_autoscaley_on(True)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        # Instantaneous rate since the last tick (~200ms), not a cumulative
        # average -- matches what "is the stream actually still flowing"
        # questions actually want to know. Too noisy on its own to read
        # off a true underlying sample rate, though -- see the
        # since-start average below for that.
        now = time.monotonic()
        dt = now - self._rate_last_t
        pkt_rate = (self.sample_recv.packets_received - self._rate_last_pkts) / dt if dt > 0 else 0.0
        byte_rate_mb = (self.sample_recv.bytes_received - self._rate_last_bytes) / dt / 1e6 if dt > 0 else 0.0
        self._rate_last_t = now
        self._rate_last_pkts = self.sample_recv.packets_received
        self._rate_last_bytes = self.sample_recv.bytes_received

        # Cumulative average since listening started: same bytes_received
        # counter, just divided by the whole elapsed time instead of one
        # tick -- the per-tick jitter above averages out, leaving a
        # stable read on the real sample rate (8 bytes/sample: 4 channels
        # x 16 bits, see axi_dsp.sv's debug-bus packing). Settles over
        # the first several seconds; trust it more the longer it's run.
        avg_dt = now - self._rate_start_t
        avg_byte_rate_mb = self.sample_recv.bytes_received / avg_dt / 1e6 if avg_dt > 0 else 0.0
        avg_sample_rate_hz = self.sample_recv.bytes_received / 8.0 / avg_dt if avg_dt > 0 else 0.0

        self.sample_status_var.set(
            f"Listening on :{SAMPLE_PORT} -- {self.sample_recv.packets_received} pkts "
            f"({pkt_rate:.0f}/s), {self.sample_recv.bytes_received / 1e6:.2f} MB "
            f"({byte_rate_mb:.2f} MB/s), {self.sample_recv.packets_dropped} dropped | "
            f"avg over {avg_dt:.0f}s: {avg_byte_rate_mb:.3f} MB/s = {avg_sample_rate_hz:.1f} Hz/ch"
        )
        self.sample_canvas.draw_idle()

        self._rds_update(raw[RDS_CHANNEL])

    def _rds_update(self, raw_words):
        """Fold the latest ch1_i window ({addr, byte} words, oldest to
        newest) into the RDS byte table and refresh the decoded-RDS panel.
        Windows overlap poorly with the ~200ms poll (each is only the last
        1024 samples = 21ms), but the FPGA sweeps all 75 entries every
        1.56ms, so every window carries a complete refresh."""
        if len(raw_words):
            rds_apply_words(self.rds_table, self.rds_seen, raw_words.astype(np.uint16))
        pi, ps, rt, status = rds_summary(self.rds_table, self.rds_seen)
        self.rds_pi_var.set(pi)
        self.rds_ps_var.set(f"'{ps}'" if ps.strip("-") else ps)
        self.rds_rt_var.set(f"'{rt}'" if rt and not rt.startswith("(") else rt)
        self.rds_status_var.set(status)

    def _on_close(self):
        self._stop_sample_stream()
        self.link.close()
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
