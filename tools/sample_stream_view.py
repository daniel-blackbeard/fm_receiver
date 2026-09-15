#!/usr/bin/env python3
"""
Live FFT viewer for fm_receiver's RX sample stream (axi_dsp -> DDR -> GEM0
zero-copy TX -- see private/sample_streaming_plan.md for the full pipeline).

Wire format, one UDP datagram (SAMPLE_DEST_PORT=5556 in sw/eth0.h):
    A plain UDP payload, exactly SAMPLE_STREAM_NOTIF_BYTES=1024 bytes -- no
    header of its own (the Ethernet/IP/UDP headers are already stripped by
    the OS before recvfrom() sees this). Straight DDR bytes, unmodified,
    from wherever axi_dsp last wrote (see eth_send_sample_packet() /
    eth_poll_sample_stream() in sw/eth0.c).

    Those 1024 bytes are 128 back-to-back 8-byte samples. Each 8-byte
    sample is one axi_dsp.sv AXI3 S_AXI_HP0 write word:

        WDATA[63:0] = {ch0_i, ch0_q, ch1_i, ch1_q}   (each 16-bit, MSB-first)

    AXI byte-lane mapping puts the *lowest* addressed byte at WDATA's LSB
    (little-endian steering, the S_AXI_HP0 default), so in DDR memory --
    and therefore in this raw UDP payload, byte-for-byte -- each 8-byte
    sample lands as four little-endian int16 words in this order:

        ch1_q, ch1_i, ch0_q, ch0_i

    (confirmed against axi_dsp.sv's `assign i_data = {ch0_i, ch0_q, ch1_i,
    ch1_q};` and sample_streaming_plan.md's "Packing order" section -- not
    guessed.) Each 16-bit value is signed: the RTL's x8 decimation
    accumulates 8 raw AD9361 samples (12-bit two's complement each) into a
    16-bit running sum, so treat every sample as a plain int16, no scaling.

Known caveat inherited from the RTL (private/sample_streaming_plan.md,
"Deferred RTL TODO"): the decimation accumulator isn't gated by adc_valid
yet, so packet *content* isn't guaranteed to be meaningful real sample data
until that's fixed -- this tool will happily plot whatever comes across.

Effective per-channel sample rate: clk_dsp (~29.8MHz, BBPLL/32,
hardware-measured -- see README.md's AD9361 RX digital bring-up section)
divided by the x8 decimation factor, ~3.725MHz. Used only to label the FFT
frequency axis; not read from hardware. Editable live via the "Sample
rate (Hz)" field above the plots (defaults to this figure) -- e.g. for a
future CORDIC-generated test signal at a different rate than the real
AD9361 decimation chain.

Prereqs: numpy + matplotlib (both already present in this project's
C:\\msys64\\ucrt64 Python -- see CLAUDE.md's MSYS2 UCRT-first tooling
preference). Run: python sample_stream_view.py [fft_size]

Before running: the board must be pointed at *this* machine's IP --
SAMPLE_DEST_IP0-3 in sw/eth0.h -- and this machine's IP must be that same
address for the OS to deliver the (unicast IP, broadcast Ethernet) datagram
up to a UDP socket at all.
"""

import socket
import queue
import struct
import sys
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, RadioButtons

BIND_IP = "0.0.0.0"
SAMPLE_PORT = 5556           # SAMPLE_DEST_PORT, sw/eth0.h
PAYLOAD_BYTES = 1024         # SAMPLE_STREAM_NOTIF_BYTES, sw/eth0.h
SAMPLES_PER_PACKET = PAYLOAD_BYTES // 8   # one 8-byte quad word per sample

DEFAULT_FFT_SIZE = 1024
SAMPLE_RATE_HZ = 29.8e6 / 8.0   # clk_dsp / x8 decimation, see docstring

CHANNEL_NAMES = ("ch0_i", "ch0_q", "ch1_i", "ch1_q")


class SampleReceiver:
    """Background UDP receiver: unpacks each 1KB datagram into four
    per-channel ring buffers holding the last `fft_size` samples. One
    socket, one thread -- matplotlib's animation callback only ever reads
    the ring buffers under the lock, never touches the socket itself."""

    def __init__(self, fft_size=DEFAULT_FFT_SIZE):
        self.fft_size = fft_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((BIND_IP, SAMPLE_PORT))
        self.sock.settimeout(1.0)
        self.buffers = {name: deque(maxlen=fft_size) for name in CHANNEL_NAMES}
        self.lock = threading.Lock()
        self.packets_received = 0
        self.packets_dropped = 0
        self.bytes_received = 0
        self._stop = False
        self._thread = None
        # Opt-in, continuous (non-rolling-window) ch0_i sample feed for
        # pc_console.py's audio pipeline (2026-09-15) -- separate from
        # `buffers` above on purpose: those are fixed-size rolling
        # windows for the FFT/eye-diagram displays (each redraw just
        # wants "the last fft_size samples", repeats are fine), but audio
        # needs every sample exactly once, in order, with no gaps -- a
        # rolling snapshot would silently drop the vast majority of
        # samples between polls. None (the default) means "don't
        # bother" -- _run() skips the audio path entirely, no extra
        # per-packet cost when nothing's listening for it.
        self.audio_queue = None

    def set_audio_queue(self, q):
        """q: a queue.Queue (or None to disable) that will receive each
        packet's raw ch0_i samples (unsigned, one small int array per
        packet) as they arrive. Safe to call at any time, whether or not
        the receiver thread is currently running."""
        with self.lock:
            self.audio_queue = q

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        self.sock.close()

    def _run(self):
        while not self._stop:
            try:
                data, _addr = self.sock.recvfrom(2048)
            except (socket.timeout, OSError):
                continue
            if len(data) != PAYLOAD_BYTES:
                # Not a valid 1KB sample slice -- e.g. a stray packet on
                # this port from something else. Drop rather than
                # mis-unpack it as sample data.
                self.packets_dropped += 1
                continue
            # Always unsigned ('H') -- the raw bit pattern, untouched.
            # signed/unsigned is a *display* cast (see cast_signed()
            # below), not a receipt-time choice: applying it here would
            # bake whichever interpretation was active at receipt into
            # the stored buffer, so toggling mid-stream would leave old-
            # and new-interpretation samples mixed together until the
            # whole buffer cycled out.
            samples = struct.unpack(f"<{SAMPLES_PER_PACKET * 4}H", data)
            with self.lock:
                ch0_i = samples[3::4]
                self.buffers["ch1_q"].extend(samples[0::4])
                self.buffers["ch1_i"].extend(samples[1::4])
                self.buffers["ch0_q"].extend(samples[2::4])
                self.buffers["ch0_i"].extend(ch0_i)
                self.packets_received += 1
                self.bytes_received += len(data)
                if self.audio_queue is not None:
                    try:
                        self.audio_queue.put_nowait(np.asarray(ch0_i, dtype=np.int64))
                    except queue.Full:
                        # Audio consumer fell behind -- drop this chunk
                        # rather than block the receiver thread (packet
                        # loss here would corrupt the FFT/eye buffers
                        # too, audio glitching is the lesser harm).
                        pass

    def snapshot(self):
        """{name: np.ndarray}, oldest-to-newest, raw unsigned (0-65535);
        may be shorter than fft_size until enough packets have arrived.
        Apply cast_signed() at display time for the signed view."""
        with self.lock:
            return {name: np.fromiter(buf, dtype=np.float64) for name, buf in self.buffers.items()}


def cast_signed(arr):
    """Reinterprets a raw unsigned (0-65535) array as two's complement
    16-bit signed (-32768..32767) -- the real wire format. A pure display
    cast, applied fresh at every redraw so it's always consistent across
    every consumer of a SampleReceiver's snapshot(), never baked into the
    stored buffer itself."""
    return np.where(arr >= 32768, arr - 65536, arr)


def main():
    fft_size = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FFT_SIZE
    recv = SampleReceiver(fft_size=fft_size)
    recv.start()
    print(f"Listening for sample-stream UDP packets on {BIND_IP}:{SAMPLE_PORT} "
          f"(fft_size={fft_size}) ...")
    print("If nothing arrives: check SAMPLE_DEST_IP0-3 in sw/eth0.h matches "
          "this machine's IP, and that the board is streaming "
          "(eth_poll_sample_stream() in the main loop).")

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    fig.subplots_adjust(top=0.86)  # leave room for the sample-rate field above the plots
    window = np.hanning(fft_size)
    lines = {}
    for ax, name in zip(axes.flat, CHANNEL_NAMES):
        (line,) = ax.plot(np.zeros(fft_size // 2 + 1), np.zeros(fft_size // 2 + 1))
        ax.set_title(name)
        ax.set_xlabel("Freq (MHz)")
        ax.set_ylabel("Magnitude (dB)")
        ax.set_ylim(-20, 100)
        lines[name] = line

    # Editable sample rate, defaulting to the current hardware-measured
    # decimated rate (see SAMPLE_RATE_HZ's own comment) -- drives the FFT
    # frequency axis. A plain matplotlib TextBox rather than a Tkinter
    # widget, since this standalone tool is a bare pyplot window, not a
    # Tk app (unlike pc_console.py's embedded FFT tab).
    state = {"rate_hz": SAMPLE_RATE_HZ}

    def apply_rate(rate_hz):
        freqs_mhz = np.fft.rfftfreq(fft_size, d=1.0 / rate_hz) / 1e6
        for line in lines.values():
            line.set_xdata(freqs_mhz)
        for ax in axes.flat:
            ax.set_xlim(freqs_mhz[0], freqs_mhz[-1])
        fig.canvas.draw_idle()

    def on_rate_submit(text):
        try:
            rate_hz = float(text)
            if rate_hz <= 0:
                raise ValueError
        except ValueError:
            rate_box.set_val(f"{state['rate_hz']:.0f}")  # revert to last-good on bad input
            return
        state["rate_hz"] = rate_hz
        apply_rate(rate_hz)

    rate_ax = fig.add_axes((0.32, 0.92, 0.15, 0.05))
    rate_box = TextBox(rate_ax, "Sample rate (Hz)  ", initial=f"{SAMPLE_RATE_HZ:.0f}")
    rate_box.on_submit(on_rate_submit)
    apply_rate(SAMPLE_RATE_HZ)

    # Reinterprets the same raw 16-bit wire bits as signed (two's
    # complement, the real format) or unsigned -- a diagnostic toggle for
    # comparing which one actually matches what's on the wire, not a
    # "pick whichever looks right" setting. Applied fresh at every
    # redraw (see cast_signed()), not baked into the receiver's stored
    # buffer, so it takes effect immediately and uniformly.
    signed_ax = fig.add_axes((0.56, 0.88, 0.12, 0.1))
    signed_radio = RadioButtons(signed_ax, ("Signed", "Unsigned"), active=0)
    state["signed"] = True

    def on_signed_change(label):
        state["signed"] = (label == "Signed")

    signed_radio.on_clicked(on_signed_change)

    def update(_frame):
        data = recv.snapshot()
        for name, line in lines.items():
            buf = data[name]
            if len(buf) < fft_size:
                continue
            if state["signed"]:
                buf = cast_signed(buf)
            spectrum = np.fft.rfft(buf[-fft_size:] * window)
            line.set_ydata(20 * np.log10(np.abs(spectrum) + 1e-9))
        fig.suptitle(
            f"fm_receiver RX sample stream -- live FFT  "
            f"({recv.packets_received} pkts, {recv.bytes_received / 1024:.0f} KB, "
            f"{recv.packets_dropped} dropped)"
        )
        return list(lines.values())

    ani = animation.FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        recv.stop()


if __name__ == "__main__":
    main()
