#!/usr/bin/env python3
"""
Generate quantized FIR coefficients for the mono (L+R) audio low-pass
filter, extracting the 0-15kHz mono audio band from the MPX composite
signal (mpx_data_desc, 240kHz post mpx_decimator) while rejecting the
19kHz pilot and 23-53kHz stereo subcarrier.

Explicit power-of-2 quantization scale from the start (2026-09-15
lesson from gen_fir_taps.py's auto-scale mismatch, see project memory
fir_time_multiplexed_i_channel_bug.md) -- NOT auto-scaled to the
largest coefficient.
"""
import numpy as np
from scipy import signal

FS       = 240_000   # mpx_data_desc rate (post mpx_decimator)
PASSBAND = 15_000     # standard FM mono audio bandwidth
STOPBAND = 18_000     # leaves margin below the 19kHz pilot
NUMTAPS  = 96          # multiple of PARALLELISM=8; ~28dB stopband, ~0.68dB ripple (Parks-McClellan)
COEFF_WIDTH = 16
OUT_FILE = "mono_lpf_coeffs.hex"


def quantize_taps(taps, coeff_width, scale):
    quantized = np.round(taps * scale).astype(np.int64)
    limit = (1 << (coeff_width - 1)) - 1
    clipped = np.clip(quantized, -limit - 1, limit)
    if not np.array_equal(quantized, clipped):
        raise SystemExit(f"ERROR: scale={scale} clips at least one coefficient -- reduce QUANT_SCALE")
    return clipped


def write_readmemh(quantized, coeff_width, filename):
    hex_digits = (coeff_width + 3) // 4
    mask = (1 << coeff_width) - 1
    with open(filename, "w") as f:
        for val in quantized:
            twos = int(val) & mask
            f.write(f"{twos:0{hex_digits}x}\n")


if __name__ == "__main__":
    taps = signal.remez(NUMTAPS, [0, PASSBAND, STOPBAND, FS / 2], [1, 0], fs=FS)

    w, H = signal.freqz(taps, worN=8192, fs=FS)
    pb_mask = w <= PASSBAND
    stop_mask = w >= STOPBAND
    pb_ripple_db = 20 * np.log10(np.max(np.abs(H[pb_mask])) / (np.min(np.abs(H[pb_mask])) + 1e-12) + 1e-12)
    stop_atten_db = 20 * np.log10(np.max(np.abs(H[stop_mask])) + 1e-12)
    print(f"[float] passband ripple: {pb_ripple_db:.3f} dB, worst stopband gain: {stop_atten_db:.2f} dB")

    max_val = np.max(np.abs(taps))
    full_scale = (1 << (COEFF_WIDTH - 1)) - 1
    needed_scale = full_scale / max_val
    quant_shift = int(np.floor(np.log2(needed_scale)))
    QUANT_SCALE = 2 ** quant_shift
    print(f"max |float tap| = {max_val:.6f}, needed_scale={needed_scale:.2f}, "
          f"using explicit power-of-2 scale 2^{quant_shift}={QUANT_SCALE} (largest that fits)")

    quantized = quantize_taps(taps, COEFF_WIDTH, QUANT_SCALE)

    taps_deq = quantized.astype(np.float64) / QUANT_SCALE
    w2, H2 = signal.freqz(taps_deq, worN=8192, fs=FS)
    pb_ripple_db2 = 20 * np.log10(np.max(np.abs(H2[pb_mask])) / (np.min(np.abs(H2[pb_mask])) + 1e-12) + 1e-12)
    stop_atten_db2 = 20 * np.log10(np.max(np.abs(H2[stop_mask])) + 1e-12)
    print(f"[quantized 16-bit] passband ripple: {pb_ripple_db2:.3f} dB, worst stopband gain: {stop_atten_db2:.2f} dB")

    dc_gain = np.sum(quantized) / QUANT_SCALE
    print(f"DC gain check: sum(quantized_taps)/scale = {dc_gain:.6f} (target 1.0)")

    write_readmemh(quantized, COEFF_WIDTH, OUT_FILE)
    print(f"Wrote {len(quantized)} coefficients to {OUT_FILE} ({COEFF_WIDTH}-bit signed hex, one per line)")
