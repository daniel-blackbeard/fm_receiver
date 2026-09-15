#!/usr/bin/env python3
"""
Golden-vector generator for fir_time_multiplexed.sv (2026-09-15).

Loads the ACTUAL fir_coeffs.hex (the exact bits synthesized into
hardware, post-quantization -- not the float design), drives a
representative chirp I/Q waveform through it using the RTL's EXACT
bit-accurate algorithm (per-tap floor-shift by PROD_SHIFT, sum, final
floor-shift by OUT_SHIFT, 16-bit two's-complement wrap), and writes both
the input and the expected output as $readmemh-compatible hex files for
tb/tb_fir_golden.sv to drive the real RTL against and compare bit-for-bit.

This replaces reasoning about the filter's gain/scale in the abstract --
the taps used here are read from the same file Vivado synthesizes, so if
this script's output ever disagrees with the RTL's, the disagreement is
real, not a modeling assumption.
"""
import numpy as np

NUM_TAPS = 48
PROD_SHIFT = 8
OUT_SHIFT = 8
FS_DEC = 30_000_000 / 25          # 1.2 MHz, matches gen_fir_taps.py's fs_dec
N_SAMPLES = 600
AMPLITUDE = 8000                   # representative, well below overflow risk

def load_taps(path):
    taps = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            v = int(line, 16)
            if v & 0x8000:
                v -= 0x10000
            taps.append(v)
    assert len(taps) == NUM_TAPS, len(taps)
    return taps

def to_s16_wrap(v):
    v &= 0xFFFF
    if v & 0x8000:
        v -= 0x10000
    return v

def rtl_exact_filter(taps, x):
    """x: list of int16 samples (oldest first). Returns list of RTL-exact
    outputs, one per input sample, using a sliding window exactly matching
    the RTL's shift register (newest sample pairs with taps[0])."""
    out = []
    hist = [0] * NUM_TAPS  # hist[0] = newest
    for sample in x:
        hist = [sample] + hist[:-1]
        acc = 0
        for k in range(NUM_TAPS):
            prod = hist[k] * taps[k]
            acc += prod >> PROD_SHIFT  # Python >> is floor/arithmetic for ints, matches >>>
        y = acc >> OUT_SHIFT
        out.append(to_s16_wrap(y))
    return out

def write_hex(path, vals):
    with open(path, "w") as f:
        for v in vals:
            f.write(f"{v & 0xFFFF:04x}\n")

if __name__ == "__main__":
    taps = load_taps("src/fir_coeffs.hex")

    # Chirp sweeping from near-DC through the passband/stopband edges,
    # covering both channels with a 90deg phase offset (I=cos, Q=sin).
    freq_start = 0.001
    freq_end = 0.40
    n = np.arange(N_SAMPLES)
    freq = freq_start + (freq_end - freq_start) * (n / N_SAMPLES)
    phase = np.cumsum(2 * np.pi * freq)

    i_in = np.round(np.cos(phase) * AMPLITUDE).astype(int)
    q_in = np.round(np.sin(phase) * AMPLITUDE).astype(int)
    i_in = [to_s16_wrap(v) for v in i_in]
    q_in = [to_s16_wrap(v) for v in q_in]

    i_out = rtl_exact_filter(taps, i_in)
    q_out = rtl_exact_filter(taps, q_in)

    wrap_events = sum(1 for k in range(1, N_SAMPLES)
                       if abs(i_out[k] - i_out[k-1]) > 40000 or abs(q_out[k] - q_out[k-1]) > 40000)
    print(f"Generated {N_SAMPLES} samples. Max |i_out|={max(abs(v) for v in i_out)} "
          f"max |q_out|={max(abs(v) for v in q_out)} suspicious_jumps={wrap_events}")

    write_hex("tb/fir_golden_in_i.hex", i_in)
    write_hex("tb/fir_golden_in_q.hex", q_in)
    write_hex("tb/fir_golden_exp_i.hex", i_out)
    write_hex("tb/fir_golden_exp_q.hex", q_out)
    print("Wrote tb/fir_golden_in_i.hex, fir_golden_in_q.hex, fir_golden_exp_i.hex, fir_golden_exp_q.hex")
