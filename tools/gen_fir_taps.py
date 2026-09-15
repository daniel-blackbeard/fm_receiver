#!/usr/bin/env python3
"""
Generate quantized FIR coefficients for the post-CIC channel-select filter,
with CIC droop compensation baked into the passband shape, and write them
to a $readmemh-compatible hex file for RTL synthesis.

Design flow:
  1. Build the target frequency response: flat-inverse-of-CIC-droop across
     the passband, zero in the stopband (see cic_mag()).
  2. Design a linear-phase FIR (firwin2) matching that target.
  3. Quantize coefficients to signed fixed-point (two's complement).
  4. Verify the quantized filter still meets spec (freqz on the quantized
     taps, not just the float design) -- quantization can measurably erode
     stopband depth if COEFF_WIDTH is too small, so this is not optional.
  5. Write one coefficient per line, in hex, matching $readmemh's expected
     format for a `logic signed [COEFF_WIDTH-1:0] coeffs [0:NUMTAPS-1]` array.
"""

import numpy as np
from scipy import signal


def cic_mag(f, fs_in, R, N):
    """CIC magnitude response (normalized to unity DC gain)."""
    f = np.where(f == 0, 1e-9, f)
    h1 = np.abs(np.sin(np.pi * f * R / fs_in) / (R * np.sin(np.pi * f / fs_in)))
    return h1 ** N


def design_channel_fir(fs_in, R, N, numtaps, passband_edge, stopband_edge,
                        n_pb_points=20):
    """
    Design the droop-compensated channel-select FIR.

    fs_in          : rate INTO the CIC (e.g. 30 MHz)
    R, N           : CIC decimation ratio and order (must match the RTL CIC)
    numtaps        : FIR length. Odd length gives an exact-integer linear-phase
                     group delay ((numtaps-1)/2); even length gives a half-sample
                     delay instead -- both are valid linear-phase FIRs, this just
                     affects delay bookkeeping in any timing-sensitive test bench,
                     not the filter's correctness. Match this to whatever your
                     RTL FIR engine actually requires (e.g. a multiple of its
                     PARALLELISM for a time-multiplexed engine).
    passband_edge  : Hz, edge of the desired channel (e.g. 130 kHz)
    stopband_edge  : Hz, where full rejection should begin (e.g. 272 kHz)
    """
    fs_dec = fs_in / R

    f_pb = np.linspace(0, passband_edge, n_pb_points)
    comp = 1.0 / cic_mag(f_pb, fs_in, R, N)
    comp = comp / comp[0]  # normalize so DC gain is exactly 1

    freqs = np.concatenate([f_pb, [stopband_edge, fs_dec / 2]])
    desired = np.concatenate([comp, [0, 0]])

    taps = signal.firwin2(numtaps, freqs, desired, fs=fs_dec)
    return taps, fs_dec, freqs, desired


def quantize_taps(taps, coeff_width, scale=None):
    """
    Quantize float taps to signed two's-complement integers.

    coeff_width : total bit width per coefficient (sign + magnitude bits)
    scale       : if None, auto-scales so the largest |tap| uses the full
                  range (2^(coeff_width-1) - 1). Pass an explicit int scale
                  to match a fixed Q-format instead (e.g. for a known Q1.14).
    """
    if scale is None:
        max_val = np.max(np.abs(taps))
        full_scale = (1 << (coeff_width - 1)) - 1
        scale = full_scale / max_val

    quantized = np.round(taps * scale).astype(np.int64)

    limit = (1 << (coeff_width - 1)) - 1
    quantized = np.clip(quantized, -limit - 1, limit)

    return quantized, scale


def write_readmemh(quantized, coeff_width, filename):
    """Write one coefficient per line, in hex, two's-complement, for $readmemh."""
    hex_digits = (coeff_width + 3) // 4
    mask = (1 << coeff_width) - 1

    with open(filename, "w") as f:
        for val in quantized:
            twos = val & mask  # two's complement bit pattern for negative values
            f.write(f"{twos:0{hex_digits}x}\n")


def verify_quantized_response(quantized, scale, fs_dec, passband_edge,
                               stopband_edge, label=""):
    """Compare quantized-filter response against the design targets."""
    taps_deq = quantized.astype(np.float64) / scale
    w, H = signal.freqz(taps_deq, worN=8192, fs=fs_dec)

    pb_mask = w <= passband_edge
    stop_mask = w >= stopband_edge

    pb_ripple_db = 20 * np.log10(
        np.max(np.abs(H[pb_mask])) / (np.min(np.abs(H[pb_mask])) + 1e-12) + 1e-12
    )
    stop_atten_db = 20 * np.log10(np.max(np.abs(H[stop_mask])) + 1e-12)

    print(f"[{label}] passband ripple: {pb_ripple_db:.3f} dB, "
          f"worst stopband gain: {stop_atten_db:.2f} dB")
    return w, H


if __name__ == "__main__":
    # --- parameters: match these to your actual RTL CIC/FIR ---
    FS_IN         = 30_000_000   # rate into the CIC
    R             = 25
    N             = 3
    NUMTAPS       = 48
    PASSBAND_EDGE = 130_000
    STOPBAND_EDGE = 272_000
    COEFF_WIDTH   = 16           # bits per coefficient, incl. sign -- tune to your DSP48/mult width
    OUT_FILE      = "fir_coeffs.hex"

    taps, fs_dec, freqs, desired = design_channel_fir(
        FS_IN, R, N, NUMTAPS, PASSBAND_EDGE, STOPBAND_EDGE
    )

    # sanity check float design first
    w, H = signal.freqz(taps, worN=8192, fs=fs_dec)
    pb_mask = w <= PASSBAND_EDGE
    stop_mask = w >= STOPBAND_EDGE
    print(f"[float]     passband ripple: "
          f"{20*np.log10(np.max(np.abs(H[pb_mask]))/(np.min(np.abs(H[pb_mask]))+1e-12)+1e-12):.3f} dB, "
          f"worst stopband gain: {20*np.log10(np.max(np.abs(H[stop_mask]))+1e-12):.2f} dB")

    # Explicit power-of-2 scale, NOT auto-scaled to the largest tap
    # (2026-09-15 fix): quantize_taps()'s default auto-scale maximizes
    # coefficient resolution but lands on an arbitrary non-power-of-2
    # scale (was 99911.39 here) -- the RTL can only apply integer
    # bit-shifts (PROD_SHIFT+OUT_SHIFT), so no achievable shift exactly
    # recovers this filter's designed unity DC/passband gain
    # (`comp = comp/comp[0]` above). The mismatch was real and measured:
    # OUT_SHIFT=7 gave ~3.05x actual gain (99911/2^15) -- a 16-bit output
    # overflow bug; the OUT_SHIFT=9 "fix" for that gave ~0.76x
    # (99911/2^17) -- safely inside 16 bits, but silently ~24% quieter
    # than intended, exactly matching the user's real-hardware
    # observation of low FIR amplitude.
    #
    # 2^16=65536 is the largest power-of-2 scale that still fits every
    # tap in COEFF_WIDTH=16 bits here (verified: max |tap|=0.328,
    # 0.328*65536=21493, comfortably under +-32767 -- 2^17 does NOT fit,
    # 0.328*131072=42986 clips). Paired with the RTL's PROD_SHIFT=8, an
    # OUT_SHIFT=8 (updated in src/fir_time_multiplexed.sv) recovers the
    # designed gain EXACTLY (up to quantization rounding), not
    # approximately -- 8+8=16=log2(65536).
    QUANT_SCALE = 2 ** 16
    quantized, scale = quantize_taps(taps, COEFF_WIDTH, scale=QUANT_SCALE)
    print(f"quantization scale factor: {scale:.2f}  "
          f"(explicit 2^{int(np.log2(scale))}, not auto-scaled -- see comment above)")

    verify_quantized_response(quantized, scale, fs_dec, PASSBAND_EDGE,
                               STOPBAND_EDGE, label=f"quantized {COEFF_WIDTH}-bit")

    write_readmemh(quantized, COEFF_WIDTH, OUT_FILE)
    print(f"Wrote {len(quantized)} coefficients to {OUT_FILE} "
          f"({COEFF_WIDTH}-bit signed hex, one per line)")