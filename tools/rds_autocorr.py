#!/usr/bin/env python3
"""
rds_autocorr.py -- standalone RDS presence check via autocorrelation.

Question this answers: is there a real, structured digital (RDS) signal
in the ch1_i channel, or just noise/leakage -- independent of the eye
diagram, which currently CAN'T show a converged eye regardless of signal
quality (RDS's ~40.42 samples/symbol ratio is non-integer, so the
existing fixed-integer-block eye-diagram code smears no matter what).

Detection signature (verified against synthetic data before trusting
it -- see below): RDS is Manchester/biphase-coded at 1187.5 baud (fixed
by spec, 57kHz/48), independent of any timing recovery. A first attempt
at this script looked for a POSITIVE autocorrelation peak at multiples
of the symbol period -- wrong, and a synthetic self-test caught it:
genuinely random Manchester-coded data has ~zero correlation at a full
symbol-period lag, because shifting by one symbol compares against an
independent random bit. What Manchester coding DOES guarantee is a
strong NEGATIVE dip at HALF the symbol period (~20.2 samples here) --
each symbol's second half is deliberately inverted relative to its
first half. Confirmed on synthetic data: -0.51 noiseless, still -0.48
at 4:1 SNR, still clearly distinguishable from the noise floor even at
a weak 0.3x signal buried in strong noise.

A second calibration pass (Monte Carlo over 30 noise-only trials) caught
a second bug: reading the dip as a min() over a small window around the
expected lag systematically biases the reading (order-statistic effect,
same direction for noise and signal), so noise alone could occasionally
read as "3 sigma". Fixed by reading the autocorrelation via linear
interpolation at the exact (non-integer) expected lag instead -- with
that fix, noise-only trials center on sigma~0 (max ~2.0 across 30
trials) and even a weak synthetic signal consistently reads 3+, a real
and reliable gap. Threshold below is calibrated against that, not
guessed.

2026-09-19: rds.sv's post-CIC filter was pulled out entirely (to isolate
"is the RTL filter design wrong" from "is there even a signal there") --
this script now does the candidate filtering itself, in Python, on the
raw CIC output. Reports the sigma reading BOTH unfiltered and filtered,
side by side, so filtering's actual effect is directly visible instead
of guessed at.

Usage: python rds_autocorr.py [--seconds S] [--channel CH] [--cutoff HZ]
                               [--order N] [--no-filter]
    --seconds S   : capture duration (default 3.0)
    --channel CH  : which sample-stream channel (default ch1_i, RDS)
    --cutoff HZ   : lowpass cutoff, Hz (default 2500 -- RDS's own data
                    bandwidth runs to ~2.4kHz, so this is a reasonable
                    starting point: keep RDS, reject the ~19kHz leakage)
    --order N     : Butterworth filter order (default 4)
    --no-filter   : skip the filtered comparison, unfiltered analysis only

Reuses SampleReceiver from sample_stream_view.py (same UDP capture path
pc_console.py's FFT/eye tabs already use) -- board must already be
pointed at this machine's IP, same prereq as that tool.
"""

import argparse
import time

import numpy as np
import scipy.signal as scipy_signal
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from sample_stream_view import SampleReceiver, CHANNEL_NAMES, cast_signed

RDS_SYMBOL_RATE_HZ = 1187.5           # fixed by the RDS spec (57kHz/48)
OUTPUT_RATE_HZ = 48_000.0             # nominal; real rate is close but not exact
SAMPLES_PER_SYMBOL = OUTPUT_RATE_HZ / RDS_SYMBOL_RATE_HZ  # ~40.42
HALF_SYMBOL_LAG = SAMPLES_PER_SYMBOL / 2.0                # ~20.21

MAX_LAG_SYMBOLS = 6   # how many symbol periods out to plot


def autocorrelation_fft(x):
    """Normalized autocorrelation via FFT (O(N log N)), zero-padded to
    avoid circular wraparound. Returns ac[0..len(x)-1], ac[0] == 1.0."""
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean()
    n = len(x)
    nfft = 1
    while nfft < 2 * n:
        nfft *= 2
    spec = np.fft.rfft(x, n=nfft)
    ac = np.fft.irfft(spec * np.conj(spec), n=nfft)[:n]
    if ac[0] != 0:
        ac = ac / ac[0]
    return ac


def lowpass_filter(x, cutoff_hz, order, fs_hz=OUTPUT_RATE_HZ):
    """Zero-phase Butterworth lowpass (filtfilt) -- zero-phase matters
    here specifically because we're reading dip POSITION precisely
    (half-symbol lag), and a causal filter would smear/shift that."""
    b, a = scipy_signal.butter(order, cutoff_hz, btype="low", fs=fs_hz)
    return scipy_signal.filtfilt(b, a, x)


def evaluate(buf, label, max_lag):
    """Runs the half-symbol-dip detection on `buf`, prints a small report,
    and returns (lags, ac_lags, half_lags_exact, baseline_mean,
    baseline_std, sigmas) for plotting."""
    ac = autocorrelation_fft(buf)
    lags = np.arange(1, min(max_lag, len(ac)))
    ac_lags = ac[lags]

    half_lags_exact = [HALF_SYMBOL_LAG * k for k in (1, 3, 5) if HALF_SYMBOL_LAG * k < max_lag]
    exclude_radius = 1.5
    mask = np.ones_like(lags, dtype=bool)
    for el in half_lags_exact:
        mask &= np.abs(lags - el) > exclude_radius
    baseline_mean = ac_lags[mask].mean()
    baseline_std = ac_lags[mask].std()

    ac_index = np.arange(len(ac))
    sigmas = []
    print(f"\n--- {label} ---")
    print(f"Noise-floor autocorrelation: mean={baseline_mean:.4f} std={baseline_std:.4f}")
    print(f"{'lag (samples)':>14} {'half-symbol #':>14} {'ac value':>10} {'sigma below baseline':>22}")
    for k, el in zip((1, 3, 5), half_lags_exact):
        val = float(np.interp(el, ac_index, ac))
        sigma = (baseline_mean - val) / baseline_std if baseline_std > 0 else float("nan")
        sigmas.append(sigma)
        print(f"{el:>14.2f} {k:>14d} {val:>10.4f} {sigma:>22.2f}")

    verdict_sigma = sigmas[0] if sigmas else None
    # Calibrated against 30 noise-only trials (see docstring): noise alone
    # centers on sigma~0 and tops out around ~2.0, so 3 is a real margin.
    if verdict_sigma is not None and verdict_sigma > 3:
        print(f"VERDICT ({label}): {verdict_sigma:.1f} sigma -- looks like a real "
              f"structured (Manchester-like) signal, not just noise.")
    elif verdict_sigma is not None:
        print(f"VERDICT ({label}): only {verdict_sigma:.1f} sigma -- not convincing, could be noise.")

    return lags, ac_lags, half_lags_exact, baseline_mean, baseline_std, sigmas


def main():
    p = argparse.ArgumentParser(description="RDS presence check via autocorrelation")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--channel", default="ch1_i", choices=CHANNEL_NAMES)
    p.add_argument("--cutoff", type=float, default=2500.0,
                    help="lowpass cutoff in Hz (default 2500 -- RDS data bandwidth ~2.4kHz)")
    p.add_argument("--order", type=int, default=4)
    p.add_argument("--no-filter", action="store_true", help="skip the filtered comparison")
    args = p.parse_args()

    fft_size = int(OUTPUT_RATE_HZ * args.seconds * 1.5)
    print(f"Listening for {args.seconds:.1f}s on channel {args.channel} "
          f"(buffer sized for {fft_size} samples)...")

    recv = SampleReceiver(fft_size=fft_size)
    recv.start()
    time.sleep(args.seconds)

    data = recv.snapshot()
    buf = cast_signed(data[args.channel]).astype(np.float64)
    recv.stop()

    n_captured = len(buf)
    print(f"Captured {n_captured} samples "
          f"({recv.packets_received} packets, {recv.packets_dropped} dropped).")
    if n_captured < int(SAMPLES_PER_SYMBOL * MAX_LAG_SYMBOLS * 4):
        print("WARNING: not many samples captured relative to the lag range "
              "of interest -- consider a longer --seconds.")

    print(f"\nExpected symbol period: {SAMPLES_PER_SYMBOL:.2f} samples "
          f"(RDS {RDS_SYMBOL_RATE_HZ}Hz @ ~{OUTPUT_RATE_HZ:.0f}Hz)")
    print("Looking for a NEGATIVE dip at half-symbol-period lags (Manchester signature)")

    max_lag = int(SAMPLES_PER_SYMBOL * MAX_LAG_SYMBOLS)
    results = {}
    results["raw (no filter)"] = evaluate(buf, "raw (no filter)", max_lag)

    if not args.no_filter:
        filtered = lowpass_filter(buf, args.cutoff, args.order)
        results[f"filtered ({args.cutoff:.0f}Hz, order {args.order})"] = evaluate(
            filtered, f"filtered ({args.cutoff:.0f}Hz, order {args.order})", max_lag
        )

    fig, axes = plt.subplots(len(results), 1, figsize=(11, 5 * len(results)), squeeze=False)
    for ax, (label, (lags, ac_lags, half_lags_exact, baseline_mean, baseline_std, sigmas)) in \
            zip(axes[:, 0], results.items()):
        ax.plot(lags, ac_lags, lw=0.8, color="tab:blue")
        ax.axhline(baseline_mean, color="gray", ls="--", lw=1, label="noise-floor mean")
        ax.axhline(baseline_mean - 3 * baseline_std, color="tab:red", ls=":", lw=1, label="-3 sigma")
        for k, el in zip((1, 3, 5), half_lags_exact):
            ax.axvline(el, color="tab:orange", ls="--", lw=0.7, alpha=0.6,
                        label=f"half-symbol x{k}" if k == 1 else None)
        ax.set_xlabel("lag (samples)")
        ax.set_ylabel("normalized autocorrelation")
        sigma0 = sigmas[0] if sigmas else float("nan")
        ax.set_title(f"{label} -- {n_captured} samples, primary dip sigma={sigma0:.2f}")
        ax.legend()
        ax.grid(alpha=0.3)
    fig.tight_layout()
    out_path = "rds_autocorr.png"
    fig.savefig(out_path, dpi=130)
    print(f"\nPlot saved to {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
