#!/usr/bin/env python3
"""
gen_rds_golden.py -- synthetic RDS stream + per-stage golden vectors for
tb/tb_rds_golden.sv (single testbench, every stage of src/rds.sv checked in
one pass against these vectors).

What it produces (all under tb/):
  rds_gold_in.hex          240 kHz stimulus, i.e. exactly what dsp.sv feeds rds.sv
                           (composite MPX -> dsp.sv's "mpx * cos57k >>> 15" mixer)
  rds_gold_s0.hex  ...     one golden file per stage (see STAGES below)
  rds_gold_cfg.svh         counts, tolerances, NCO constants, expected PI/PS/RT

Pipeline modelled (each function below is one stage of the RTL to build):
  S0  CIC  R=5, N=5, 28-bit registers, OUT_SHIFT=12, truncated to [15:0]   (bit-exact)
  S1  low-pass biquad (Butterworth, causal)                                (tolerance)
  S2  symbol NCO phase word, 32-bit, step = round(2^32*1187.5/48000)       (bit-exact)
  S3  biphase integrate-and-dump: soft value + hard bit per symbol         (soft: tol, bit: exact)
  S4  differential decode                                                  (bit-exact)
  S5  (26,10) syndrome of the last 26 bits, per bit                        (bit-exact)
  S6  offset-word match type (none/A/B/C/C'/D), per bit                    (bit-exact)
  S7  block sync: lock after LOCK_RUN consecutive correct blocks, then one
      (data16, type) event per block                                       (bit-exact)
  S8  final PI / PS / RadioText registers                                  (bit-exact)

Every model here is checked against something independent before the vectors
are written (see the "verify" section in main()): the CIC against an FIR of
its own impulse response, the filter against scipy.lfilter and a slow
direct-form loop, the whole chain by decoding the original payload back out
through the demodulated bits and through tools/rds_decode.py's decoder.

Run:  C:\\msys64\\ucrt64\\bin\\python.exe tools\\gen_rds_golden.py
"""

import argparse
import contextlib
import io
import re
import sys
from pathlib import Path

import numpy as np
import scipy.signal as sps

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rds_decode as rd  # noqa: E402  (protocol constants + proven decoder pieces)

FS_IN = 240_000
DECIM = 5
FS_S0 = FS_IN // DECIM
BIT_RATE = 1187.5
T_IN = FS_IN / BIT_RATE          # 202.105... input samples per bit
CIC_W = 28
OUT_SHIFT = 12
STEP = int(round((1 << 32) * BIT_RATE / FS_S0))

TYPE_CODE = {"": 0, "A": 1, "B": 2, "C": 3, "C'": 4, "D": 5}
TYPE_NAME = {v: k for k, v in TYPE_CODE.items()}
NEXT = {"A": ("B",), "B": ("C", "C'"), "C": ("D",), "C'": ("D",), "D": ("A",)}

# per-stage "may lag by this many trailing items" (no more input arrives to flush them)
SLACK = {0: 0, 1: 2, 2: 2, 3: 2, 4: 2, 5: 2, 6: 2, 7: 1}


# --------------------------------------------------------------------------
# Payload: blocks, groups, bitstream (encoder is independent of rds_decode's)
# --------------------------------------------------------------------------

def crc_lfsr(bits):
    """Bit-serial LFSR division by g(x), MSB first, direct form (the feedback
    XOR already multiplies the message by x^10, so no flush bits are needed).
    Deliberately a different algorithm from rds_decode.py's shift-and-xor."""
    low = rd.GEN_POLY & 0x3FF
    reg = 0
    for b in bits:
        fb = ((reg >> 9) & 1) ^ b
        reg = (reg << 1) & 0x3FF
        if fb:
            reg ^= low
    return reg


def block_bits(data16, offset_name):
    dbits = [(data16 >> (15 - i)) & 1 for i in range(16)]
    check = crc_lfsr(dbits) ^ rd.OFFSET_WORDS[offset_name]
    return dbits + [(check >> (9 - i)) & 1 for i in range(10)]


def make_groups(pi, ps, rt, n_groups):
    """Even groups: 0A (PS, one 2-char segment each). Odd groups: 2A (RadioText,
    4 chars each). Returns [(wordA, wordB, wordC, wordD), ...] and their kinds."""
    assert len(ps) == 8 and len(rt) % 4 == 0
    tp, pty = 1, 10
    groups = []
    for gi in range(n_groups):
        if gi % 2 == 0:
            seg = (gi // 2) % 4
            b = (0 << 12) | (0 << 11) | (tp << 10) | (pty << 5) | (1 << 4) | (1 << 3) | seg
            c = 0xE1CD                                 # arbitrary AF word
            d = (ord(ps[2 * seg]) << 8) | ord(ps[2 * seg + 1])
        else:
            nseg = len(rt) // 4
            seg = (gi // 2) % nseg
            b = (2 << 12) | (0 << 11) | (tp << 10) | (pty << 5) | (0 << 4) | seg
            c = (ord(rt[4 * seg]) << 8) | ord(rt[4 * seg + 1])
            d = (ord(rt[4 * seg + 2]) << 8) | ord(rt[4 * seg + 3])
        groups.append((pi, b, c, d))
    return groups


# --------------------------------------------------------------------------
# Signal synthesis (240 kHz, as seen by rds.sv after dsp.sv's mixer)
# --------------------------------------------------------------------------

def g_pulse(x):
    """Half-cosine-spectrum pulse (EN 50067 biphase shaping), x in bit periods.
    g(x) = cos(4*pi*x) / (1 - 64 x^2); limit pi/4 at |x| = 1/8."""
    x = np.asarray(x, dtype=float)
    den = 1.0 - 64.0 * x * x
    num = np.cos(4.0 * np.pi * x)
    out = np.empty_like(x)
    sing = np.abs(den) < 1e-7
    out[~sing] = num[~sing] / den[~sing]
    out[sing] = np.pi / 4.0
    return out


def rds_baseband(enc, n_in, width=10):
    """Shaped biphase waveform. Bit j occupies x in [j, j+1) (x in bit periods
    since stream start). enc[j]=1 -> (+, -) halves, 0 -> (-, +): the Manchester
    impulse pair sits at the centre of each half so the optimum integrate-and-
    dump window is exactly the bit period."""
    n = np.arange(n_in)
    x = n * 19.0 / 3840.0                       # == n * 1187.5 / 240000, exact
    j0 = np.floor(x).astype(int)
    a = 2.0 * np.asarray(enc, dtype=float) - 1.0
    m = np.zeros(n_in)
    for d in range(-width, width + 1):
        j = j0 + d
        ok = (j >= 0) & (j < len(a))
        aj = np.where(ok, a[np.where(ok, j, 0)], 0.0)
        u = x - j
        m += aj * (g_pulse(u - 0.25) - g_pulse(u - 0.75))
    return m


def synthesize(enc, n_in, rds_peak, rds_phase, noise_lsb, rng):
    """Composite MPX (mono + pilot + DSB stereo + RDS) then dsp.sv's mixer:
    rds_mix_i = (mpx_data * vco3_cos) >>> 15, both 16-bit signed."""
    n = np.arange(n_in, dtype=np.int64)

    def ph(num, den):
        return 2.0 * np.pi * ((n * num) % den) / den   # exact phase, no drift

    left = np.sin(ph(11, 6000))                  # 440 Hz
    right = np.sin(ph(13, 2400))                 # 1300 Hz
    mono, diff = 0.5 * (left + right), 0.5 * (left - right)
    pilot = np.sin(ph(19, 240))
    st38 = np.sin(ph(38, 240))
    car57 = np.cos(ph(57, 240) + rds_phase)

    m = rds_baseband(enc, n_in)
    m *= rds_peak / np.max(np.abs(m))
    mpx = 0.40 * mono + 0.40 * diff * st38 + 0.09 * pilot + m * car57
    mpx_q = np.clip(np.round(mpx * 32767.0), -32768, 32767).astype(np.int64)
    if noise_lsb > 0:
        mpx_q = np.clip(mpx_q + np.round(rng.normal(0, noise_lsb, n_in)).astype(np.int64),
                        -32768, 32767)
    cos_q = np.round(np.cos(ph(57, 240)) * 32767.0).astype(np.int64)
    mix = (mpx_q * cos_q) >> 15                  # arithmetic shift, like SV's >>>
    assert np.max(np.abs(mix)) < 32768
    return mix


# --------------------------------------------------------------------------
# S0: CIC, register-for-register copy of rds.sv (NBA semantics: every stage
# reads the OTHER stages' pre-edge value)
# --------------------------------------------------------------------------

def _w(v):
    v &= (1 << CIC_W) - 1
    return v - (1 << CIC_W) if v & (1 << (CIC_W - 1)) else v


def _out16(com5):
    v = _w(com5 + (1 << (OUT_SHIFT - 1))) >> OUT_SHIFT
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def cic_model(x_in):
    """Returns (out16 array, raw com5 array), one entry per decimation event."""
    i1 = i2 = i3 = i4 = i5 = 0
    c1 = c2 = c3 = c4 = c5 = 0
    d5 = d1 = d2 = d3 = d4 = 0
    counter = 0
    raw = []
    for x in x_in:
        x = int(x)
        if counter == 0:
            n_c1, n_d5 = _w(i5 - d5), i5
            n_c2, n_d1 = _w(c1 - d1), c1
            n_c3, n_d2 = _w(c2 - d2), c2
            n_c4, n_d3 = _w(c3 - d3), c3
            n_c5, n_d4 = _w(c4 - d4), c4
            c1, c2, c3, c4, c5 = n_c1, n_c2, n_c3, n_c4, n_c5
            d5, d1, d2, d3, d4 = n_d5, n_d1, n_d2, n_d3, n_d4
            raw.append(c5)
        i1, i2, i3, i4, i5 = (_w(i1 + x), _w(i2 + i1), _w(i3 + i2),
                              _w(i4 + i3), _w(i5 + i4))
        counter = 0 if counter == 4 else counter + 1
    return np.array([_out16(r) for r in raw], dtype=np.int64), np.array(raw, dtype=np.int64)


def verify_cic_model(rng):
    """Model vs. an FIR of its own impulse response: find the pipeline delay D
    such that com5[k] == sum_i h[i] x[5k - D - i] (h = boxcar5 convolved 5x),
    then confirm that equality on random input. Catches integrator/comb
    ordering bugs and wrap handling; the RTL-vs-model check happens in sim."""
    h = np.array([1])
    for _ in range(5):
        h = np.convolve(h, np.ones(5, dtype=np.int64))
    x = rng.integers(-2000, 2000, 1000).astype(np.int64)
    _, raw = cic_model(x)
    full = np.convolve(x, h)                      # full[m] = sum_i h[i] x[m-i]
    for dly in range(0, 60):
        idx = 5 * np.arange(len(raw)) - dly
        ok = idx >= 0
        if np.array_equal(raw[ok], full[idx[ok]]):
            return dly
    raise AssertionError("CIC model is not equivalent to boxcar^5 FIR + delay")


# --------------------------------------------------------------------------
# S1: causal low-pass (float reference, quantised to int16)
# --------------------------------------------------------------------------

def lpf_reference(x, order, cutoff):
    sos = sps.butter(order, cutoff, btype="low", fs=FS_S0, output="sos")
    y = sps.sosfilt(sos, x.astype(float))
    # independent checks of the reference itself
    b, a = sps.butter(order, cutoff, btype="low", fs=FS_S0)
    assert np.allclose(sps.lfilter(b, a, x.astype(float)), y, atol=1e-6)
    slow = np.zeros(len(x))
    for n in range(len(x)):                       # direct form, obviously correct
        acc = sum(b[k] * x[n - k] for k in range(len(b)) if n - k >= 0)
        acc -= sum(a[k] * slow[n - k] for k in range(1, len(a)) if n - k >= 0)
        slow[n] = acc / a[0]
        if n > 600:                               # spot-check the first 600 samples
            break
    assert np.allclose(slow[:601], y[:601], atol=1e-6)
    return np.clip(np.round(y), -32768, 32767).astype(np.int64), sos


def impulse_centroid(sos, n=4000):
    imp = np.zeros(n)
    imp[0] = 1.0
    h = sps.sosfilt(sos, imp)
    return float(np.sum(np.arange(n) * h) / np.sum(h))


# --------------------------------------------------------------------------
# S2/S3: NCO and biphase integrate-and-dump
# --------------------------------------------------------------------------

def nco_words(n0, phase0):
    """U[k] = phase0 + k*STEP (unwrapped, int64). Word for sample k = U mod 2^32;
    symbol index = U >> 32; half-symbol flag = bit 31."""
    return phase0 + np.arange(n0, dtype=np.int64) * STEP


def soft_symbols(s1, phase0):
    u = nco_words(len(s1), phase0)
    sym = u >> 32
    half = (u >> 31) & 1
    sgn = np.where(half == 0, 1.0, -1.0)
    s_last = int(sym[-1])                         # symbols 0..s_last-1 are complete
    soft = np.bincount(sym, weights=sgn * s1, minlength=s_last + 1)
    return np.round(soft[:s_last]).astype(np.int64), s_last


def calibrate_phase0(s1, lo=3, hi_trim=3, n_grid=1024):
    best = []
    for g in range(n_grid):
        p0 = (g << 32) // n_grid
        soft, s_last = soft_symbols(s1, p0)
        best.append(np.mean(np.abs(soft[lo:s_last - hi_trim])))
    best = np.array(best)
    top = best.max()
    sel = np.flatnonzero(best >= 0.995 * top)
    ang = 2 * np.pi * sel / n_grid                # circular mean of the plateau
    mean_ang = np.angle(np.mean(np.exp(1j * ang))) % (2 * np.pi)
    return int(round(mean_ang / (2 * np.pi) * (1 << 32))) & 0xFFFFFFFF, top, best


# --------------------------------------------------------------------------
# S7: block sync (bit-serial, mirrors what the RTL state machine must do)
# --------------------------------------------------------------------------

def golden_blocks(types, data, first_i, end_i, lock_run):
    """types[i]: offset-word name matched by the 26-bit window ENDING at bit i
    ('' if none). Lock = lock_run consecutive hits 26 bits apart whose types follow
    A->B->(C|C')->D->A. The block that completes the run is the first one emitted;
    while locked, a block whose type is not a legal successor drops lock and hunts
    again from scratch. Returns [(i, data16, type_code), ...]."""
    chain = {}                                    # i -> (len, type)
    blocks = []
    locked, last_t, next_i = False, None, None
    for i in range(first_i, end_i):
        t = types[i]
        if locked:
            if i != next_i:
                continue
            if t and t in NEXT[last_t]:
                blocks.append((i, _bits_int(data[i - 25:i - 9]), TYPE_CODE[t]))
                last_t, next_i = t, i + 26
            else:
                locked, chain = False, {}
            continue
        if not t:
            continue
        prev = chain.get(i - 26)
        ln = prev[0] + 1 if prev and t in NEXT[prev[1]] else 1
        chain[i] = (ln, t)
        if ln >= lock_run:
            locked, last_t, next_i = True, t, i + 26
            blocks.append((i, _bits_int(data[i - 25:i - 9]), TYPE_CODE[t]))
    return blocks, locked


def _bits_int(bits):
    v = 0
    for b in bits:
        v = (v << 1) | int(b)
    return v


def golden_content(blocks):
    """Group assembly from the S7 event stream. A group is applied when its D
    block arrives after A,B,(C|C') in order. Unseen characters stay 0x00."""
    pi = 0
    ps = [0] * 8
    rt = [0] * 64
    seq, ab_prev = [], None
    for _i, d, tc in blocks:
        t = TYPE_NAME[tc]
        if t == "A":
            pi, seq = d, [("A", d)]
        elif t == "B" and seq and seq[-1][0] == "A":
            seq.append(("B", d))
        elif t in ("C", "C'") and seq and seq[-1][0] == "B":
            seq.append(("C", d))
        elif t == "D" and seq and seq[-1][0] == "C":
            (_, _pi), (_, b), (_, c) = seq
            gtype, ver = b >> 12, (b >> 11) & 1
            if gtype == 0:
                seg = b & 3
                ps[2 * seg], ps[2 * seg + 1] = d >> 8, d & 0xFF
            elif gtype == 2:
                seg, ab = b & 0xF, (b >> 4) & 1
                if ab_prev is not None and ab != ab_prev:
                    rt = [0] * 64
                ab_prev = ab
                if ver == 0:
                    rt[4 * seg:4 * seg + 4] = [c >> 8, c & 0xFF, d >> 8, d & 0xFF]
                else:
                    rt[2 * seg:2 * seg + 2] = [d >> 8, d & 0xFF]
            seq = []
        else:
            seq = []
    return pi, ps, rt


# --------------------------------------------------------------------------
# Output helpers
# --------------------------------------------------------------------------

def write_hex(path, values, bits):
    digits = (bits + 3) // 4
    mask = (1 << bits) - 1
    with open(path, "w") as f:
        for v in values:
            f.write(f"{int(v) & mask:0{digits}X}\n")


def sv_hex(nbits, value):
    return f"{nbits}'h{value:0{(nbits + 3) // 4}X}"


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--groups", type=int, default=10, help="groups after the lead-in (8 content + tail repeats)")
    p.add_argument("--lead-bits", type=int, default=37, help="random bits before the first group (not a multiple of 26 on purpose)")
    p.add_argument("--tail-bits", type=int, default=4)
    p.add_argument("--pi", type=lambda s: int(s, 0), default=0x5A5A)
    p.add_argument("--ps", default="FMGOLDEN")
    p.add_argument("--rt", default="GOLDEN RDS TEST!")
    p.add_argument("--rds-peak", type=float, default=0.03, help="peak RDS injection, fraction of composite full scale")
    p.add_argument("--rds-phase", type=float, default=0.0, help="RDS carrier phase vs the 57 kHz LO, radians (pi = polarity flip)")
    p.add_argument("--noise-lsb", type=float, default=0.0, help="white noise sigma added to the composite, int16 LSB")
    p.add_argument("--lpf-order", type=int, default=2)
    p.add_argument("--lpf-cutoff", type=float, default=2500.0)
    p.add_argument("--lock-run", type=int, default=3)
    p.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "tb"))
    args = p.parse_args()

    out = Path(args.out)
    rng = np.random.default_rng(args.seed)

    print("== self-test of the reference decoder pieces ==")
    rd.self_test()

    # ---- payload ---------------------------------------------------------
    groups = make_groups(args.pi, args.ps, args.rt, args.groups)
    rt_padded = args.rt.ljust(len(args.rt))
    lead = [int(b) for b in rng.integers(0, 2, args.lead_bits)]
    tail = [int(b) for b in rng.integers(0, 2, args.tail_bits)]
    data_true = list(lead)
    true_blocks = []                              # (bit position of block END in data_true, word, name)
    for grp in groups:
        for word, name in zip(grp, ("A", "B", "C", "D")):
            data_true += block_bits(word, name)
            true_blocks.append((len(data_true) - 1, word, name))
    data_true += tail
    data_true = np.array(data_true, dtype=np.uint8)

    # independent check of the encoder: every encoded block's syndrome is its offset word
    for end_i, word, name in true_blocks:
        w = data_true[end_i - 25:end_i + 1]
        assert rd.syndrome_of(w) == rd.OFFSET_WORDS[name], "encoder/decoder syndrome disagreement"
    print(f"payload: {len(lead)} lead bits + {len(true_blocks)} blocks + {len(tail)} tail bits = {len(data_true)} bits")

    enc = np.zeros(len(data_true), dtype=np.uint8)
    prev = 0
    for j, b in enumerate(data_true):             # transmitter differential encode
        prev = int(b) ^ prev
        enc[j] = prev

    n_in = DECIM * int(np.ceil((len(enc) * T_IN + 20) / DECIM))
    print(f"stimulus: {n_in} samples @ {FS_IN} Hz ({n_in / FS_IN:.3f} s), "
          f"{n_in // DECIM} after the CIC")

    # ---- S0 --------------------------------------------------------------
    stim = synthesize(enc, n_in, args.rds_peak, args.rds_phase, args.noise_lsb, rng)
    dly = verify_cic_model(rng)
    print(f"CIC model verified vs boxcar^5 FIR; pipeline delay D = {dly} input samples")
    s0, _raw = cic_model(stim)
    n0 = len(s0)
    assert n0 == n_in // DECIM
    assert np.max(np.abs(s0)) < 30000, "S0 too close to int16 range"
    print(f"S0: {n0} samples, peak |S0| = {np.max(np.abs(s0))}")

    # ---- S1 --------------------------------------------------------------
    s1, sos = lpf_reference(s0, args.lpf_order, args.lpf_cutoff)
    tol_s1 = max(4, int(np.ceil(0.01 * np.max(np.abs(s1)))))
    print(f"S1: Butterworth order {args.lpf_order} @ {args.lpf_cutoff:.0f} Hz, peak |S1| = "
          f"{np.max(np.abs(s1))}, tolerance = {tol_s1} LSB")

    # ---- S2 / S3 ---------------------------------------------------------
    phase0, top, _ = calibrate_phase0(s1)
    delta = (dly + 10) / DECIM + impulse_centroid(sos)       # signal delay in S0 samples
    pred = (-delta * STEP) % (1 << 32)
    err_samp = ((phase0 - pred + (1 << 31)) % (1 << 32) - (1 << 31)) / STEP
    print(f"NCO PHASE0 = 0x{phase0:08X} (calibrated, mean|soft| = {top:.0f}); "
          f"analytic prediction from chain delay {delta:.2f} samples = 0x{int(pred):08X}, "
          f"disagreement {err_samp:+.2f} samples")
    assert abs(err_samp) < 1.5, "calibrated symbol phase disagrees with the analytic group delay"

    soft, s_last = soft_symbols(s1, phase0)
    bits_b = (soft > 0).astype(np.uint8)          # v1 > v2 -> 1, tie -> 0 (rds_decode convention)
    tol_s3 = 41 * tol_s1
    margin = 3 * tol_s3
    ok_sym = np.abs(soft) >= margin
    skip = 2
    while skip < s_last and not ok_sym[skip:].all():
        # allow trimming only at the very end (below); leading transients grow skip
        bad_tail = np.flatnonzero(~ok_sym[skip:])
        if bad_tail.min() + skip >= s_last - 3:
            break
        skip += 1
    end = s_last
    while end > skip and not ok_sym[end - 1]:
        end -= 1
    assert ok_sym[skip:end].all()
    print(f"S3: {s_last} symbols, soft tolerance {tol_s3}, decision margin >= {margin}; "
          f"bit checks cover symbols [{skip}, {end}), min |soft| there = {np.min(np.abs(soft[skip:end]))}")

    # ---- S4 --------------------------------------------------------------
    s4 = rd.differential_decode(bits_b, invert=False)
    # truth check: NCO symbol s carries transmitted bit j = s-1
    span = range(1, end - 1)
    bad = [j for j in span if j < len(data_true) and s4[j + 1] != data_true[j]]
    assert not bad, f"demodulated data bits differ from the transmitted bits at {bad[:10]}"
    print(f"S4: demodulated bits == transmitted bits for all {len(list(span))} checked symbols "
          f"(symbol s carries bit s-1)")

    # ---- S5 / S6 ---------------------------------------------------------
    s5_from = skip + 25
    syn_w = rd.all_syndromes(s4)                  # window p -> ends at bit p+25
    syn = np.zeros(len(s4), dtype=np.int64)
    syn[25:] = syn_w
    names_by_val = {v: k for k, v in rd.OFFSET_WORDS.items()}
    types = [names_by_val.get(int(s), "") if i >= 25 else "" for i, s in enumerate(syn)]
    s6 = np.array([TYPE_CODE[t] for t in types], dtype=np.int64)
    for i in range(s5_from, min(s5_from + 200, end)):        # slow-vs-fast spot check
        assert syn[i] == rd.syndrome_of(s4[i - 25:i + 1])
    hits_true = {end_i for end_i, _w2, _n in true_blocks}
    spurious = [i for i in range(s5_from, end) if types[i] and i not in {h + 1 for h in hits_true}]
    print(f"S5/S6: {sum(1 for i in range(s5_from, end) if types[i])} offset hits in range, "
          f"{len(spurious)} of them chance matches at non-block positions "
          f"(expected ~{(end - s5_from) * 5 / 1024:.1f} for random data)")

    # ---- S7 --------------------------------------------------------------
    blocks, locked = golden_blocks(types, s4, s5_from, end, args.lock_run)
    assert locked and blocks, "golden block sync never locked"
    # independent truth check: emitted blocks are a contiguous suffix of the true block list
    true_list = [(e + 1, w2, TYPE_CODE[n]) for e, w2, n in true_blocks]   # +1: S4 index = data index + 1
    first = next(k for k, t in enumerate(true_list) if t[0] == blocks[0][0])
    for got, exp in zip(blocks, true_list[first:]):
        assert got == exp, f"S7 block {got} != transmitted {exp}"
    print(f"S7: lock after {args.lock_run} blocks, first emitted block ends at bit {blocks[0][0]} "
          f"(= transmitted block #{first}); {len(blocks)} blocks emitted, all match the transmitted words")

    # ---- S8 --------------------------------------------------------------
    pi, ps, rt = golden_content(blocks)
    assert pi == args.pi
    assert "".join(map(chr, ps)) == args.ps
    assert bytes(rt).rstrip(b"\0").decode() == rt_padded.rstrip("\0")
    hits = rd.find_offset_hits(s4)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rd.decode_content(s4, hits, len(s4))
    txt = buf.getvalue()
    ps_ref = re.search(r'name \(group 0\): "(.*)"', txt).group(1)
    rt_ref = re.search(r'RadioText \(group 2\): "(.*)"', txt).group(1)
    assert ps_ref == args.ps, f"rds_decode.py reads PS {ps_ref!r}"
    assert rt_ref == args.rt, f"rds_decode.py reads RT {rt_ref!r}"
    print(f"S8: PI=0x{pi:04X} PS={args.ps!r} RT={args.rt!r} -- cross-checked against "
          f"rds_decode.decode_content() on the same bits")

    # ---- write vectors ---------------------------------------------------
    out.mkdir(exist_ok=True)
    write_hex(out / "rds_gold_in.hex", stim, 16)
    write_hex(out / "rds_gold_s0.hex", s0, 16)
    write_hex(out / "rds_gold_s1.hex", s1, 16)
    write_hex(out / "rds_gold_s2.hex", nco_words(n0, phase0) & 0xFFFFFFFF, 32)
    write_hex(out / "rds_gold_s3_soft.hex", soft, 32)
    write_hex(out / "rds_gold_s3_bit.hex", bits_b, 8)
    write_hex(out / "rds_gold_s4.hex", s4, 8)
    write_hex(out / "rds_gold_s5.hex", syn, 16)
    write_hex(out / "rds_gold_s6.hex", s6, 8)
    write_hex(out / "rds_gold_s7_data.hex", [b[1] for b in blocks], 16)
    write_hex(out / "rds_gold_s7_type.hex", [b[2] for b in blocks], 8)

    ps_word = int.from_bytes(bytes(ps), "big")
    rt_word = int.from_bytes(bytes(rt), "big")
    lines = [
        "// Generated by tools/gen_rds_golden.py -- do not edit; re-run the script.",
        f"// seed={args.seed} groups={args.groups} lead={args.lead_bits} tail={args.tail_bits} "
        f"rds_peak={args.rds_peak} rds_phase={args.rds_phase} noise_lsb={args.noise_lsb} "
        f"lpf=butter{args.lpf_order}@{args.lpf_cutoff:.0f}Hz lock_run={args.lock_run}",
        f"localparam int N_IN       = {n_in};",
        f"localparam int N_S0       = {n0};",
        f"localparam int N_S1       = {n0};",
        f"localparam int N_S2       = {n0};",
        f"localparam int N_SYM      = {s_last};   // symbols emitted by the NCO (S3 items)",
        f"localparam int N_BIT      = {len(s4)};   // == N_SYM: one data bit per symbol",
        f"localparam int N_BLK      = {len(blocks)};",
        f"localparam int SYM_SKIP   = {skip};   // first symbol whose hard decision is checked",
        f"localparam int SYM_END    = {end};   // one past the last symbol whose hard decision is checked",
        f"localparam int S5_FROM    = {s5_from};   // first bit index whose 26-bit window lies fully in the checked range",
        f"localparam int TOL_S1     = {tol_s1};",
        f"localparam int TOL_S3     = {tol_s3};",
        f"localparam int SLACK_S0   = {SLACK[0]};",
        f"localparam int SLACK_S1   = {SLACK[1]};",
        f"localparam int SLACK_S2   = {SLACK[2]};",
        f"localparam int SLACK_S3   = {SLACK[3]};",
        f"localparam int SLACK_S4   = {SLACK[4]};",
        f"localparam int SLACK_S5   = {SLACK[5]};",
        f"localparam int SLACK_S6   = {SLACK[6]};",
        f"localparam int SLACK_S7   = {SLACK[7]};",
        f"localparam logic [31:0] NCO_STEP   = {sv_hex(32, STEP)};",
        f"localparam logic [31:0] NCO_PHASE0 = {sv_hex(32, phase0)};",
        f"localparam int LOCK_RUN   = {args.lock_run};",
        f"localparam logic [15:0]  EXP_PI = {sv_hex(16, pi)};",
        f"localparam logic [63:0]  EXP_PS = {sv_hex(64, ps_word)};   // char 0 in [63:56]",
        f"localparam logic [511:0] EXP_RT = {sv_hex(512, rt_word)};   // char 0 in [511:504], unseen = 0x00",
    ]
    (out / "rds_gold_cfg.svh").write_text("\n".join(lines) + "\n")
    print(f"\nwrote vectors + rds_gold_cfg.svh to {out}")
    print(f"NCO_STEP = 0x{STEP:08X}, NCO_PHASE0 = 0x{phase0:08X}")


if __name__ == "__main__":
    main()
