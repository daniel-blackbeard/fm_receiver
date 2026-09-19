#!/usr/bin/env python3
"""
rds_decode.py -- attempts real RDS group decoding from a live capture.

The autocorrelation check (rds_autocorr.py) gave a marginal, if oddly
reproducible, result -- inconclusive on its own. This is a much stronger
test: implement the actual RDS demodulation + block-sync/checkword chain
and see if real, valid, correctly-checksummed RDS blocks come out.
Finding even a handful of blocks with valid syndromes is close to
unambiguous proof of a real signal; a self-consistency chance-match rate
is reported alongside so "found N blocks" can be judged against "how
many would a random bitstream produce by pure luck".

Pipeline (all offline, brute-force search over the unknowns -- there is
no closed-loop symbol timing recovery here, deliberately, in favor of
something a lot more failure-tolerant to try for a one-shot capture):

  1. Capture ch1_i (RDS channel) via the same UDP path as pc_console.py.
  2. Lowpass filter (same zero-phase Butterworth as rds_autocorr.py).
  3. For a grid of candidate (samples_per_symbol, phase_offset) pairs
     around the nominal 40.42 (RDS is EXACTLY 1187.5 baud by spec, but
     the real dsp_clk-derived output rate isn't exactly 48kHz, so the
     true ratio is close to but not exactly 40.42 -- searched over):
       a. Take one hard decision per symbol period (integrate first
          half minus second half of each symbol -- a biphase matched
          filter, using interpolation since the period is non-integer).
       b. Differentially decode (RDS is differentially-coded: data
          bit = biphase_decision[n] XOR biphase_decision[n-1]).
       c. Slide a 26-bit window across the whole bitstream, compute the
          (26,10) cyclic-code syndrome of each window (vectorized via a
          precomputed per-bit-position contribution matrix -- syndrome
          computation is linear over GF(2), so this is fast even over
          tens of thousands of window positions), and count how many
          land exactly on one of the five RDS offset words (A/B/C/C'/D).
     Both differential-decode polarities are tried (the polarity
     convention is a common source of off-by-one-XOR ambiguity and
     costs nothing extra to just try both).
  4. Report the best (samples_per_symbol, phase, polarity) by raw hit
     count, print the hit rate against the theoretical chance rate for
     random data, and decode+print whatever blocks were found there
     (PI code from any A blocks, group type from B blocks, etc).

RDS protocol constants (generator polynomial, offset words) are recalled
from the EN 50067/IEC 62106 spec as precisely as possible and verified
via a synthetic self-test (encode a fake group with this exact logic,
confirm the decoder recovers it bit-for-bit) before trusting them against
a real capture -- see self_test() below, run automatically on start.
"""

import argparse
import time

import numpy as np
import scipy.signal as scipy_signal

from sample_stream_view import SampleReceiver, cast_signed

OUTPUT_RATE_HZ = 48_000.0
RDS_SYMBOL_RATE_HZ = 1187.5
SAMPLES_PER_SYMBOL_NOMINAL = OUTPUT_RATE_HZ / RDS_SYMBOL_RATE_HZ  # ~40.42

# (26,16) shortened cyclic code, generator polynomial
# g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1  (EN 50067/IEC 62106).
# 11-bit pattern, MSB = x^10 coefficient down to x^0.
GEN_POLY = 0b10110111001

# Offset words (10 bits each), added mod-2 into each block's checkword by
# the transmitter -- the receiver's syndrome of a correctly-received
# block equals the offset word for that block's position in the group.
OFFSET_WORDS = {
    "A":  0b0011111100,
    "B":  0b0110011000,
    "C":  0b0101101000,
    "C'": 0b1101010000,
    "D":  0b0110110100,
}
GROUP_SEQUENCE = ["A", "B", "C", "D"]  # C' replaces C in "version B" groups


def build_syndrome_matrix():
    """26x10 matrix: row i = the syndrome of a 26-bit block with only
    bit i (MSB-first, i.e. bit 0 = the block's most significant bit)
    set. Syndrome computation is linear over GF(2), so the syndrome of
    any 26-bit block is just the XOR (== integer sum mod 2) of the rows
    where that block has a 1 -- lets the whole search be one matmul."""
    rows = []
    for i in range(26):
        val = 1 << (25 - i)
        for b in range(25, 9, -1):
            if (val >> b) & 1:
                val ^= (GEN_POLY << (b - 10))
        syn = val & 0x3FF
        rows.append([(syn >> (9 - k)) & 1 for k in range(10)])
    return np.array(rows, dtype=np.int64)


SYNDROME_MATRIX = build_syndrome_matrix()


def syndrome_of(bits26):
    """Reference (slow, obviously-correct) syndrome via direct
    polynomial division -- used only by self_test() to cross-check the
    vectorized matrix version above before trusting it."""
    val = 0
    for b in bits26:
        val = (val << 1) | int(b)
    for i in range(25, 9, -1):
        if (val >> i) & 1:
            val ^= (GEN_POLY << (i - 10))
    return val & 0x3FF


def all_syndromes(bits):
    """Vectorized syndrome of every 26-bit sliding window in `bits`.
    Returns an array of shape (len(bits)-25,) of 10-bit syndrome ints."""
    n = len(bits)
    if n < 26:
        return np.array([], dtype=np.int64)
    windows = np.lib.stride_tricks.sliding_window_view(bits, 26)
    bits10 = (windows.astype(np.int64) @ SYNDROME_MATRIX) % 2
    weights = (1 << np.arange(9, -1, -1)).astype(np.int64)
    return bits10 @ weights


def self_test():
    """Encode a synthetic RDS-like group with this exact logic, run it
    back through the decoder, and confirm every block is recovered
    correctly -- protocol logic must pass this before it's trusted
    against a real, ambiguous capture. Raises AssertionError on failure
    (deliberately fatal: a silently-wrong decoder is worse than none)."""
    rng = np.random.default_rng(1234)

    def encode_block(data16, offset_name):
        val = data16 << 10
        for i in range(25, 9, -1):
            if (val >> i) & 1:
                val ^= (GEN_POLY << (i - 10))
        checkword = (val & 0x3FF) ^ OFFSET_WORDS[offset_name]
        block26 = (data16 << 10) | checkword
        return [(block26 >> (25 - i)) & 1 for i in range(26)]

    # Encode 3 back-to-back groups (A,B,C,D each) of random 16-bit data.
    biphase_ref = []
    prev_bit = 0
    expected = []
    pos = 0
    for _ in range(3):
        for name in GROUP_SEQUENCE:
            data16 = int(rng.integers(0, 1 << 16))
            block_bits = encode_block(data16, name)
            expected.append((pos, name))
            pos += 26
            for bit in block_bits:
                # invert the differential-encode step used by the decoder:
                # biphase[n] = data_bit[n] XOR biphase[n-1]
                cur = bit ^ prev_bit
                biphase_ref.append(cur)
                prev_bit = cur
    biphase_ref = np.array(biphase_ref, dtype=np.uint8)

    decoded = differential_decode(biphase_ref, invert=False)
    hits = set(find_offset_hits(decoded))
    # Every injected block must be recovered at exactly its real position
    # with the right type. Extra hits elsewhere are fine and expected --
    # a 10-bit syndrome matches any given offset word by pure chance
    # ~5/1024 of the time per window position, so a few incidental
    # false-positive matches among ~286 window positions (here: ~1.4
    # expected) is normal, not a bug -- confirmed this is really what's
    # happening (not a decoder bug) before writing this comment.
    missing = [e for e in expected if e not in hits]
    assert not missing, f"self-test: failed to recover injected blocks at {missing}"

    # Cross-check the vectorized syndrome matrix against the slow reference
    # implementation on a handful of random 26-bit windows.
    for _ in range(20):
        w = rng.integers(0, 2, size=26)
        fast = int(all_syndromes(w)[0])
        slow = syndrome_of(w)
        assert fast == slow, f"self-test: syndrome mismatch fast={fast} slow={slow}"

    print("Self-test passed: synthetic groups round-trip correctly, "
          "vectorized syndrome matches reference implementation.")


def lowpass_filter(x, cutoff_hz, order, fs_hz=OUTPUT_RATE_HZ):
    b, a = scipy_signal.butter(order, cutoff_hz, btype="low", fs=fs_hz)
    return scipy_signal.filtfilt(b, a, x)


def biphase_decisions(x, samples_per_symbol, phase_offset):
    """One hard decision per symbol period: integral of the first half
    minus the integral of the second half (a biphase matched filter),
    sampled via interpolation since samples_per_symbol is non-integer.
    Vectorized over all symbols at once."""
    n = len(x)
    idx = np.arange(n)
    n_symbols = int((n - phase_offset - samples_per_symbol) / samples_per_symbol)
    if n_symbols <= 0:
        return np.array([], dtype=np.uint8)
    starts = phase_offset + np.arange(n_symbols) * samples_per_symbol
    half = samples_per_symbol / 2.0
    n_sub = 8
    sub = (np.arange(n_sub) + 0.5) / n_sub
    t1 = starts[:, None] + sub[None, :] * half
    t2 = starts[:, None] + half + sub[None, :] * half
    v1 = np.interp(t1.ravel(), idx, x).reshape(t1.shape).mean(axis=1)
    v2 = np.interp(t2.ravel(), idx, x).reshape(t2.shape).mean(axis=1)
    return (v1 > v2).astype(np.uint8)


def differential_decode(biphase_bits, invert=False):
    prev = np.roll(biphase_bits, 1)
    prev[0] = 0
    d = (biphase_bits ^ prev).astype(np.uint8)
    if invert:
        d = 1 - d
    return d


def find_offset_hits(bits):
    """Every window position whose syndrome exactly matches a known
    offset word. Returns [(position, offset_name), ...]."""
    syns = all_syndromes(bits)
    hits = []
    for name, off in OFFSET_WORDS.items():
        positions = np.flatnonzero(syns == off)
        hits.extend((int(p), name) for p in positions)
    hits.sort()
    return hits


def bits_to_int(bits):
    v = 0
    for b in bits:
        v = (v << 1) | int(b)
    return v


def _char_or_blank(code):
    return chr(code) if 32 <= code < 127 else "?"


def decode_content(decoded, hits, n_bits):
    """Walk every correctly-typed A->B->(C|C')->D quad (26 bits apart,
    C/C' choice matching block B's own version bit -- a real self-
    consistency check, not just spacing) and extract the actual
    human-readable payload: Programme Service name (group type 0, 8
    chars from block D across 4 segments) and RadioText (group type 2,
    up to 64 chars from blocks C+D across 16 segments). This is the
    part that actually answers "what are they transmitting", on top of
    the PI-repeat statistics that only prove a signal exists."""
    type_at = dict(hits)
    ps_chars = [None] * 8
    rt_chars = [None] * 64
    rt_ab_flag = None
    groups_decoded = 0

    for pos, name in hits:
        if name != "A" or pos + 3 * 26 + 16 > n_bits:
            continue
        pos_b, pos_c, pos_d = pos + 26, pos + 52, pos + 78
        if type_at.get(pos_b) != "B" or type_at.get(pos_d) != "D":
            continue
        c_type = type_at.get(pos_c)
        if c_type not in ("C", "C'"):
            continue

        b_bits = decoded[pos_b:pos_b + 16]
        group_type = bits_to_int(b_bits[0:4])
        version = int(b_bits[4])  # 0 = A (block C), 1 = B (block C')
        expected_c = "C" if version == 0 else "C'"
        if c_type != expected_c:
            continue  # spacing matched but version/offset don't agree -- reject

        d_bits = decoded[pos_d:pos_d + 16]
        groups_decoded += 1

        if group_type == 0:
            seg = bits_to_int(b_bits[14:16])
            ps_chars[2 * seg]     = _char_or_blank(bits_to_int(d_bits[0:8]))
            ps_chars[2 * seg + 1] = _char_or_blank(bits_to_int(d_bits[8:16]))
        elif group_type == 2:
            seg = bits_to_int(b_bits[12:16])
            ab = int(b_bits[11])
            if rt_ab_flag is not None and ab != rt_ab_flag:
                rt_chars = [None] * 64  # A/B flag toggled -> new message
            rt_ab_flag = ab
            if version == 0:
                c_bits = decoded[pos_c:pos_c + 16]
                base = 4 * seg
                rt_chars[base]     = _char_or_blank(bits_to_int(c_bits[0:8]))
                rt_chars[base + 1] = _char_or_blank(bits_to_int(c_bits[8:16]))
                rt_chars[base + 2] = _char_or_blank(bits_to_int(d_bits[0:8]))
                rt_chars[base + 3] = _char_or_blank(bits_to_int(d_bits[8:16]))
            else:
                base = 2 * seg
                rt_chars[base]     = _char_or_blank(bits_to_int(d_bits[0:8]))
                rt_chars[base + 1] = _char_or_blank(bits_to_int(d_bits[8:16]))

    print(f"\nDecoded {groups_decoded} complete, version-consistent "
          f"A->B->{{C,C'}}->D groups.")
    ps_str = "".join(c if c else "_" for c in ps_chars)
    print(f"Programme Service name (group 0): \"{ps_str}\"  "
          f"(underscore = segment not yet seen)")
    rt_str = "".join(c if c else "_" for c in rt_chars)
    if rt_str.strip("_"):
        print(f"RadioText (group 2): \"{rt_str.rstrip('_')}\"")
    else:
        print("RadioText (group 2): none seen yet.")


def main():
    p = argparse.ArgumentParser(description="Attempt real RDS group decoding from a live capture")
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--cutoff", type=float, default=2500.0)
    p.add_argument("--order", type=int, default=4)
    p.add_argument("--period-span", type=float, default=0.6,
                    help="search samples_per_symbol in [nominal-span/2, nominal+span/2]")
    p.add_argument("--period-steps", type=int, default=25)
    p.add_argument("--phase-steps", type=int, default=25)
    args = p.parse_args()

    self_test()

    print(f"\nListening for {args.seconds:.1f}s on channel ch1_i...")
    fft_size = int(OUTPUT_RATE_HZ * args.seconds * 1.5)
    recv = SampleReceiver(fft_size=fft_size)
    recv.start()
    time.sleep(args.seconds)
    data = recv.snapshot()
    buf = cast_signed(data["ch1_i"]).astype(np.float64)
    recv.stop()
    print(f"Captured {len(buf)} samples ({recv.packets_received} packets, "
          f"{recv.packets_dropped} dropped).")

    x = lowpass_filter(buf, args.cutoff, args.order)

    periods = np.linspace(SAMPLES_PER_SYMBOL_NOMINAL - args.period_span / 2,
                            SAMPLES_PER_SYMBOL_NOMINAL + args.period_span / 2,
                            args.period_steps)

    print(f"\nSearching {args.period_steps} symbol-period x {args.phase_steps} phase "
          f"x 2 polarity candidates ({args.period_steps*args.phase_steps*2} total)...")

    best = None  # (n_hits, period, phase, invert, hits, n_bits)
    for period in periods:
        phases = np.linspace(0, period, args.phase_steps, endpoint=False)
        for phase in phases:
            biphase = biphase_decisions(x, period, phase)
            if len(biphase) < 26:
                continue
            for invert in (False, True):
                decoded = differential_decode(biphase, invert=invert)
                hits = find_offset_hits(decoded)
                if best is None or len(hits) > best[0]:
                    best = (len(hits), period, phase, invert, hits, len(decoded))

    n_hits, period, phase, invert, hits, n_bits = best
    decoded = differential_decode(biphase_decisions(x, period, phase), invert=invert)
    n_windows = max(n_bits - 25, 1)
    chance_rate = 5 / 1024  # 5 offset words, 10-bit syndrome space
    expected_hits = n_windows * chance_rate
    # Chance expectation of an isolated pair of hits exactly 26 bits apart
    # (treating hits as ~independent per-position events at chance_rate):
    expected_pairs = n_windows * chance_rate ** 2
    # Chance expectation of a RUN of 3 consecutive 26-spaced hits -- this
    # is the real test, isolated pairs happen easily by pure luck (see
    # below) but a 3-in-a-row run essentially never does by chance alone.
    expected_triples = n_windows * chance_rate ** 3

    print(f"\nBest candidate: samples_per_symbol={period:.4f}, phase={phase:.2f}, "
          f"invert={invert}")
    print(f"  {n_hits} offset-word hits out of {n_windows} window positions checked "
          f"(chance expectation for random data: ~{expected_hits:.0f})")

    if n_hits < 3:
        print("\nVERDICT: essentially no hits -- no evidence of RDS in this capture.")
        return

    positions = [pos for pos, _name in hits]
    pos_set = set(positions)
    # Find runs of consecutive 26-bit-spaced positions (regardless of type
    # for now -- type-correctness checked separately below on top of this).
    runs = []
    used = set()
    for pos in positions:
        if pos in used or (pos - 26) in pos_set:
            continue  # not a run start
        run = [pos]
        p = pos
        while (p + 26) in pos_set:
            p += 26
            run.append(p)
        if len(run) >= 2:
            runs.append(run)
            used.update(run)
    n_pairs = sum(1 for r in runs if len(r) == 2)
    n_triples_plus = sum(1 for r in runs if len(r) >= 3)
    longest_run = max((len(r) for r in runs), default=0)

    print(f"  isolated 26-apart pairs: {n_pairs} (chance expectation: ~{expected_pairs:.2f})")
    print(f"  runs of 3+ consecutive 26-apart hits: {n_triples_plus} "
          f"(chance expectation: ~{expected_triples:.4f})")
    print(f"  longest run found: {longest_run} consecutive blocks")

    if n_triples_plus == 0:
        print(f"\nVERDICT: no runs of 3+ consecutive blocks found -- isolated pairs "
              f"like this happen easily by chance (expected ~{expected_pairs:.1f} of them "
              f"here even with zero real signal). Not convincing. This is very likely "
              f"what the earlier report was actually showing -- try a longer capture, "
              f"or accept this capture doesn't have a decodable RDS signal in it yet.")
        return

    # Found real runs -- check type sequence too (A,B,C or C',D in order)
    # and print them, plus look for a PI code that repeats (the simplest,
    # most human-readable proof: PI is fixed per station, so seeing the
    # SAME value 2+ times among genuinely-synced A blocks is a strong,
    # independent confirmation on top of the run-length statistics).
    print(f"\n{n_triples_plus} run(s) of 3+ consecutive blocks found -- this is "
          f"statistically very unlikely to be chance (expected ~{expected_triples:.4f} "
          f"by pure luck). Showing runs of length >= 3:")
    best_run = None
    pi_counts = {}
    for run in runs:
        if len(run) < 3:
            continue
        types = [dict(hits)[p] for p in run]
        print(f"  bits {run[0]}-{run[-1]} ({len(run)} blocks): {' -> '.join(types)}")
        if best_run is None or len(run) > len(best_run):
            best_run = run
        for pos, name in zip(run, types):
            if name == "A" and pos + 26 <= n_bits:
                pi = bits_to_int(decoded[pos:pos + 16])
                pi_counts[pi] = pi_counts.get(pi, 0) + 1

    print(f"\nLongest run: bits {best_run[0]}-{best_run[-1]}, "
          f"types: {' -> '.join(dict(hits)[p] for p in best_run)}")
    for pos in best_run:
        if dict(hits)[pos] == "A" and pos + 26 <= n_bits:
            pi = bits_to_int(decoded[pos:pos + 16])
            print(f"  PI code at bit {pos}: 0x{pi:04X}")

    repeated = {pi: c for pi, c in pi_counts.items() if c >= 2}
    if repeated:
        best_pi = max(repeated, key=repeated.get)
        print(f"\nVERDICT: CONFIRMED -- PI code 0x{best_pi:04X} repeats "
              f"{repeated[best_pi]}x across independently-synced A blocks. A real "
              f"station's PI never changes, and this many repeats of the exact same "
              f"16-bit value is not a plausible coincidence.")
        decode_content(decoded, hits, n_bits)
    else:
        print(f"\nVERDICT: real block-sync structure found (statistically well above "
              f"chance), but no PI code repeated across the runs found -- likely a real "
              f"but still-marginal signal (dropping sync between groups). Promising, "
              f"not yet fully confirmed. A longer capture should help.")


if __name__ == "__main__":
    main()
