#!/usr/bin/env python3
"""Generate the pilot stimulus for tb/tb_pll.sv: a 19kHz tone sampled at
240kHz (the PLL's strb rate), starting phase 45deg, stepping to 19005Hz at
t=50ms for 200ms, then back to 19000Hz for the remainder of a 1s run.
Phase is kept continuous across the step (real frequency-step behavior,
not a phase discontinuity). Output: tb/pll_stim.hex, Q0.15, one 4-hex-digit
two's-complement value per line, for $readmemh.
"""
import numpy as np

FS = 240_000
DURATION = 1.0
N = int(FS * DURATION)

F_NOMINAL = 19_000
F_STEP = 19_005
T_STEP_START = 0.050
STEP_DURATION = 0.200
T_STEP_END = T_STEP_START + STEP_DURATION

START_PHASE = np.pi / 4

dt = 1.0 / FS
phase = START_PHASE
samples = np.zeros(N, dtype=np.int16)

t = 0.0
for n in range(N):
    f = F_STEP if (T_STEP_START <= t < T_STEP_END) else F_NOMINAL
    val = np.sin(phase)
    q = int(round(val * 32767))
    q = max(-32768, min(32767, q))
    samples[n] = q
    phase += 2 * np.pi * f * dt
    t += dt

with open("tb/pll_stim.hex", "w") as fo:
    for s in samples:
        fo.write(f"{int(s) & 0xFFFF:04x}\n")

print(f"wrote {N} samples to tb/pll_stim.hex")
print(f"step: {F_STEP}Hz from t={T_STEP_START*1000:.0f}ms to t={T_STEP_END*1000:.0f}ms, "
      f"{F_NOMINAL}Hz otherwise, start phase={np.degrees(START_PHASE):.0f}deg")
