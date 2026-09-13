#!/usr/bin/env python3
"""
Generate a $readmemh-compatible hex file for a quarter-wave sine LUT,
using midpoint (offset) angle sampling so the address-complement trick
(cos = sin(90deg - theta) -> addr_cos = ~addr) is exact, and so no table
entry ever lands exactly on 0 deg or 90 deg.

Angle mapping (addr_bits = N, table depth = 2**N):
    angle(i) = (2*i + 1) * pi / 2**(N+2)   for i in [0, 2**N - 1]

This spans the open interval (0, 90deg), symmetric about 45deg, which is
what makes addr_cos = ~addr exact (see derivation: j = 2**N - 1 - i).

Output values are unsigned magnitudes scaled to fill [0, 2**data_bits - 1]
by default -- i.e. this LUT stores |sin(theta)| for one quadrant only.
Quadrant selection and sign are assumed to be handled by the instantiating
module (as in Daniel's rom_sincos), not by this table.
"""

import math


def gen_quarter_sine_lut(
    addr_bits: int,
    data_bits: int,
    filename: str,
    full_scale: int | None = None,
    round_mode: str = "nearest",
) -> None:
    """
    Write a quarter-wave sine magnitude LUT to `filename` in $readmemh hex format.

    addr_bits  : number of address bits -> depth = 2**addr_bits entries
    data_bits  : output word width in bits
    filename   : output .hex path
    full_scale : max unsigned code (defaults to 2**data_bits - 1, i.e. full range).
                 Since midpoint sampling never actually reaches sin(90deg) = 1.0,
                 the true max entry will be slightly below full_scale by design.
    round_mode : "nearest" (round-half-away-from-zero) or "trunc"
    """
    depth = 1 << addr_bits
    if full_scale is None:
        full_scale = (1 << data_bits) - 1

    hex_digits = (data_bits + 3) // 4  # ceil(data_bits / 4)

    with open(filename, "w") as f:
        for i in range(depth):
            angle = (2 * i + 1) * math.pi / (1 << (addr_bits + 2))
            sample = math.sin(angle) * full_scale

            if round_mode == "nearest":
                code = math.floor(sample + 0.5)
            elif round_mode == "trunc":
                code = int(sample)
            else:
                raise ValueError(f"unknown round_mode: {round_mode}")

            code = max(0, min(code, (1 << data_bits) - 1))  # safety clamp
            f.write(f"{code:0{hex_digits}x}\n")


if __name__ == "__main__":
    # Daniel's rom_sincos: 10-bit quarter-wave address, 11-bit unsigned
    # magnitude (narrowed from 12 -> 11 bits so a sign bit can be added
    # by the instantiating module while keeping the same overall signed
    # width).
    gen_quarter_sine_lut(
        addr_bits=10,
        data_bits=11,
        filename="sine_table.hex",
    )
    print("Wrote sine_table.hex: 1024 entries, 11-bit unsigned, midpoint-sampled quarter wave")