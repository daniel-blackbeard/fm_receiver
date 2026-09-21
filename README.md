# fm_receiver

A bare-metal FM broadcast receiver, built from scratch on a Zynq-7020 +
AD9361 SDR board. No vendor SDR framework, no OS, no FSBL — the RF
front end, the entire digital demodulation chain (tuning, FM
discrimination, stereo, RDS), and the software driving it are all
custom, built up piece by piece and verified against real over-the-air
broadcasts.

**Working today, hardware-confirmed:** tune anywhere across a ~30MHz
capture window (about 83-113MHz), demodulate mono/stereo FM audio, and
decode RDS entirely in RTL — PI code, station name (PS) and RadioText from
a real broadcast, shown live on the PC.

## Hardware

- **HamGeek XC7Z020 + AD9361** ("Pluto clone") — Zynq-7020 (dual
  Cortex-A9 + PL fabric) with an AD9361 RF transceiver.
- RX-only in this project (TX is out of scope). Antenna into the
  AD9361's RX1A input.

## What it does

1. **Tunes** anywhere across the AD9361's captured RF band via a
   digital NCO/mixer — station selection is entirely digital, the
   AD9361's own RX LO stays fixed at 98MHz. From the PC the station is
   moved with +/-10, 25 and 250kHz buttons that write the NCO phase step.
2. **Demodulates FM** via a CORDIC-based discriminator.
3. **Decodes stereo** (L/R) via a PLL that locks onto the 19kHz pilot
   tone and derives phase-coherent 38kHz/57kHz references from it.
4. **Decodes RDS in RTL**: the 57kHz subcarrier is downconverted, then a
   nine-stage chain (CIC decimation, low-pass, symbol timing, biphase and
   differential decode, (26,10) syndrome, block sync, group decode)
   produces the PI code, the 8-character station name and the 64-character
   RadioText, with a lock flag.
5. **Streams the results over Ethernet** (UDP) for live monitoring: stereo
   audio, the 19kHz PLL reference, and the decoded RDS data, displayed and
   played back on a PC — no vendor debug core, no ILA needed for
   day-to-day work.

All of this runs on the ARM Cortex-A9 with no OS: a single bare-metal
program brings up the AD9361 over a custom SPI bridge, then serves a
register console and the sample stream over Ethernet.

## Architecture

```
 Antenna
    |
    v
 AD9361 (RX1A input, LO fixed at 98MHz, 30MSPS complex, SPI-configured)
    |  LVDS (12-bit I/Q, DDR)
    v
 ad3961_if_rx.sv        recovers dsp_clk (30MHz), decodes I/Q, adc_valid
    |  30MHz
    v
 synth_core.sv + rom_sincos.sv     NCO/mixer -- station tuning (phase_step, AXI 0x0C)
    |
    v
 cic_dec.sv (R=25) -> fir_time_multiplexed.sv     1.2MHz channel-select filter
    |
    v
 cordic_vector.sv       FM discriminator: atan2(Q,I), then the phase difference
    |  1.2MHz             (leaky-integrator AFC term fed back into the NCO step)
    v
 mpx_decimator.sv        R=5 -> 240kHz composite (MPX)
    |
    +----------------------------------------------+
    v                                               v
 pll.sv                                      57kHz downconvert (dsp.sv)
 shared-CORDIC Costas loop,                          |
 locks to the 19kHz pilot,                           v
 derives 19/38/57kHz refs                        rds.sv  (S0..S8, see below)
    |                                          PI / PS / RadioText / lock
    v                                                |
 mpx_demod.sv + mpx_fir.sv                           |
 stereo L/R, 256-tap FIR, 48kHz                      |
    |                                                |
    +--------------------+---------------------------+
                         v
   debug bus, one 64-bit sample per 48kHz strobe:
       { audio_R, audio_L, RDS word, 19kHz pilot reference }
                         |
                         v
        axi_dsp.sv -> DDR (1MB ring, S_AXI_HP0) -> axi_notifications.sv
                         |
                         v
        sw/eth0.c (bare-metal GEM/Ethernet driver, zero-copy UDP)
                         |
                         v
        tools/pc_console.py  (PC: FFT display, stereo audio playback,
                               decoded RDS panel, station tuning, register console)
```

A parallel path lets the PC read/write any AXI-mapped register live
(tuning, AFC/reset control, RX gain, etc.) over the same Ethernet link, or
over UART1 as a fallback — see `tools/pc_console.py`'s own docstring for
the exact wire protocol.

### Clocks and clock-domain crossings

- **`dsp_clk` = 30MHz** — the AD9361's recovered data clock (BBPLL 960MHz,
  ADC clock 30MHz, no further decimation). All the DSP runs on it.
- **`fclk0`** (~100MHz, PS7 FCLK0) — the AXI peripherals, the DDR burst
  master (`axi_dsp`) and `axi_notifications`.
- `phase_step` and the DSP config word cross from `fclk0` into `dsp_clk`
  through a single plain register (a deliberate, documented shortcut: a
  torn word can only perturb the NCO for one cycle). The RX-mode bit uses a
  2-flop synchronizer, the reset is async-assert/sync-release, and the
  streaming hand-off inside `axi_dsp` uses a toggle flag through a 2-flop
  synchronizer.

### RDS decoder (`src/rds.sv`)

The input is the 240kHz composite multiplied by the PLL's 57kHz carrier.
Nine stages, each verified against a golden model (see Verification):

| Stage | What it does |
|---|---|
| S0 | 5th-order CIC decimator, R=5, to 48kHz |
| S1 | 2nd-order Butterworth low-pass (2.5kHz), direct form I, Q.16 coefficients with guard bits, one time-shared multiplier |
| S2 | Symbol-clock NCO: step = 1187.5/48000 of a cycle per sample |
| S3 | Biphase integrate-and-dump: (first half - second half) per symbol, a soft value and a hard bit |
| S4 | Differential decode |
| S5 | (26,10) syndrome of the last 26 bits, as a sliding window |
| S6 | Match against the five offset words (A, B, C, C', D) |
| S7 | Block sync: locks after three legal-order hits exactly 26 bits apart, then emits one block per 26 bits and drops lock on an illegal successor |
| S8 | Group decode: PI, PS (8 characters) and RadioText (64 characters) registers |

### Getting the RDS result to the PC

The decoded registers (592 bits) travel over the same 16-bit channel the
raw RDS signal used to use. A small scanner in `dsp.sv` sends, on every
48kHz strobe, one word `{addr, byte}`: it sweeps 75 addresses (PI, the
eight PS characters, the 64 RadioText characters, and a status byte whose
bit 0 is the lock flag) every 1.56ms. There is no framing — every word says
where it belongs — so a lost packet just delays one byte until the next
sweep. `tools/pc_console.py` keeps a byte table filled from those words and
shows it under the FFT panels.

### Register map and command protocol

The PS reaches the PL through AXI GP0 (`0x4000_0000` | peripheral ID << 16);
`axi_if.sv` fans the access out to the peripherals, each of which selects on
its own ID:

| ID | Peripheral | Purpose |
|---|---|---|
| 0x01 | `axi_registers` | 0x00 blink enable, 0x04 blink speed, 0x08 AD9361 pins (enable, TXNRX, RESETB release, R1 mode, bit 8 silences the sample stream), **0x0C NCO phase step**, **0x10 DSP config** (bit 0 AFC enable, bits 28-31 per-block resets) |
| 0x02 | `axi_spi` | one AXI access is one AD9361 SPI transaction |
| 0x03 | `axi_cdc_status` | read-only dsp_clk-domain diagnostics (valid/error counters, frame bits, raw sample snapshot) |
| 0x04 | `axi_notifications` | PL-to-PS status: "a new 1KB sample slice landed" |

Station selection: `phase_step = round((f_station - 98MHz) / 30MHz * 2^32)`
(mod 2^32); a positive step moves a station above the LO down to DC.

Commands use one 8-byte shape over both UART1 (115200) and UDP (port 5555,
with a `COM\0` preamble): `[device][read/write][address16][data32]`.
Devices cover the AXI registers, the AD9361 SPI, system modes (test and
mission, BIST patterns), the CDC status map, RX gain, GEM/descriptor peeks
and the RX LO (unused: the LO is fixed).

### Sample streaming

`axi_dsp.sv` packs `{ch0_i, ch0_q, ch1_i, ch1_q}` = `{audio R, audio L, RDS
word, 19kHz reference}` per 48kHz strobe into two 16-sample banks, writes
128-byte AXI3 bursts into a 1MB circular buffer in DDR, and after every
1KB updates `axi_notifications`. The firmware polls that register and sends
the slice as a UDP packet (192.168.3.9 port 5556) without copying it: a
scatter-gather TX descriptor points straight into the DDR ring. That is
384kB/s, about 375 packets a second.

## Repository layout

```
src/
  fm_receiver.sv         top-level module: PS7 wrapper + AXI peripherals + RF/DSP chain
  ad3961_if_rx.sv         AD9361 RX-only LVDS digital interface
  synth_core.sv / rom_sincos.sv    NCO-based digital downconversion (station tuning)
  cic_dec.sv               CIC decimator (channel narrowing pre-filter)
  fir_time_multiplexed.sv  decimating channel-select FIR (shared I/Q)
  cordic_vector.sv          FM discriminator (vectoring-mode CORDIC)
  cordic.sv                  shared rotation-mode CORDIC core (used by pll.sv)
  mpx_decimator.sv        narrows the MPX composite band to 240kHz
  pll.sv                    shared-CORDIC 3-VCO Costas loop, pilot-locked
  mpx_demod.sv / mpx_fir.sv  stereo (L/R) demod + decimating audio FIR
  rds.sv                   RDS decoder: CIC, low-pass, symbol timing, biphase/differential
                            decode, block sync, PI/PS/RadioText
  dsp.sv                    wires the whole DSP chain above together and drives the
                            debug bus (audio, pilot reference, RDS scanner)
  axi_if.sv                 AXI3<->peripheral-bus protocol bridge
  axi_registers.sv           general-purpose AXI regmap
  axi_spi.sv                  AD9361 SPI bridge (PL bank I/O, not a hard SPI controller)
  axi_cdc_status.sv            dsp_clk-domain read-only status regmap
  axi_dsp.sv                    streams demodulated channels to DDR
  axi_notifications.sv           PL->PS "new data" signaling, no DMA/IRQ needed
tb/                       one testbench per RTL module of note (cycle-accurate,
                           golden-vector or synthetic-stimulus verified before
                           anything gets trusted on real hardware)
sw/
  startup.S / linker.ld / main.c   bare-metal ARM Cortex-A9 app, no OS/FSBL/libc
  eth0.c / eth0.h                   bare-metal GEM (Ethernet) MAC driver
  build.bat                          assembles/links into sw/build/fm_axi_test.elf
tools/
  pc_console.py            main live console: FFT display, decoded-RDS panel, station
                            tuning buttons, AXI regmap tab, stereo audio playback,
                            sample-rate stats
  sample_stream_view.py     shared UDP sample-stream receiver used by every tool below
  rds_autocorr.py           RDS presence detector (Manchester autocorrelation)
  rds_decode.py              full offline RDS protocol decoder (PI/PS/RadioText),
                            the golden reference for rds.sv
  gen_rds_golden.py          synthetic RDS stream + per-stage golden vectors for
                            tb/tb_rds_golden.sv
  rate_counter.py            minimal ground-truth sample-rate tool
  gen_sincos_lut.py / gen_fir_taps.py / gen_mono_lpf_taps.py / gen_pll_stim.py
                              coefficient/LUT/stimulus generators for the RTL above
  soak_udp_traffic.py        long-running UDP traffic generator for soak testing
```

This repo covers the RTL, firmware, testbenches, and PC-side tools —
the actual code. The Vivado build flow, pin/timing constraints, and PS7
block design used to build and deploy this locally are not included
here.

## Tools

Point `tools/pc_console.py` at a running board (over Ethernet) for live
monitoring, tuning and audio playback:

- **Command Console tab** — station tuning (six buttons: -250, -25, -10,
  +10, +25, +250kHz, starting at 98.000MHz and limited to 83-113MHz), mission
  and test modes, RX gain control, an AD9361 PLL-lock check, and a raw
  register send panel.
- **AXI Regmap tab** — bulk read and decode of the axi_registers words.
- **RX Sample Stream tab** — FFT panels for the right and left audio
  channels and the 19kHz PLL reference (with measured tone frequency and
  purity), optional stereo audio playback, sample-rate statistics, and the
  **decoded RDS panel** (PI, PS, RadioText and lock).

`tools/rds_decode.py` and `tools/rds_autocorr.py` read the same UDP sample
stream for offline RDS analysis; `rds_decode.py` is also the reference the
RTL decoder was checked against. `tools/pc_console.py`'s own docstring
documents the command wire protocol (UART and UDP both speak the same
8-byte register command format).

## Verification

Every RTL module of note has a cycle-accurate testbench (`tb/`), verified
against golden vectors or synthetic stimulus before anything is trusted on
hardware. The RDS decoder has a single testbench, `tb/tb_rds_golden.sv`,
that scores all nine stages in one pass. Its stimulus and per-stage golden
vectors come from `tools/gen_rds_golden.py`: a synthetic 240kHz stream built
from a known PI/PS/RadioText, encoded through the real block code and the
composite MPX chain, whose own outputs are cross-checked against
`rds_decode.py`. Stages are compared bit-exactly where the arithmetic is
fixed, and within a stated tolerance for the filter and the soft decision.

## Status

The full chain — SPI/RX digital bring-up, digital tuning, FM
discrimination, pilot-locked stereo and RDS reception including the RDS
protocol decode (biphase demod through PI, PS and RadioText) — is
hardware-confirmed end to end. A live broadcast decodes with the correct
PI code, station name and RadioText.

Known limitations:

- **RDS lock depends on signal quality.** On a weak or noisy signal the
  decoder drops lock and re-acquires; block sync drops on the first bad
  block, and there is no error correction.
- **The symbol clock is a fixed nominal 48kHz-derived step.** The real
  decimated rate differs from nominal by a few parts per million, which
  slowly walks the symbol timing over minutes.
- **Reprogramming the bitstream while the ARM app is running can wedge the
  debug port and the PS-PL bus;** a power cycle is the only recovery, so
  power-cycle before programming. A JTAG ELF-only reload does not reset the
  AD9361, so RF and rate measurements are only meaningful after a real
  bitstream reprogram or power cycle.
- UDP command replies can be unreliable under heavy sample-stream load; the
  UART transport in the console is the fallback.
