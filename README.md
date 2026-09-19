# fm_receiver

A bare-metal FM broadcast receiver, built from scratch on a Zynq-7020 +
AD9361 SDR board. No vendor SDR framework, no OS, no FSBL — the RF
front end, the entire digital demodulation chain (tuning, FM
discrimination, stereo, RDS), and the software driving it are all
custom, built up piece by piece and verified against real over-the-air
broadcasts.

**Working today, hardware-confirmed:** tune anywhere across a ~30MHz
capture window, demodulate mono/stereo FM audio, and decode RDS data
(station name, PI code) from a real broadcast — station name text
included.

## Hardware

- **HamGeek XC7Z020 + AD9361** ("Pluto clone") — Zynq-7020 (dual
  Cortex-A9 + PL fabric) with an AD9361 RF transceiver.
- RX-only in this project (TX is out of scope). Antenna into the
  AD9361's RX1A input.

## What it does

1. **Tunes** anywhere across the AD9361's captured RF band via a
   digital NCO/mixer — station selection is entirely digital, the
   AD9361's own RX LO stays fixed.
2. **Demodulates FM** via a CORDIC-based discriminator.
3. **Decodes stereo** (L/R) via a PLL that locks onto the 19kHz pilot
   tone and derives phase-coherent 38kHz/57kHz references from it.
4. **Decodes RDS**: the 57kHz RDS subcarrier is downconverted and
   CIC-decimated in RTL; the rest of the RDS chain (filtering, symbol
   timing recovery, block sync, protocol decode) currently runs
   offline in Python against the same live sample stream.
5. **Streams every stage out over Ethernet** (UDP) for live monitoring,
   audio playback, and analysis on a PC — no vendor debug core, no ILA
   needed for day-to-day work.

All of this runs on the ARM Cortex-A9 with no OS: a single bare-metal
program brings up the AD9361 over a custom SPI bridge, then serves a
register console and the sample stream over Ethernet.

## Architecture

```
 Antenna
    |
    v
 AD9361 (RF frontend, SPI-configured)
    |  LVDS (I/Q samples)
    v
 ad3961_if_rx.sv        recovers dsp_clk, decodes RX I/Q
    |
    v
 synth_core.sv + rom_sincos.sv     digital NCO/mixer -- station tuning
    |
    v
 cic_dec.sv -> fir_time_multiplexed.sv     decimate + channel-select filter
    |
    v
 cordic_vector.sv       FM discriminator (instantaneous phase -> derivative)
    |
    v
 mpx_decimator.sv        narrows the MPX composite band (240kHz)
    |
    +----------------------------------------------+
    v                                               v
 pll.sv                                      (raw MPX, for AFC feedback
 shared-CORDIC Costas loop,                   into synth_core.sv above)
 locks to the 19kHz pilot,
 derives 19/38/57kHz refs
    |
    +---------------------+------------------------+
    v                     v                        v
 mpx_demod.sv +      rds.sv                   (mono path folded into
 mpx_fir.sv           57kHz downconvert        mpx_demod.sv's L/R matrix)
 stereo L/R audio      + CIC decimate to 48kHz
    |                      |
    +----------+-----------+
               v
        axi_dsp.sv -> DDR -> axi_notifications.sv
               |
               v
        sw/eth0.c (bare-metal GEM/Ethernet driver, UDP)
               |
               v
        tools/pc_console.py  (PC: live FFT/eye-diagram display,
                               audio playback, AXI regmap console)
               |
               v
        tools/rds_autocorr.py, tools/rds_decode.py
        (RDS presence detection + full protocol decode:
         PI code, Programme Service name, RadioText)
```

A parallel path lets the PC read/write any AXI-mapped register live
(tuning frequency, AFC/reset control, RX gain, etc.) over the same
Ethernet link, or over UART1 as a fallback — see `tools/pc_console.py`'s
own docstring for the exact wire protocol.

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
  rds.sv                   RDS subcarrier downconvert + CIC decimate
  dsp.sv                    wires the whole DSP chain above together
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
  pc_console.py            main live console: FFT/eye-diagram display, AXI regmap
                            tab, audio playback, sample-rate stats
  sample_stream_view.py     shared UDP sample-stream receiver used by every tool below
  rds_autocorr.py           RDS presence detector (Manchester autocorrelation)
  rds_decode.py              full offline RDS protocol decoder (PI/PS/RadioText)
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
monitoring, tuning, and audio playback; `tools/rds_decode.py`/
`tools/rds_autocorr.py` read the same UDP sample stream for RDS
analysis. `tools/pc_console.py`'s own docstring documents the wire
protocol (UART and UDP both speak the same 8-byte register command
format).

## Status

The full chain — SPI/RX digital bring-up, digital tuning, FM
discrimination, pilot-locked stereo, and RDS reception — is
hardware-confirmed end to end. The RDS protocol decode (biphase demod
through PI/PS extraction) currently lives in Python as the reference
implementation; porting it into RTL is a planned next step.
