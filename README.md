# fm_receiver — Zynq-7020 PS7 + AXI + bare-metal bring-up

This project builds a Zynq-7020 design using pure non-project batch-mode
Vivado (`vivado -mode batch -source scripts/foo.tcl`, no `.xpr` ever
created). The PS7 (Zynq processing system) is reused from a HamGeek vendor
reference project rather than hand-derived, since DDR3L timing and MIO/clock
parameters are risky to get right by hand from the schematic; everything
else (RTL, AXI peripherals, constraints, bare-metal software) is written
from scratch.

**Current state: AXI + UART + SPI-to-AD9361 confirmed working on real
hardware; the AXI peripheral architecture has since been substantially
refactored and re-verified in simulation and synthesis, not yet re-tried on
real hardware.** The ARM Cortex-A9 core, running a bare-metal C program with
no FSBL/BSP/OS, drives PS7's hard UART1 peripheral directly (no PL
involvement) as an interactive register console: an 8-byte binary command
over UART1 triggers a register read/write, routed through `axi_if` — a thin
AXI3-to-internal-bus protocol bridge — to whichever of three PL peripherals
claims the address: `axi_registers` (a general-purpose regmap; live writes
visibly change the LED's blink rate), `axi_spi` (a custom SPI master
bridging to the AD9361, since its SPI pins land on plain PL bank I/O rather
than either of PS7's hard SPI controllers), and `axi_cdc_status` (a
read-only status regmap native to the recovered `dsp_clk` domain, not yet
fed real data). The AD9361 itself is verified alive and responding
correctly over SPI: `REG_PRODUCT_ID` reads back the expected product ID, and
a write/readback round-trip on another register confirms both SPI
directions work.

An intermittent full-console hang affected earlier stages of this bring-up;
see "Intermittent hang investigation" below for the full story — currently
mitigated via MMU/instruction-cache configuration in `sw/startup.S`.

**AD9361 RX digital bring-up: BBPLL locked, RX clock-divider chain
configured, the RX LO synthesizer (98MHz) derived and locked, and the LVDS
RX interface confirmed genuinely live on real hardware** — `REG_STATE`
reaches real RX (`0x08`) automatically at boot, and `rx_data` changes on
every sample with the coarse chip-side delay tuned correctly. This is now
permanently in `sw/main.c` (`ad9361_common_init()`), not just hand
JTAG/SPI verification — reaching real RX state turned out to be common to
both test and mission mode (BIST/PRBS only ever flowed *because* of it),
so it runs unconditionally at boot rather than behind a mode-select
command; `enter_test_mode()`/`enter_mission_mode()` now hold only their
genuinely mode-specific tail ends. See "AD9361 RX digital bring-up" below
for the full story. This part of the bring-up predates the AXI refactor
and is unaffected by it.

**The AXI peripheral architecture was rewritten** from a single hand-written
`axi_fm` slave plus a small `axi_interconnect` address-range mux, to a
shared `axi_if` protocol bridge fanning out to independent, self-selecting
peripheral modules (`axi_registers`, `axi_spi`, `axi_cdc_status`) — see
"AXI peripherals" below for the current architecture and why. This included
catching and fixing a real bug before it ever reached hardware: the
peripheral-select field was initially compared against `addr[31:24]`, which
a real `M_AXI_GP0` transaction can never actually present (see the address-map
section). All four testbenches (`tb_axi_fm.sv`, `tb_axi_spi.sv`,
`tb_axi_cdc_status.sv`, `tb_fm_receiver.sv`) pass, and the full Vivado flow
(synthesis → implementation → bitstream) is clean — 0 errors, all timing
constraints met, 0 DRC violations — but this hasn't been re-tried on real
hardware yet since the refactor landed.

This doc covers the whole pipeline: PS7 configuration, synthesis/bitstream,
the AXI peripherals, the PS7 UART command console, and the bare-metal
software + JTAG bring-up flow.

## Directory map

```
src/
  fm_receiver.sv        top-level module: PS7 wrapper + axi_if + axi_registers + axi_spi + axi_cdc_status + ad3961_if_rx
  axi_if.sv              AXI3<->peripheral-bus protocol bridge (no peripheral logic of its own)
  axi_registers.sv        general-purpose 8 x 32-bit AXI read/write regmap (PERIPH_ID 0x01)
  axi_spi.sv               direct single-transaction AD9361 SPI bridge (PERIPH_ID 0x02) — hand-written by the project owner
  axi_cdc_status.sv         64 x 8-bit read-only status regmap, native to dsp_clk, toggle-handshake CDC (PERIPH_ID 0x03)
  axi_interconnect.sv        retired — no longer instantiated anywhere, kept pending a deletion decision
  ad3961_if_rx.sv        AD9361 RX-only LVDS digital interface (recovers dsp_clk, decodes I/Q samples)
  PS/bd/design_1/                      versioned PS7 block design (source of truth)
    design_1.bd                        block design definition (JSON)
    ip/design_1_processing_system7_0_0/
      design_1_processing_system7_0_0.xci   the actual PS7 IP customization
tb/
  tb_axi_fm.sv           axi_if + axi_registers (filename predates the axi_registers rename, kept for continuity)
  tb_axi_spi.sv           axi_if + axi_spi (+ a modeled AD9361 SPI slave)
  tb_axi_cdc_status.sv     axi_if + axi_cdc_status, drives both clock domains independently
  tb_fm_receiver.sv         integration: all three peripherals sharing axi_if at once — the OR-dones/AND-no_addrs/mux-rdata combining logic fm_receiver.sv itself uses
sw/
  startup.S                            Cortex-A9 entry point, vector table, VBAR/stack setup
  linker.ld                            places the app in DDR at 0x00100000
  main.c                               bare-metal UART register console: dispatches to axi_registers, axi_spi, or (not yet) axi_cdc_status
  build.bat                            assembles/compiles/links sw/ into sw/build/fm_axi_test.elf
scripts/
  common.tcl            shared part/top/path variables, sourced by everything else
  ps7_configure.tcl      SOURCE-MODIFYING: edits + saves design_1.bd itself
  ps7.tcl                 regenerates HDL/.xdc from design_1.bd into output/ (build step)
  synth.tcl / impl.tcl / bitstream.tcl / program.tcl   the rest of the Vivado build flow
  sim.tcl                 behavioral simulation — one `set tb "..."` line selects which of the four testbenches above elaborates as top
  ps7_jtag_init.tcl       xsdb (not Vivado): brings up PS7's own registers over JTAG
  load_run_app.tcl        xsdb (not Vivado): loads + runs the bare-metal ELF over JTAG
  run_*.bat               thin wrappers: source env.bat, invoke vivado/xsdb, log to output/
.vscode/tasks.json      VS Code tasks wrapping each run_*.bat / sw/build.bat
```

**File ownership.** `axi_spi.sv` is hand-written by the project owner —
Claude only touches formatting/whitespace there on explicit request, never
logic. `axi_if.sv`, `axi_registers.sv`, `axi_cdc_status.sv`, and all four
testbenches are regular, actively-developed project files with no such
restriction.

See `explanation.md` for a field-by-field breakdown of what's inside
`design_1.bd`/`.xci`, and `blinky.md` for the original phased plan this
build grew out of (written before the AXI/bare-metal phases below existed —
now mostly of historical interest, see "Other docs" at the bottom).

## Two very different kinds of Vivado script here

- **`ps7_configure.tcl` modifies source.** It rewrites `design_1.bd`/`.xci`
  under `src/PS/`, which are meant to be committed. Run it only when you
  deliberately want to change the PS7's configuration, then review/commit
  the resulting diff like any other source change. It is *not* part of the
  build chain (`Vivado: Full Flow` does not depend on it).
- **`ps7.tcl` produces disposable build output.** It reads whatever is
  currently committed in `design_1.bd` and turns it into plain Verilog +
  `.xdc` under `output/` (gitignored). It has no opinion about what the PS7
  should be configured as — that's entirely `ps7_configure.tcl`'s job.

## Prerequisites

- **Vivado 2025.2.1**, installed at `C:\AMDDesignTools\2025.2.1` — path is
  hardcoded in `scripts/env.bat`, update there if your install differs.
- **Arm GNU Toolchain** (`arm-none-eabi-gcc`, `-ld`, `-objcopy`) on your
  `PATH`. Installed standalone via the official `.msi`
  (`arm-gnu-toolchain-*-mingw-w64-x86_64-arm-none-eabi.msi`) — MSYS2 is not
  needed for this project. `sw/build.bat` calls these tools directly by
  name, so a fresh shell (not just the one they were installed from) needs
  to see them on `PATH`.
- **`xsdb`** (Xilinx System Debugger) reachable on `PATH` for the two JTAG
  bring-up scripts — ships with the Vivado/Vitis install.
- Board connected over JTAG and powered on for any `XSDB:` or
  `Program FPGA` task.

## Build & bring-up sequence (VS Code tasks, in order)

Run via the Command Palette → "Tasks: Run Task" (the task picker sometimes
fails to open from its usual shortcut — see the TODO list below; Command
Palette reliably works).

1. **`Vivado: Configure PS7 Clocks (modifies src/PS)`** — only needed once,
   or after deliberately changing PS7 config (see below). Not part of the
   regular rebuild loop.
2. **`Vivado: Full Flow`** — runs Synthesize → Implement → Bitstream in
   sequence. Produces the `.bit` file under `output/vivado/bitstream/`.
3. **`Vivado: Program FPGA`** — downloads the bitstream over JTAG via
   Vivado's hardware manager. This configures the PL fabric only — PS7's
   own internal state (clocks, DDR controller, MIO muxing) is untouched by
   this step.
4. **`XSDB: Init PS7 over JTAG (run after Program FPGA)`** — brings up
   PS7's registers (clock generators, DDR controller) over JTAG. Required
   because this flow has no FSBL: normally an FSBL does this automatically
   during boot; here nothing does it unless this script runs. **Must
   happen exactly once per power-on session**, after `Program FPGA` and
   before anything DDR-dependent.
5. **`SW: Build Bare-Metal App`** — assembles/compiles/links `sw/` into
   `sw/build/fm_axi_test.elf`. Independent of steps 1–4; re-run any time
   just the C/assembly source changes.
6. **`XSDB: Load & Run Bare-Metal App (run after PS7 JTAG Init)`** —
   downloads the ELF into DDR over JTAG and starts the core running from
   `_start`. Repeatable: as long as PS7 init (step 4) hasn't been undone by
   a power cycle, you can loop steps 5→6 freely while iterating on
   `sw/main.c` without re-touching the FPGA side at all.

`Vivado: Simulate` runs `scripts/sim.tcl`, which elaborates whichever
testbench its `set tb "..."` line currently names — change that line and
re-run to switch between `tb_axi_fm`, `tb_axi_spi`, `tb_axi_cdc_status`, and
`tb_fm_receiver`.

## Changing the PS7's configuration (clocks, AXI ports, anything else)

Edit the `set_property` calls in `scripts/ps7_configure.tcl`, then run it
(`scripts/run_ps7_configure.bat`, or the VS Code task from step 1 above).
It:

1. Opens `design_1.bd` in a disposable helper project under `output/`.
2. Upgrades the PS7 cell if needed (see gotcha below).
3. Applies the requested `CONFIG.*` properties.
4. Validates, reads the result back, and **aborts before saving** if a
   property didn't actually apply — rather than silently writing a
   half-applied config.
5. Saves back into the real `src/PS/bd/design_1/design_1.bd`.
6. Clears the stale generated-HDL cache under `output/vivado/ps7_gen`, so
   the next build regenerates from the new config instead of reusing an old
   one.

### Gotchas learned getting FCLK0 + M_AXI_GP0 working (apply these to any future change)

- **IP revision lock.** `design_1.bd` was authored under Vivado 2021.1; a
  newer install's IP catalog carries a different `processing_system7`
  revision, which locks the cell until upgraded. Fix: `upgrade_bd_cells
  [get_bd_cells -hierarchical]` right after `open_bd_design`, before
  touching any properties.
- **Some parameters gate others.** `PCW_FPGA_FCLK0_ENABLE` refused to apply
  ("Attempt to set value '1' on disabled parameter ... is ignored") until
  `PCW_EN_CLK0_PORT` was set first — that's the switch that makes the pin
  exist on the cell at all. Set gating parameters as **separate
  `set_property` calls**, not bundled in one `-dict`, and in the right
  order — mirrors how the GUI greys out a field until its prerequisite
  checkbox is ticked.
- **Enabling a pin ≠ exposing it.** Turning on `PCW_EN_CLK0_PORT` only
  creates the pin on the `processing_system7_0` cell inside the BD — it does
  *not* put it on the block design's boundary. Without an explicit
  `make_bd_pins_external [get_bd_pins processing_system7_0/<pin>]`, the
  signal won't appear on `design_1_wrapper.v` at all. Bus interfaces (like
  `M_AXI_GP0`) need the interface-level equivalent,
  `make_bd_intf_pins_external`, instead.
- **A clock-enable pin still needs a real clock source.** Enabling
  `PCW_USE_M_AXI_GP0` adds an `M_AXI_GP0_ACLK` input that
  `validate_bd_design` will reject as unconnected unless something drives
  it — normally Connection Automation does this for you. Here it's wired
  directly with `connect_bd_net` from the PS7 instance's own `FCLK_CLK0`
  output (a loopback: out through PL routing, back into the same PS7
  instance's GP0 ACLK input), since the AXI peripheral consuming GP0 lives
  in that same clock domain anyway.
- **Check the real pin name, don't guess polarity.** The reset pin is
  `FCLK_RESET0_N` — active-**low** — confirmed by grepping how the vendor's
  own ADI-derived reference design consumes it
  (`zc706_system_bd.tcl`: `sys_rstgen/ext_reset_in ← sys_ps7/FCLK_RESET0_N`),
  not assumed from the parameter name.
- **The script wasn't idempotent at first.** `connect_bd_net`,
  `make_bd_pins_external`, and `make_bd_intf_pins_external` all errored
  outright (rather than silently no-op'ing) if re-run against a pin/net/port
  that a previous run had already created and saved — e.g.
  `connect_bd_net` failing with `all ports/pins are already connected to
  '/processing_system7_0_FCLK_CLK0'`. Fixed by guarding each call with a
  check-before-acting `if {[get_bd_ports ...] eq ""} { ... }` (or the
  `get_bd_nets`/`get_bd_intf_ports` equivalent). The script is now safely
  re-runnable any time, whether or not there's an actual change to apply —
  useful since you'll want to come back to this file for future config
  changes without worrying about what state it's already in.

## Building (regenerating HDL + synthesizing)

`scripts/ps7.tcl` (sourced from `scripts/synth.tcl`) does the HDL
generation step:

```tcl
if {![file exists "$ps7GenDir/hdl/design_1_wrapper.v"]} {
    create_project -force ps7_gen $ps7GenProj -part $part
    add_files -norecurse $ps7BdFile
    generate_target all [get_files design_1.bd] -force
    close_project
}
```

This produces, under `output/vivado/ps7_gen/...`, four files `synth.tcl`
needs to read alongside your own RTL:

- `hdl/design_1_wrapper.v` — the instantiable top wrapper around the BD
- `synth/design_1.v` — the BD's own top-level netlist (connects
  `processing_system7_0`'s ports to the BD boundary); without this,
  `synth_design` fails with `module 'design_1' not found` the moment the
  wrapper is instantiated
- `ip/design_1_processing_system7_0_0/synth/design_1_processing_system7_0_0.v`
  — thin per-instance wrapper around the PS7 core
- `ip/design_1_processing_system7_0_0/hdl/verilog/processing_system7_v5_5_processing_system7.v`
  — the actual PS7 primitive definition; instantiates the native Xilinx
  `PS7` library cell, resolved natively by the synth engine, no further
  file needed after this one
- `ip/design_1_processing_system7_0_0/design_1_processing_system7_0_0.xdc`
  — MIO/DDR/FIXED_IO `PACKAGE_PIN`/`IOSTANDARD` constraints; a project-mode
  flow applies this automatically, this non-project flow doesn't, so
  `synth.tcl` reads it explicitly

```tcl
source ./scripts/common.tcl
source ./scripts/ps7.tcl

read_verilog -sv {*}$rtl_files
read_verilog $ps7WrapperV
read_verilog $ps7BdNetlistV
read_verilog $ps7CoreV
read_verilog $ps7PrimitiveV
read_xdc $ps7Xdc
read_xdc $projRoot/constraints/constraints.xdc

synth_design -top $top -part $part
```

**Cache invalidation:** `ps7.tcl`'s regeneration guard only checks whether
`design_1_wrapper.v` already exists, not whether `design_1.bd` changed since
then. `ps7_configure.tcl` clears this cache automatically after a save, but
if you ever edit `design_1.bd` some other way, delete
`output/vivado/ps7_gen` yourself before rebuilding.

**Clock constraint for `fclk0`:** it's an internal net (`design_1_wrapper`'s
`FCLK_CLK0` output, feeding the counter and `axi_if` in `fm_receiver.sv`),
not a top-level port, so it can't be constrained via `get_ports`.
`constraints.xdc` constrains it by exact pin path instead, found from
`post_route_timing.rpt`'s own `no_clock` check the first time it was
missing:

```tcl
create_clock -name fclk0 -period 10.000 \
    [get_pins {ps_u/design_1_i/processing_system7_0/inst/PS7_i/FCLKCLK[0]}]
```

**CDC exceptions for the `dsp_clk` crossings** live right after it: a
`set_false_path -to` for each synchronizer's first stage
(`adc_r1_mode_dsp_reg` for the `fclk0`→`dsp_clk` mode-select sync,
`u_axi_cdc_status/r_req_toggle_meta_reg` and
`u_axi_cdc_status/r_ack_toggle_meta_reg` for `axi_cdc_status`'s read
handshake). Missing one of these isn't cosmetic — leaving
`axi_cdc_status`'s new synchronizers unconstrained produced a genuine hold
violation at synthesis (`-0.622ns`) the first time, since Vivado's default
STA has no way to know a 2FF synchronizer's first stage is deliberately
asynchronous.

`clk_i` (the design's original standalone-clock input, `PACKAGE_PIN N18`)
is still declared as a top-level port and constrained, but is otherwise
unused now that `fclk0` (from PS7) clocks everything — kept deliberately
rather than removed.

## Wiring the PS7 + AXI peripherals into RTL

`fm_receiver.sv` **is** the top-level module (no separate `system_top.sv`
wrapper — that was an earlier plan, superseded once the AXI peripherals
needed direct access to the same `DDR_*`/`FIXED_IO_*`/AXI signal set
anyway). It:

- Declares all `DDR_*`/`FIXED_IO_*` ports as `inout` (widths copied
  verbatim from the generated `design_1_wrapper.v`, never hand-typed).
- Declares `spi_csn`/`spi_clk`/`spi_mosi`/`spi_miso` as top-level ports —
  the AD9361's SPI bus is plain PL bank I/O (pins already fixed in
  `constraints/constraints.xdc`: `spi_csn`=R17, `spi_clk`=V18,
  `spi_mosi`=P16, `spi_miso`=V17), not a PS7 hard peripheral. Driven
  directly by `axi_spi`'s `spi_ce`/`spi_ck` (generic chip-enable/clock
  naming, mapped to this board's pins at the instantiation).
- Declares `enable`/`gpio_resetb`/`txnrx` as top-level outputs — the
  AD9361's `ENABLE`, active-low `RESETB`, and `TXNRX` pins
  (`constraints.xdc`: T15/R19/P18) — driven straight from three bits of
  `axi_registers`' `regmap` (`regmap[64]`/`regmap[66]`/`regmap[65]`, see
  the register table below). `gpio_resetb` was originally left
  unconnected (floating), which held the AD9361 in reset/sleep with an
  inactive SPI interface; wiring it fixed the chip not returning valid
  SPI data.
- Declares `fclk0`/`fclk0_rstn` and the full 36-signal `m_axi_gp0_*` AXI3
  bus as internal wires (never top-level ports — the bus never leaves the
  chip).
- Instantiates `design_1_wrapper` (PS7), whose `m_axi_gp0_*` master feeds
  `axi_if` directly (no interconnect layer at all). `axi_if` turns AXI3
  into a plain `wen`/`ren` peripheral bus, broadcast identically to
  `axi_registers`, `axi_spi`, and `axi_cdc_status` in parallel — each one
  self-selects by comparing `addr[23:16]` against its own `PERIPH_ID`, and
  their individual `done`/`no_addr`/`rdata` get combined back into the
  signals `axi_if` sees: dones OR'd together, `no_addr`s AND'd together
  (so `axi_if` only replies DECERR once *every* peripheral has said "not
  me"), `rdata` muxed by whichever peripheral's `rdone` is high — safe
  since `PERIPH_ID` match is mutually exclusive by construction. See
  `src/axi_if.sv`'s port-list comment for the full contract.
- Drives the LED from `axi_registers`' register output:

```systemverilog
wire [255:0] regmap;
wire         blink_en    = regmap[0];
wire [7:0]   blink_speed = regmap[39:32];

always_ff @(posedge fclk0) counter <= counter + 32'b1;

assign gpio_3p3_1 = blink_en & counter[blink_speed[4:0]];
```

**Historical: `axi_interconnect` gotchas.** The retired `axi_interconnect`
(1 master → N slaves, address-range decode) had two real bugs worth
remembering even though the module itself is dead code now, because the
underlying lessons carried forward into `axi_if.sv`'s design: (1) route
*every* AXI channel field faithfully, including `AWID`/`ARID`/`WID`/`BID`/
`RID` and `LOCK`/`CACHE`/`PROT`/`QOS` — a hardcoded/unmatched `BID`/`RID`
stalls a real AXI3 master forever on the very first access, invisible to a
testbench that doesn't replicate a real master's ID-matching enforcement;
(2) AXI3's `LEN` is 4 bits, not AXI4's 8 — declaring it 8 bits synthesizes
with a silent width-truncation warning instead of an error.

## AXI peripherals

Every peripheral shares the same contract with `axi_if`: `wen`/`ren` are
held (a level, not a pulse) until the peripheral asserts `done` or
`no_addr`; `addr[23:16]` must match the peripheral's own `PERIPH_ID`;
`addr[15:0]` is that peripheral's own internal address space. `addr[31:24]`
is never inspected by any peripheral — see the address-map section below
for why.

### `axi_registers` (PERIPH_ID `0x01`)

General-purpose AXI read/write regmap. Exposes 8 word-aligned 32-bit
registers as one packed `reg_out[255:0]` output; `fm_receiver.sv` slices
the bits it cares about out of that. Combinational `done`/`no_addr` — same
1-cycle write/read latency as a plain register file.

| Offset | Register | Bits used | Meaning |
|--------|----------|-----------|---------|
| `0x00` | `CTRL`   | `[0]`     | Blink enable |
| `0x04` | `DIV`    | `[4:0]`   | Which `counter` bit drives the LED (blink rate) |
| `0x08` | (unnamed) | `[0]`=`enable`, `[1]`=`txnrx`, `[2]`=`gpio_resetb`, `[3]`=`adc_r1_mode` | AD9361 physical control pins + RX decode-path select (see wiring section above) — `gpio_resetb` is active-low, write 1 to release reset |

### `axi_spi` (PERIPH_ID `0x02`)

A direct, single-transaction bridge to a custom SPI master driving the
AD9361 (see `ad9361_registers.md` for the chip's own register map and SPI
framing) — not a register file. **One AXI write is one complete AD9361 SPI
write**: the AD9361 register address goes in `addr[11:2]` (not
`addr[9:0]` directly — shifted up by 2 so the resulting address stays
word-aligned regardless of the AD9361 register number, which isn't
restricted to multiples of 4), the data byte in the write data. **One AXI
read is one complete SPI read** the same way, using `addr[11:2]`. Both
block on the AXI bus itself (`BVALID`/`RVALID`) until the real ~24-`SPI_CLK`-edge
transaction finishes — no separate launch/status registers, no software
polling loop. (An earlier design staged the transaction through
CTRL/ADDR/WDATA/STATUS registers; that's been fully replaced.)

**Real bug caught on real hardware**: the first version placed the AD9361
address directly at `addr[9:0]`, which produces an unaligned pointer for
any register number that isn't a multiple of 4 (most of them) —
`REG_CTRL` (`0x3DF`) faulted on the very first SPI write with an ARM Data
Abort (Alignment Fault, `DFSR=0x801`) before the access ever reached the
AXI bus at all. Simulation never exercised real ARM load/store alignment
rules, so this only surfaced on hardware — see the halt-and-inspect
procedure below for how it was diagnosed.

### `axi_cdc_status` (PERIPH_ID `0x03`)

Read-only from the AXI side — any write matching this `PERIPH_ID` comes
back DECERR unconditionally, since there's nothing here for software to
legitimately write. 64 x 8-bit lanes (`dsp_reg_out[511:0]`), storage native
to the `dsp_clk` domain: `dsp_clk`-side logic writes freely via a plain
synchronous `dsp_wen`/`dsp_windex`/`dsp_wdata` port (no CDC needed, it's
already home), and reads that same array combinationally. AXI reads cross
into `fclk0` via a **toggle-synchronized** request/ack handshake (not
level-based — see the false-path note above for why that distinction
mattered in practice) — see `src/axi_cdc_status.sv`'s header comment for
the exact mechanism. Extended from the original 16 lanes to 64 to leave
headroom (reg10-63) for a correlator and a variable-time-window histogram
module, both planned as the user's own next DSP work once real RX data is
flowing.

**Currently wired to a diagnostic mirror, not the final producer**:
`fm_receiver.sv` cycles a free-running `cdc_mirror_seq` counter that writes
reg0-9 round-robin with debug taps (`rx_data` snapshot, valid/error
counters, raw `rx_frame_s`/`rx_data` pre-decode taps, a `dsp_counter`
heartbeat) — built to diagnose the RX interface bring-up (see "AD9361 RX
digital bring-up" above), not the real sample valid/error counter logic
this peripheral was originally designed for (see "LED-based debugging"
below — haven't been built).

## Zynq-7000 GP0/GP1 address map — read before picking any AXI base address

Per the official Zynq-7000 Technical Reference Manual (UG585 Table 4-1,
"System-Level Address Map"), the two general-purpose AXI master ports from
PS to PL each get a **fixed, non-configurable** 1GB slice of the CPU's 4GB
address space:

| Address range | Port |
|---|---|
| `0x4000_0000`–`0x7FFF_FFFF` | `M_AXI_GP0` |
| `0x8000_0000`–`0xBFFF_FFFF` | `M_AXI_GP1` |

This split happens **inside the PS7 hard macro itself**, before a
CPU-issued address ever reaches PL logic — not something any PL-side RTL
can see or influence. **This design only wires up `M_AXI_GP0`** (confirmed
by grepping the generated `design_1_wrapper.v` — zero `gp1`-related signals
exist anywhere), so **every AXI-reachable peripheral base address must stay
within `0x4000_0000`–`0x7FFF_FFFF`.** An address at or above `0x8000_0000`
is routed toward GP1 instead — a port this design never connects to
anything — so the CPU's access simply never reaches any PL logic at all,
and the core stalls forever waiting for a response that can never come.
This is invisible to RTL simulation, synthesis, and timing analysis, since
all of those only model *from* `M_AXI_GP0` onward — the failure happens
entirely on the PS side, before that point. This bit us for real once:
`axi_spi` was originally given base `0x8000_0000` under the old
`axi_interconnect` scheme, which hung every single access on real hardware
while passing every RTL-side check cleanly.

**Peripheral addressing today: `PERIPH_ID` at `addr[23:16]`, not
`addr[31:24]`.** Every real GP0 transaction necessarily has `addr[31:24]`
somewhere in `0x40`–`0x7F` (since that's GP0's whole window — the top two
bits are fixed at `01`). The first version of this scheme compared
`PERIPH_ID` against the full top byte (`addr[31:24]`) using small logical
values (`0x01`, `0x02`, `0x03`) — values that can *never* appear in
`addr[31:24]` on real hardware, meaning every peripheral would have DECERR'd
on every access, forever, regardless of software. This was caught in
simulation review before ever touching hardware (every testbench
constructs `m_axi_gp0_*` directly, bypassing the real PS7, which is exactly
why it wasn't caught earlier — those addresses were never something a real
GP0 transaction could produce). Fixed by moving the compared field down to
`addr[23:16]`, which `axi_if` never inspects at `addr[31:24]` at all — so
that byte is free to be whatever GP0 fixes it to, and each peripheral still
gets a full 16 bits (`addr[15:0]`) of its own internal address space,
comfortably more than any of the three currently need. `sw/main.c`'s
`AXI_BASE`/`SPI_AXI_BASE` are `0x40000000 | (PERIPH_ID << 16)`.

Current allocation, fanned out identically to every peripheral by `axi_if`
(`src/axi_if.sv`) with no interconnect/range table at all:

| PERIPH_ID | Peripheral | Notes |
|---|---|---|
| `0x01` | `axi_registers` | 8 word-aligned 32-bit registers; `AXI_BASE` in `sw/main.c` |
| `0x02` | `axi_spi` | Direct single-transaction AD9361 SPI bridge; `SPI_AXI_BASE` in `sw/main.c` |
| `0x03` | `axi_cdc_status` | 64 x 8-bit status lanes, read-only from AXI, native to `dsp_clk`; UART dispatch via `CMD_DEV_CDC` (`0x0C`) |

### The rest of the PS address map (for whichever peripheral we need next)

Table 4-1's full picture, beyond just GP0/GP1 — most future needs (beyond
the AXI/UART already in use) will land in one of these:

| Address range | Region |
|---|---|
| `0x0000_0000`–`0x3FFF_FFFF` | DDR (and OCM aliasing at the low end — see UG585 §4.1's notes for the full story) |
| `0x4000_0000`–`0x7FFF_FFFF` | `M_AXI_GP0` (PL) — this project, see above |
| `0x8000_0000`–`0xBFFF_FFFF` | `M_AXI_GP1` (PL) — not wired up in this design |
| `0xE000_0000`–`0xE02F_FFFF` | I/O Peripherals (IOP) — hard peripheral registers, see table below |
| `0xE100_0000`–`0xE5FF_FFFF` | Static Memory Controller (SMC) memories |
| `0xF800_0000`–`0xF800_0BFF` | SLCR (System Level Control Registers) |
| `0xF800_1000`–`0xF880_FFFF` | Misc PS system registers — timers, DMAC, watchdog, DDR controller, AXI_HP ports, OCM control, CoreSight (see table below) |
| `0xF890_0000`–`0xF8F0_2FFF` | CPU private registers (SCU, GIC, etc.) |
| `0xFC00_0000`–`0xFDFF_FFFF` | Quad-SPI linear address space |

**I/O Peripheral (IOP) register map** (Table 4-6, 32-bit APB bus) — UART1
(already in use) is the second entry here. Note PS7's hard SPI0/SPI1
controllers exist at `0xE000_6000`/`0xE000_7000` but are **disabled** in
this project's PS7 config (`PCW_SPI0_PERIPHERAL_ENABLE`/
`PCW_SPI1_PERIPHERAL_ENABLE` both `0`) — that's the whole reason `axi_spi`
had to be built as a custom PL peripheral instead of just using a hard SPI
block: the AD9361's SPI pins land on plain PL bank I/O, not MIO-routed to
either hard controller on this board.

| Base address | Peripheral |
|---|---|
| `0xE000_0000` | UART Controller 0 |
| `0xE000_1000` | UART Controller 1 — in use, `sw/main.c` |
| `0xE000_2000` | USB Controller 0 |
| `0xE000_3000` | USB Controller 1 |
| `0xE000_4000` | I2C Controller 0 |
| `0xE000_5000` | I2C Controller 1 |
| `0xE000_6000` | SPI Controller 0 (hard peripheral — disabled in this project) |
| `0xE000_7000` | SPI Controller 1 (hard peripheral — disabled in this project) |
| `0xE000_8000` | CAN Controller 0 |
| `0xE000_9000` | CAN Controller 1 |
| `0xE000_A000` | GPIO Controller |
| `0xE000_B000` | Ethernet Controller 0 |
| `0xE000_C000` | Ethernet Controller 1 |
| `0xE000_D000` | Quad-SPI Controller |
| `0xE000_E000` | Static Memory Controller (SMC) |
| `0xE010_0000` | SDIO Controller 0 |
| `0xE010_1000` | SDIO Controller 1 |

**Misc PS system registers** (Table 4-7, 32-bit AHB bus) — the two most
likely to matter here later: the triple timer counters (a real,
clock-derived timer, vs. `sw/main.c`'s current uncalibrated NOP-loop
`delay()`), and the AXI_HP ports (high-performance PL→PS/DDR *slave*
interfaces — the opposite direction from GP0/GP1, meant for bulk DMA-style
transfers rather than register pokes):

| Base address | Peripheral |
|---|---|
| `0xF800_1000` | Triple Timer Counter 0 (TTC0) |
| `0xF800_2000` | Triple Timer Counter 1 (TTC1) |
| `0xF800_3000` | DMA Controller (secure) |
| `0xF800_4000` | DMA Controller (non-secure) |
| `0xF800_5000` | System Watchdog Timer (SWDT) |
| `0xF800_6000` | DDR Memory Controller |
| `0xF800_7000` | Device Configuration Interface (DevC) |
| `0xF800_8000`–`0xF800_B000` | AXI_HP 0–3 (high-performance AXI slave ports, PL→PS) |
| `0xF800_C000` | On-Chip Memory (OCM) control |
| `0xF880_0000` | CoreSight debug control |

Source: [UG585 Zynq-7000 SoC Technical Reference Manual — Address Map](https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/Address-Map) (Tables 4-1, 4-6, 4-7)

## PS7 UART1 (hard peripheral, not PL/AXI)

Unlike the AXI peripherals, this doesn't go through `M_AXI_GP0` or the PL
at all — UART1 is one of PS7's own hard peripherals, sitting on the PS's
internal bus at a fixed address, and the ARM core talks to it directly.

**No `.bd`/`.xci`/RTL changes were needed to enable it.** The vendor's
original PS7 design (inherited into `src/PS/bd/design_1/design_1.bd`
before this project ever touched it) already has `PCW_UART1_PERIPHERAL_ENABLE=1`
routed through **MIO 8 (TX) / MIO 9 (RX)**, confirmed against this board's
actual schematic. Because it's MIO-routed rather than EMIO-routed, the
signal is handled entirely inside PS7 silicon and reaches the physical
package pins through `FIXED_IO_mio[53:0]` — a bus `fm_receiver.sv` already
declares and wires straight through to `design_1_wrapper`, so there was no
new top-level port to add (`design_1_wrapper.v`'s port list has zero
UART-related signals — confirmed by grepping the generated wrapper).

PS7's own generated `ps7_init.tcl` (run by `XSDB: Init PS7 over JTAG`)
already enables UART1's AMBA clock (`APER_CLK_CTRL`) and its reference
clock (`UART_CLK_CTRL`, IO PLL ÷10 → 100 MHz, matching
`PCW_UART_PERIPHERAL_FREQMHZ`), and even pre-loads `CR`/`MR`/`BAUDGEN`/
`BAUDDIV` for 115200 baud — before `main.c`'s own `uart1_init()` ever runs.
`main.c`'s init is redundant with that but harmless (same target state),
and keeps the software self-contained/explicit rather than relying on
init-script behavior that isn't obvious from reading `main.c` alone.

Register offsets/bit values in `sw/main.c` (`UART_CR`, `UART_MR`,
`UART_BAUDGEN`, `UART_SR`, `UART_FIFO`, `UART_BAUDDIV`, base `0xE0001000`)
were taken directly from the vendor's Xilinx BSP header
(`docs/.../libsrc/uartps_v3_11/src/xuartps_hw.h`), not hand-derived from
the TRM — safer for getting bit positions right on the first try.

**Physical connection, board-specific:** this HamGeek board does **not**
share UART with the JTAG USB connection — UART1 has its own separate USB
port with a dedicated bridge chip. If you're looking for a serial device
and only see the JTAG-related one, you're on the wrong port; plug into the
second USB connector and a second COM port should enumerate. Terminal
settings: 115200 8N1, no flow control.

### UART register command console

`main.c` runs a blocking interactive register console
(`process_uart_command()`, called forever from `main()`). Each command is
8 raw bytes (not ASCII hex text), MSB-first:

| Byte(s) | Meaning |
|---|---|
| 0 | Device: `0x00`=`axi_registers` (direct AXI passthrough at `AXI_BASE + addr`), `0x08`=`axi_spi` (direct SPI passthrough at `SPI_AXI_BASE + AD9361 addr`), `0x04`=system (mode select), `0x0C`=`axi_cdc_status` (direct AXI passthrough at `CDC_AXI_BASE + addr`, read-only) |
| 1 | `0x00`=read, `0x01`=write (ignored for the `0x0C` CDC device — the peripheral itself is read-only from AXI) |
| 2–3 | 16-bit address, big-endian — a byte offset from the relevant base for `axi_registers`/`axi_cdc_status`, or the AD9361 register address (bits[9:0]) for `axi_spi` |
| 4–7 | 32-bit data, big-endian (write value; ignored on read; for `axi_spi` only the low byte is used) |

On a read, the result comes back as 4 raw bytes, big-endian. For `axi_cdc_status`
reads this is the addressed 8-bit lane, zero-extended (one lane per
word-aligned offset — see "AXI peripherals" above for the layout).

Example: reading AD9361's `REG_PRODUCT_ID` (`0x037`) returns a byte with
`(reply_byte & 0xF8) == 0x08` — confirmed on hardware (`0x0A`: product ID
`0x08` plus silicon revision 2): `08 00 00 37 00 00 00 00`.

**Verified on real hardware**: all four devices — `axi_registers` and
`axi_spi` reads/writes (including `REG_PRODUCT_ID` and a write/readback
round-trip), `axi_cdc_status` reads (`valid_count`/`error_count`/
`rx_frame_s` confirmed live and matching JTAG-read values), and both
`SYS_MODE_TEST`/`SYS_MODE_MISSION`. `bring_up_uart.txt` holds a
hand-verified reference set of commands for all of this — mode entry,
polling the RX diagnostics, setting the coarse RX delay, and chip-status
reads. An intermittent hang affecting the whole console showed up during
early bring-up — see "Intermittent hang investigation" below; long
resolved, unrelated to any of the above.

## Bare-metal software (`sw/`)

No FSBL, no BSP, no OS — `xsdb` sets the ARM core's program counter
directly to `_start` over JTAG, so there's no boot ROM / reset-vector
dependency to satisfy.

- **`startup.S`** — sets a known CPU mode (Supervisor, IRQ/FIQ masked),
  installs an 8-entry exception vector table via `VBAR` (cheap insurance:
  without it, an unexpected exception — e.g. an unaligned access — falls
  through to whatever garbage sits at the default `0x0` vector base, which
  is unpleasant to debug over JTAG), enables the MMU with a flat
  identity-mapped table and the instruction cache, sets up the stack, and
  calls `main()`. See "Intermittent hang investigation" below for why the
  MMU/cache configuration looks the way it does — it's a mitigation for a
  real bug, not a performance choice.
- **`linker.ld`** — places `.text`/`.rodata`/`.data`/`.bss` in DDR starting
  at `0x00100000` (1 MB in — clear of address 0, and DDR is only usable
  after PS7 JTAG init has brought up the DDR controller). Reserves a 64 KB
  stack above `.bss`, exposing `__stack_top`. `.bss` zeroing isn't
  implemented yet since the program has no globals — needed the first time
  one is added.
- **`main.c`** — raw pointer MMIO access, no libc. Initializes UART1, runs
  `ad9361_common_init()` once at boot (pin release, BBPLL, LVDS parallel
  port, RX LO synth, coarse RX delay, ALERT→RX transition — see "AD9361 RX
  digital bring-up" above), then runs the register command console forever
  (see UART section above), dispatching each 8-byte command to
  `axi_registers`, `axi_spi`, `axi_cdc_status`, or the SYS mode-select
  device. `main()` itself is now just `uart1_init()` + `ad9361_common_init()`
  + the command loop — the original boot-time LED blink demo and the
  bisection-sentinel UART bytes used while chasing the intermittent hang
  (see below) have both been removed now that they've served their purpose.
- **`build.bat`** — `arm-none-eabi-gcc -c` for each source file
  (`-mcpu=cortex-a9 -marm -ffreestanding`), then links with raw
  `arm-none-eabi-ld -T linker.ld` (bypassing the `gcc` driver deliberately,
  so nothing libc-related gets pulled in), then `objcopy -O binary` for a
  `.bin` byproduct (not used by the actual JTAG deploy flow, which loads
  the `.elf` directly).

## JTAG bring-up via `xsdb`

`ps7_jtag_init.tcl` and `load_run_app.tcl` both run via `xsdb`, not
`vivado -mode batch` — a separate tool for talking to the ARM cores' debug
access ports over the same JTAG chain Vivado's hardware manager uses for
the PL bitstream.

### Gotchas (the whole "no blinky LED" saga lived here, not in any RTL)

- **`targets`'s tabular output doesn't survive naive log redirection**
  (`xsdb foo.tcl > log.txt 2>&1`). Both scripts use `puts [targets]`
  instead of a bare `targets` call specifically so the target list is
  actually visible in the log — this was the single most useful debugging
  change made during this bring-up, since every other bug below was
  invisible until this was in place.
- **Target filter matching the wrong node.** `targets -set -filter {name
  =~ "APU*"}` matches the *parent* `"APU"` group node, not either real CPU
  core. A `dow` (download) against the group node fails with `Invalid
  context` — it needs one concrete core. Fixed filter, used in both
  scripts: `{name =~ "ARM Cortex-A9 MPCore #0*"}`.
- **`rst -processor` before `dow`** used to sit in `load_run_app.tcl` and
  threw `Invalid reset type`, silently aborting the script before `dow`/
  `con` ever ran — this was the actual root cause behind many "ran
  everything, still no blinky" attempts. Removed; a plain `stop` (halt the
  currently selected target) covers what was actually needed, wrapped in
  `catch { stop }` since the core often shows up already halted once the
  target filter is correct, and `stop` on an already-halted target errors
  with `Already stopped` instead of being a no-op.
- **A `DAP (APB AP transaction error, ...)` entry in the target list,
  with the ARM cores missing from it entirely** (not just mis-selected) is
  a wedged JTAG debug session, not hardware damage and not a script bug.
  The only fix found: a full power cycle of the board. Not yet automated
  as an explicit loud-failure check in either script (see TODOs).
- **PS7 clock/DDR state persists across PL-only reprogramming** within the
  same power-on session. `Program FPGA` alone does not undo a previous
  `PS7 JTAG Init` — only a real power cycle resets that state.
- **`output/vivado/ps7_gen` isn't just a synthesis cache — `ps7_jtag_init.tcl`
  depends on a file inside it too (`ps7_init.tcl`), independent of
  synthesis.** `ps7_configure.tcl` clears that whole directory on every
  successful save (see above), including a no-op re-save that changed
  nothing. If you skip re-synthesizing afterward because "the PL design
  didn't actually change" (a perfectly reasonable call for a config no-op,
  or even a real config change with no RTL impact), `ps7_init.tcl` stays
  missing until something regenerates it — and `XSDB: Init PS7 over JTAG`
  then fails with `couldn't read file ".../ps7_init.tcl": no such file or
  directory`, which cascades into `XSDB: Load & Run` failing with `Memory
  write error ... Cannot access DDR: the controller is held in reset` (DDR
  was never actually brought up). Both failures look unrelated to the real
  cause at first glance. Fix: run `Vivado: Synthesize` at least once after
  any `ps7_configure.tcl` run, even if you don't need a new bitstream —
  it's the only thing that regenerates `ps7_gen`, and doing so doesn't
  require following through to `Implement`/`Bitstream`/`Program FPGA` if
  the bitstream itself is genuinely unchanged.

## Intermittent hang investigation

**Symptom:** after some period of use — sometimes idle, sometimes under
active traffic — the UART console stopped responding to any command. The
PL-driven LED kept blinking throughout (rules out a dead PL clock domain).
Only a full software reload fixed it — no bitstream reload or power cycle
needed.

**Ruled out, in order:**
- UART clock drift/framing (the original theory): baud generator error
  is ~64ppm, too small to explain byte-level slips on a hard UART
  peripheral that resyncs every start bit; the hang also showed up after
  idle periods with no traffic to have drifted.
- Generic DDR/PS7 hardware corruption: `scripts/ddr_soak_test.tcl`
  hammered a DDR region unrelated to the app, over JTAG, continuously —
  including through an actual live hang — and stayed clean through 300+
  passes.
- ARM Cortex-A9 erratum 794073 (speculative fetches with the MMU
  disabled + branch prediction active) alone: applied the documented
  workaround (`SCTLR.Z` cleared), confirmed active via direct register
  readback (`sctlr = 0x08c50078`), hang still reproduced afterward.
- Errata 742230/743622 (from the vendor's own FSBL BSP,
  `docs/.../zynq_fsbl/.../boot.S`): both gated to silicon revision
  r2p2/r2p* in the workaround code itself; confirmed via direct `MIDR`
  readback (`0x413fc090` = r3p0) that this chip doesn't need them.
- MMU-disabled as a standalone cause: enabled the MMU with a flat,
  Strongly-Ordered identity map (behaviorally identical to MMU-off,
  isolating just the `M` bit) — hang still reproduced, same fault, same
  exact instruction.

**Diagnostic halts** (procedure in `xsdb_instructions.md`) caught three
different exception types across separate reproductions — a Prefetch
Abort, an Undefined Instruction, and (twice) a Data Abort — always on a
plain `movw`/`movt`-then-dependent-load sequence computing a UART
peripheral register's address: the single most frequently re-executed
instruction sequence in the program, running on every byte of every
command. The clearest capture showed the faulted value differing from
correct by exactly one bit, in the half-word a `movt` instruction had
just written — consistent with the *fetched instruction encoding
itself* being wrong on that particular DDR read, not a computation
error.

**Fix:** enable the instruction cache (`SCTLR.I`) for the DDR/code
region. This requires the MMU to be on, since cacheability is a
page-table attribute — `sw/startup.S` builds a minimal flat table for
this: DDR marked Normal/Cacheable, everything else Strongly Ordered, so
peripheral access behavior is unchanged. The hot instruction sequence
now gets fetched from DDR once and served from L1 cache afterward,
instead of re-fetched on every execution. Verified clean for 2+ hours of
continuous use (including active SPI/UART traffic), vastly exceeding
every prior configuration's failure window (all well under 10 minutes).

**Known unknowns:** this identifies *where* the corruption was being
exposed (repeated DDR instruction fetch of a hot loop) and a working
mitigation, not the exact underlying defect. This board's Zynq is a
reclaimed/used part with its package markings and QR code deliberately
sanded off — genuine Xilinx silicon (confirmed via JTAG device ID) but
of unconfirmable, plausibly lower-than-assumed speed grade, running
without a heatsink. A timing margin issue consistent with heat and/or a
slower silicon bin is the leading explanation, not a confirmed one.

## AD9361 RX digital bring-up

Everything below is downstream of SPI already working (see "Current
state" above) — this covers getting the chip past SPI comms and into a
state where its RX digital interface (LVDS) actually produces data. This
whole section predates the AXI refactor above and is unaffected by it —
`ad3961_if_rx.sv` and the AD9361 SPI register sequences didn't change.

### RTL: `ad3961_if_rx.sv` wired into `fm_receiver.sv`

RX-only (TX deliberately out of scope for now). Top-level ports added:
`rx_clk_in_p/n`, `rx_frame_in_p/n`, `rx_data_in_p/n[5:0]` (pins fixed in
`constraints.xdc`). `ad3961_if_rx` recovers its own clock from the LVDS
pins (`dsp_clk`, via `IBUFDS`+`BUFGCE`) and decodes I/Q samples using
`IDDR` + a 2-cycle frame-pattern match. `adc_r1_mode` (single-RF vs
dual-RF decode path, driven from `regmap[67]`) crosses from `fclk0` into
`dsp_clk` through a 2FF synchronizer — a genuine CDC, since `dsp_clk` is
an independently recovered clock, not derived from `fclk0`. This is the
same `dsp_clk` domain `axi_cdc_status` (see "AXI peripherals" above) now
lives in, though `dsp_rstb` isn't actually wired to a real reset yet —
tied to `1'b1` at `axi_cdc_status`'s instantiation, matching how the rest
of this `dsp_clk`-domain logic (`dsp_counter`, `dsp_rx_error_count`, this
sync chain) has no reset at all today.

### AD9361 register bring-up — verified against ADI's real driver, not guessed

Every value below was traced through ADI's reference driver
(`docs/AD9361_DOCS/.../AD936X_PS/AD936X/ad9361/ad9361.c`/`.h`) —
`ad9361_setup()`, `ad9361_bbpll_set_rate()`, `ad9361_set_trx_clock_chain()`,
`ad9361_ensm_set_state()` — register by register, not copied from a
generic example or guessed from bit names. Two real bugs were caught this
way before they cost hardware debugging time:

- **`REG_REF_DIVIDE_CONFIG_1` POR-default bit.** ADI's header marks bit 2
  `REF_DIVIDE_CONFIG_1_DFLT` with comment `/* Set to 1 */` — the real
  driver never writes it explicitly because it's already 1 at power-on
  reset and only ever touches this register via a single-bit
  read-modify-write (`RX_REF_RESET_BAR`). A full-byte UART write
  (`0x02`) silently cleared that bit, which blocked `BBPLL_LOCK` from
  ever asserting no matter how many times VCO calibration was retried.
  Fixed value: `0x06` (`REF_DIVIDE_CONFIG_1_DFLT | RX_REF_RESET_BAR`).
- **External reference clock, not onboard XTAL.** `sw/main.c`'s reference
  `default_init_param` table sets `xo_disable_use_ext_refclk_enable = 1`
  (→ `pdata->use_extclk = true` in `ad9361_api.c`), so `REG_CLOCK_ENABLE`
  needs `XO_BYPASS` set (`0x17`, not the "default" `0x07`) — otherwise
  the same POR-default class of bug.

Bring-up sequence, in order (see `ad9361_common_init()` in `sw/main.c`
for the always-on part, `bring_up_uart.txt` for the still-manual part):

1. **Pin release** — `enable`/`resetb`/`adc_r1_mode` via `axi_registers`'
   regmap word 2 (`0x0D`).
2. **Analog/reference bring-up** — `REG_CTRL`, bandgap trim ×2, ref
   divider ×2, `REG_CLOCK_ENABLE`.
3. **BBPLL programming** — `ad9361_bbpll_set_rate()`'s full sequence
   (CP current, loop filter ×3, VCO control/cal-count, SDM control,
   integer/fractional frequency word, cal-start pulse, VCO program
   registers). Target 960 MHz from the board's 40 MHz refclk — chosen
   because 960/40 = 24 exactly, no fractional word needed. Verified via
   `REG_CH_1_OVERFLOW` bit 7 (`BBPLL_LOCK`) reading back `0x80`.
4. **RX clock-divider chain** — `REG_BBPLL`'s ADC divider +
   `REG_RX_ENABLE_FILTER_CTRL`'s three half-band stages, RX FIR bypassed
   (`RX_SAMPL_FREQ = CLKRF_FREQ`, sidesteps needing a FIR coefficient
   table). Two known-good options in `bring_up_uart.txt`: BBPLL/64 →
   15 MHz, or BBPLL/32 → 30 MHz. **Confirmed against real hardware**: a
   `dsp_clk`-domain counter driving an LED gave measured periods of 17s
   and 9s respectively, which back out (`2^28 / period`) to ~15.8 MHz and
   ~29.8 MHz — matching the two configured targets almost exactly. This
   also settled that `dsp_clk` (the recovered LVDS clock) equals
   `RX_SAMPL_FREQ` directly, 1:1, not some multiple of it.
5. **LVDS parallel port** — `REG_PARALLEL_PORT_CONF_1/2/3` (IQ swap,
   pulse-mode frame, 1R1T timing, LVDS mode).
6. **Mode-specific ENSM/test config** — see below.

`constraints.xdc`'s `create_clock -period 4.00 [get_ports rx_clk_in_p]`
(250 MHz) predates all of this and is now confirmed wrong — real rate is
an order of magnitude lower. Not a timing risk (STA was pessimistic, not
optimistic), but worth correcting once a final sample rate is settled
rather than left as a stale guess.

### ENSM state machine: ALERT → real RX, once the RX LO synth is locked

The AD9361's ENSM (Enable State Machine) moves `SLEEP_WAIT` → `ALERT` →
`RX` (or `FDD`). After the bring-up sequence above, `REG_STATE` (`0x017`)
reads `0x05` (ALERT) — correct, expected, not a bug: exactly where
`ad9361_ensm_set_state()` leaves the chip after enabling clocks from
`SLEEP`.

Getting to `RX` (`0x08`) needs a completely separate PLL — the RX local
oscillator (RF synthesizer) that downconverts the antenna signal, distinct
from the BBPLL. `ad9361_ensm_set_state()` explicitly polls that synth's
lock bit (`REG_RX_CP_OVERRANGE_VCO_LOCK` bit 1, `VCO_LOCK`) before allowing
the ALERT→RX transition — confirmed directly by reading that bit as `0`
before this work, at which point `REG_ENSM_CONFIG_1 = 0x49` (`LEVEL_MODE |
TO_ALERT | FORCE_RX_ON`) just left `REG_STATE` stuck at `0x05` no matter
what. **This was also the real explanation for test mode never producing
real streaming data** (see below): `ENABLE_RX_DATA_PORT_FOR_CAL` (what
`enter_test_mode()` uses) is only ever combined with a real `FORCE_RX_ON`
transition through a dead (`#if 0`'d) code path in ADI's reference driver —
it was never a substitute for a locked RX synth, contrary to the
assumption this bring-up started from.

**The RX LO synth (98 MHz target) has now been derived and verified
working on real hardware**, register-by-register from `ad9361.c`
(`ad9361_txrx_synth_cp_calib()`, `ad9361_rfpll_vco_init()`,
`ad9361_calc_rfpll_int_divder()`, `ad9361_rfpll_int_set_rate()`), same
rigor as the BBPLL derivation above — not copied from a generic example.

- **Reference into the synth is 40MHz passthrough, not doubled** —
  decoded directly from the already-written `REG_REF_DIVIDE_CONFIG_1/2`
  bits (`RX_REF_DIVIDER_MSB`/`_LSB` both read `0`); no change needed to
  those two registers.
- **Divider math for 98 MHz** (`ad9361_calc_rfpll_int_divder`'s algorithm:
  double the target until it clears the 6GHz VCO floor, then compute an
  integer/fractional-N ratio against the reference): `vco_div = 5`
  (98MHz × 2⁶ = 6.272GHz), `integer = 156` (`0x9C`), `fract = 6710874`
  (`0x66665A`, comfortably under `RFPLL_MODULUS = 8388593`).
- **VCO band programming**: `ad9361_rfvco_tableindex(40MHz)` selects
  `LUT_FTDD_40`; the row where `VCO_MHz` first drops ≤ 6272 is
  `{6270,7,2,7,3,15,13,56,12,15,12,4,13}` (`ad9361.c:280`) — its bias/
  varactor/charge-pump/loop-filter fields were copied verbatim into their
  ~10 registers, not recomputed (this is ADI's own characterization data).
- **RX+TX synth charge-pump calibration** runs first (`REG_RX_CP_LEVEL_DETECT`
  etc., ~8 registers each side), and — a real gotcha worth remembering if
  this is ever re-derived — that calibration step *temporarily* forces
  `REG_ENSM_MODE = FDD_MODE` and `REG_ENSM_CONFIG_1 = FORCE_ALERT_STATE|TO_ALERT`
  regardless of the chip's real operating mode; `REG_ENSM_MODE` must be
  explicitly restored to `0x00` (TDD) afterward, since our own
  `ad9361_common_init()` never wrote it at all before this and the chip's
  power-on default there is `0x01` (FDD).

**Hardware result**: VCO lock (`REG_RX_CP_OVERRANGE_VCO_LOCK` bit 1) set
on the very first attempt. `REG_STATE` moved `0x05`→`0x08` — genuine RX
state, not test mode. With BIST/PRBS armed on top, `rx_data` changed on
every single sample across 20+ consecutive polls (compare: every prior
test this session showed it bit-for-bit frozen). Full write sequence in
`scripts/_rx_lo_synth_98mhz.tcl` (JTAG/SPI hand-verification only, no
bitstream/firmware changes).

**Coarse chip-side delay tuning, revisited.** Earlier in this bring-up a
sweep of `REG_RX_CLOCK_DATA_DELAY` (AD9361 reg `0x006`, `DATA_CLK_DELAY`
in bits`[7:4]`, `RX_DATA_DELAY` in bits`[3:0]`) had shown *zero* effect —
unsurprising in hindsight, since the interface had no real signal to align
yet. Re-run against the now-live interface, it found a sharp, real
transition: sweeping `RX_DATA_DELAY` (clock delay held at 0) from `0x00`
to `0x07` left `valid_count` completely flat and `err` free-running
continuously; from `0x08` to `0x0F` the frame pattern settled and `err`
stopped accumulating entirely while `valid_count` started incrementing
fast enough to wrap its 8-bit counter within a 600ms poll window. Sweeping
`DATA_CLK_DELAY` instead (data delay held at 0) showed no equivalent
transition anywhere across all 16 settings — the fix lives entirely in the
data/frame group delay, not the forwarded clock delay. Settled on
`RX_DATA_DELAY = 0x0B` (middle of the working `0x08–0x0F` band, mirroring
how ADI's own `ad9361_find_opt()` centers in the widest contiguous pass
run rather than sitting at an edge). Sweep script:
`scripts/_clkdata_delay_sweep_live.tcl`.

**Now permanent**: both the RX LO synth sequence and `RX_DATA_DELAY=0x0B`
are in `sw/main.c`'s `ad9361_common_init()`, run unconditionally at boot —
confirmed on hardware to reproduce the identical signature above
(`REG_STATE=0x08` immediately after boot, before any UART command;
`rx_data` changing every sample once `enter_test_mode()`'s BIST arm
follows). `ENABLE_RX_DATA_PORT_FOR_CAL` was dropped entirely from
`enter_test_mode()` rather than kept as dead weight, since it never
actually worked. `enter_mission_mode()` now just clears any leftover
`REG_BIST_CONFIG` — real RF reception needs nothing else once common init
has done its job. Mission mode itself (real antenna signal, not BIST) is
still unverified on hardware — no RF source confirmed ready yet.

**Test mode's promise didn't hold up.** The original theory was that
`ENABLE_RX_DATA_PORT_FOR_CAL` (`REG_ENSM_CONFIG_1 = 0x80`) forces the RX
digital data port active straight from ALERT, letting BIST/PRBS test data
flow without a real RX-state transition or the RF LO being locked. In
practice this left the interface producing a brief burst of noise at
bring-up and then bit-for-bit frozen data forever after — across every
variation tried (clearing the cal bit, re-arming BIST, a full hard reset
and from-scratch reinit). Digging into ADI's real driver explained why:
`ENABLE_RX_DATA_PORT_FOR_CAL` is only ever asserted through a dead
(`#if 0`'d) TX-monitor calibration path — it was never a working
substitute for a real, synth-locked RX state. The RX LO synth work above
is what actually fixed it; test mode remains useful now as a known-pattern
signal source once the chip is in genuine RX state, not as a way to avoid
needing the synth.

### Software: common bring-up automated, mode selection stays manual

`sw/main.c` splits into three tiers, split by what's shared vs.
mode-specific vs. still being actively tuned:

- **`ad9361_common_init()`** — pin release, BBPLL, LVDS parallel port.
  Runs automatically at boot, so a fresh FPGA upload + software reload
  doesn't need any of this replayed by hand over UART.
- **`enter_test_mode()` / `enter_mission_mode()`** — dispatched via the
  `CMD_DEV_SYS` UART command (`04 01 00 00 00 00 00 01`=test,
  `...02`=mission). `enter_test_mode()` sends the verified
  `REG_ENSM_CONFIG_1`/`REG_OBSERVE_CONFIG`/`REG_BIST_CONFIG` sequence.
  `enter_mission_mode()` no longer needs to do the RX LO synth bring-up
  itself — that moved into `ad9361_common_init()` since it turned out to
  be common to both modes (see "AD9361 RX digital bring-up" above). Both
  mode functions now hold only their genuinely mode-specific step: arm
  BIST, or clear it.
- **RX clock-divider chain** — deliberately *not* automated, even though
  it's common to both modes. It's still an actively-tuned knob (two known
  configs, more likely coming as the real sample-rate needs get decided);
  baking a specific rate into `ad9361_common_init()` would mean a full
  rebuild+reupload cycle every time it changes instead of one UART
  command. Stays in `bring_up_uart.txt`.

`bring_up_uart.txt` is the manual-command reference/scratch file — kept
lean on purpose after the register derivation above was folded into
`sw/main.c`. It currently holds: the blinky board-alive check, both
clock-divider-chain options, the raw test-mode-completion writes (same
effect as the `04 01...` shortcut, spelled out for tuning without a
rebuild), status reads, and the not-yet-working mission-mode ALERT→RX
write (commented out). Its UART wire protocol is unaffected by the AXI
refactor above — only `sw/main.c`'s internal address translation changed,
so this file needed no updates.

### The "mangled register 0x08" mystery — a tooling artifact, not RTL

During this bring-up, reading back regmap word `0x08` after writing the
value `0x0D` appeared to return a mangled/truncated result, while every
other value (including other values with the same bits set) read back
fine. The bit-pattern-independent, single-exact-value-correlated
signature (`0x0D` = ASCII Carriage Return) was the tell: this was a VS
Code serial monitor extension doing line/text-oriented processing on raw
binary UART data, not a hardware or RTL bug. No RTL changes were needed
to fix it. Worth remembering if a similarly "impossible" single-byte
readback anomaly shows up again — check the host-side tooling before
suspecting the hardware.

### Sample counters: peripheral built, CDC strategy decided, real producer still pending

LED-based debugging (`adc_valid`, a `dsp_clk`-domain counter wired straight
to an LED) was useful for the clock-presence/rate checks above, but has
real limits: a raw ~15-30MHz toggle just reads as a dim glow, not readable
blinking, and it doesn't distinguish "clock present but no valid frames"
from "genuinely locked and streaming." LEDs are also planned to go away
entirely once a heatsink is added.

The replacement — free-running valid-sample and error counters read back
through the regmap instead of an LED — now has its peripheral built and
extended (`axi_cdc_status`, 64 lanes, see "AXI peripherals" above) and its
CDC question answered: a **toggle-synchronized request/ack handshake**, not
gray-coding, crosses AXI reads from `fclk0` into `dsp_clk` and back.
`fm_receiver.sv` currently drives it with a diagnostic mirror
(`cdc_mirror_seq`, reg0-9) rather than the real counter logic — built to
debug the RX interface bring-up (see "AD9361 RX digital bring-up" above),
not the originally-intended sample valid/error counters. Writing that real
producer is still the next piece of RTL to build here, now unblocked since
real aligned RX data finally flows.

## Current status / TODO

- [x] PS7 copied into `src/PS/`, IP upgraded to the installed Vivado revision
- [x] `FCLK0` enabled at 100 MHz, exposed externally as `fclk0`/`fclk0_rstn`
- [x] `M_AXI_GP0` enabled and exposed as a raw AXI3 interface
- [x] Bare-metal Cortex-A9 toolchain, startup code, linker script, test app
- [x] Full chain verified on real hardware: software AXI writes visibly
      change LED blink behavior
- [x] `ps7_configure.tcl` made idempotent (safe to re-run any time)
- [x] PS7 UART1 enabled (inherited from vendor design, no `.bd`/`.xci`
      changes needed) and verified end-to-end on real hardware, independent
      of the AXI/PL path
- [x] UART register command console built (`process_uart_command()`),
      replacing the earlier one-way sequential-hex bring-up test —
      verified on hardware (pre-refactor addressing)
- [x] AD9361 confirmed responding correctly over SPI — `gpio_resetb` was
      the missing piece (floating, holding the chip in reset); wired to
      the regmap alongside `enable`/`txnrx`. `REG_PRODUCT_ID` and a
      write/readback round-trip both verified on real hardware
- [x] **Intermittent full UART console hang** — mitigated via MMU +
      instruction cache configuration in `sw/startup.S`; verified clean
      for 2+ hours of continuous use. See "Intermittent hang
      investigation" above for the full story, including what's still
      not fully explained.
- [x] `ad3961_if_rx.sv` wired into `fm_receiver.sv` (RX only), top-level
      LVDS pins added, `adc_r1_mode` CDC synchronizer in place
- [x] AD9361 BBPLL locked (960MHz from 40MHz refclk) — verified via
      `REG_CH_1_OVERFLOW`/`BBPLL_LOCK` reading back set
- [x] RX clock-divider chain configured and confirmed on real hardware
      (recovered LVDS clock measured matching the configured rate)
- [x] Test mode (BIST/PRBS into the RX digital pipeline) reachable from
      ALERT, no RF front end required — `sw/main.c`'s `enter_test_mode()`
- [x] Common AD9361 bring-up automated in `sw/main.c`
      (`ad9361_common_init()`, runs at boot); mode selection and the RX
      clock-divider chain stay manual (`bring_up_uart.txt`) by design
- [x] AXI architecture rewritten: `axi_fm`/`axi_interconnect` replaced by
      `axi_if` + `axi_registers`/`axi_spi`/`axi_cdc_status`, each
      self-selecting via `PERIPH_ID`; four testbenches (`tb_axi_fm`,
      `tb_axi_spi`, `tb_axi_cdc_status`, `tb_fm_receiver`) all passing
- [x] `axi_cdc_status` built and extended: 64 x 8-bit read-only status
      regmap, native to `dsp_clk`, toggle-synchronized CDC handshake for
      AXI reads (extended from 16 lanes to leave room for the user's
      correlator/histogram DSP modules)
- [x] `sw/main.c` reworked for the new `PERIPH_ID` addressing and the
      direct single-transaction `axi_spi` model
- [x] Real bug caught before hardware: `PERIPH_ID` originally compared
      against `addr[31:24]`, unreachable from real `M_AXI_GP0` traffic —
      moved to `addr[23:16]`; RTL, all four testbenches, `sw/main.c`, and
      this doc all updated together
- [x] Full Vivado flow (synthesis → implementation → bitstream) verified
      clean on the new architecture — 0 errors, all timing constraints
      met, 0 DRC violations
- [x] Re-verified on real hardware since the AXI architecture refactor —
      the RX bring-up, CDC UART path, and main.c cleanup work below were
      all confirmed end-to-end on the actual board, not just sim/synth
- [x] `axi_cdc_status` reachable over UART: new `CMD_DEV_CDC` (`0x0C`)
      device byte in `process_uart_command()`, read-only, confirmed on
      hardware to reproduce the same `valid_count`/`error_count`/
      `rx_frame_s` values as the JTAG diagnostic scripts
- [x] `bring_up_uart.txt` rewritten as a hand-verified reference: mode
      entry, RX diagnostics poll (via the new CDC device), coarse-delay
      SPI write, and chip-status SPI reads — every line confirmed against
      real hardware output
- [x] `sw/main.c` cleaned up: removed the `0xAA`/`0xBB`/`0xCC` UART
      bisection sentinels (their hang was found and fixed) and the
      commented-out boot-time LED blink demo (superseded by the UART
      console) — `main()` is now just init + `ad9361_common_init()` +
      the command loop
- [x] RX LO synthesizer (target 98MHz) derived and verified on real
      hardware — VCO locks, `REG_STATE` reaches real RX (`0x08`), `rx_data`
      changes on every sample. Coarse `RX_DATA_DELAY` tuned to `0x0B` the
      same way. **Now permanent**: both live in `sw/main.c`'s
      `ad9361_common_init()`, run unconditionally at boot, re-confirmed on
      hardware after porting
- [ ] Confirm `valid_count` behavior with a longer soak test at
      `RX_DATA_DELAY=0x0B` before treating it as fully settled
- [ ] Mission mode (real antenna signal, not BIST) still unverified on
      hardware — no RF source confirmed ready yet
- [ ] `axi_cdc_status` has no real producer yet — `fm_receiver.sv` drives
      it with a diagnostic mirror (`cdc_mirror_seq`) built to debug the RX
      interface, not the originally-intended sample valid/error counter
      logic. Next step now that real RX data flows
- [ ] No real `dsp_clk`-domain reset exists — `axi_cdc_status`'s
      `dsp_rstb` is tied to `1'b1`, matching the rest of this project's
      `dsp_clk` logic (which has no reset input at all today)
- [ ] `axi_interconnect.sv` is dead code (nothing instantiates it) —
      not yet deleted, pending a decision
- [ ] `constraints.xdc`'s `rx_clk` constraint (currently 250MHz, a stale
      guess) needs updating once a final RX sample rate is settled
- [ ] Automated DAP-wedge detection in `ps7_jtag_init.tcl`/`load_run_app.tcl`
      (currently a manual "check the target list, power-cycle if it's
      missing cores" step — see gotchas above)
- [ ] `.bss` zeroing loop in `startup.S`/`linker.ld` — not needed yet since
      `main.c` has no globals, will be needed the first time it does
- [ ] VS Code task picker occasionally flashes open and closes immediately
      on some invocation paths; Command Palette → "Tasks: Run Task" is a
      reliable workaround, root cause not yet investigated
- [ ] `explanation.md`/`blinky.md` predate this AXI/bare-metal phase and
      haven't been revisited since — may be stale in places

## Other docs in this repo

- `explanation.md` — field-by-field breakdown of what's inside
  `design_1.bd`/`.xci` and why. Predates the AXI/software phases above.
- `blinky.md` — the original phased plan for getting the ARM core to
  control the LED's blink status/divider. That plan is now complete (see
  status above); kept for history rather than as an active roadmap.
