/*
 * Minimal bare-metal bring-up test for the PL-side AXI peripheral cluster
 * (src/axi_if.sv + src/axi_registers.sv + src/axi_spi.sv), run on the
 * Cortex-A9 over JTAG (no FSBL/BSP — see startup.S for what that implies
 * about CPU state at entry).
 *
 * Raw pointer access only, no HAL/BSP macros exist in this project to use
 * instead.
 *
 * Addressing: axi_if fans a single wen/ren bus out to every peripheral in
 * parallel (no interconnect, no per-slave address range) -- each
 * peripheral self-selects by comparing addr[23:16] against its own
 * PERIPH_ID (see src/axi_if.sv's port-list comment). This replaced the
 * old axi_interconnect range-decode scheme entirely. addr[31:24] is
 * NEVER inspected by any peripheral -- it's whatever M_AXI_GP0 fixes it
 * to (always 0x40-0x7F, since GP0's window is 0x4000_0000-0x7FFF_FFFF;
 * this design doesn't wire up GP1 at all), which is exactly why the
 * PERIPH_ID field lives at addr[23:16] rather than addr[31:24] --
 * putting it there would make it unreachable, since a real GP0 address
 * can never have a top byte outside 0x40-0x7F. See README.md's
 * "Zynq-7000 GP0/GP1 address map" section before ever picking a base for
 * a new peripheral.
 *
 * axi_registers (PERIPH_ID 0x01) register map, word-aligned:
 *   offset 0x00 (register 0) = CTRL, bit[0] = blink enable
 *   offset 0x04 (register 1) = DIV,  bits[4:0] = which counter bit drives
 *                               the LED
 */

#include <stddef.h>
#include <stdint.h>

#include "eth0.h"

#define AXI_BASE (0x40000000u | (0x01u << 16)) /* GP0 base | axi_registers' PERIPH_ID */

/*
 * axi_spi (PERIPH_ID 0x02) is a direct, single-transaction bridge, not a
 * register file -- there's no CTRL/ADDR/WDATA/STATUS staging anymore
 * (that was an earlier, since-abandoned design). One AXI write IS one
 * complete AD9361 SPI write (AD9361 register address in the low 10 bits
 * of the AXI address, data byte in the write data); one AXI read IS one
 * complete SPI read the same way. Both block on the AXI bus itself
 * (BVALID/RVALID) until the real ~24-edge SPI transaction finishes, so no
 * manual status polling is needed here -- see src/axi_spi.sv's header
 * comment for the full contract.
 */
#define SPI_AXI_BASE (0x40000000u | (0x02u << 16)) /* GP0 base | axi_spi's PERIPH_ID */

/*
 * axi_cdc_status (PERIPH_ID 0x03) is a read-only, 64 x 8-bit-lane status
 * regmap native to dsp_clk (see src/axi_cdc_status.sv) -- reg0-9 currently
 * carry RX-interface diagnostics (rx_data snapshot, valid/error counters,
 * raw rx_frame_s/rx_data taps, a dsp_clk heartbeat), reg10-63 reserved for
 * the user's own correlator/histogram DSP work. `addr` below is a byte
 * offset from this base, word-aligned (bits[1:0] ignored) -- one AXI read
 * returns one 8-bit lane, zero-extended, matching the addresses used by
 * this project's JTAG diagnostic scripts (e.g. CDC_AXI_BASE+0x04 = reg1
 * = valid_count).
 */
#define CDC_AXI_BASE (0x40000000u | (0x03u << 16)) /* GP0 base | axi_cdc_status's PERIPH_ID */

/*
 * axi_registers word 2 (offset 0x08) -- the AD9361's physical control
 * pins, per src/fm_receiver.sv's regmap[64:67] assignment.
 */
#define REG_AD9361_PINS (*(volatile uint32_t *)(AXI_BASE + 0x08u))
#define PIN_ENABLE   (1u << 0)
#define PIN_TXNRX    (1u << 1)
#define PIN_RESETB   (1u << 2) /* 1 = reset released */
#define PIN_R1_MODE  (1u << 3) /* ad3961_if_rx: 1 = single-RF (R1) decode path */

/*
 * PS7 UART1 (hard peripheral, MIO 8/9 = TX/RX), not AXI/GP0 — this lives
 * on the PS's own internal bus, address/offsets/bit values below taken
 * directly from the vendor's Xilinx BSP header
 * (docs/.../libsrc/uartps_v3_11/src/xuartps_hw.h), not guessed from the
 * TRM by hand. MIO routing + the AMBA clock gate for UART1 are both already
 * programmed by ps7_init.tcl (run by scripts/ps7_jtag_init.tcl) as part of
 * bringing up the rest of PS7 — same mechanism that already brings up DDR —
 * so nothing extra is needed there, this is register-level setup only.
 */
#define UART1_BASE 0xE0001000u
#define UART_CR      (*(volatile uint32_t *)(UART1_BASE + 0x00u)) /* Control */
#define UART_MR      (*(volatile uint32_t *)(UART1_BASE + 0x04u)) /* Mode */
#define UART_BAUDGEN (*(volatile uint32_t *)(UART1_BASE + 0x18u)) /* Baud rate generator (CD) */
#define UART_SR      (*(volatile uint32_t *)(UART1_BASE + 0x2Cu)) /* Channel status */
#define UART_FIFO    (*(volatile uint32_t *)(UART1_BASE + 0x30u)) /* TX/RX FIFO data */
#define UART_BAUDDIV (*(volatile uint32_t *)(UART1_BASE + 0x34u)) /* Baud rate divider (BDIV) */

#define UART_CR_TXRST  (1u << 1)
#define UART_CR_RXRST  (1u << 0)
#define UART_CR_TX_EN  (1u << 4)
#define UART_CR_RX_EN  (1u << 2)

#define UART_MR_PARITY_NONE   0x00000020u /* bits [5:3] = 1xx per the header */
#define UART_MR_CHARLEN_8BIT  0x00000000u
#define UART_MR_STOPMODE_1BIT 0x00000000u
#define UART_MR_CHMODE_NORMAL 0x00000000u

#define UART_SR_TXFULL  (1u << 4)
#define UART_SR_RXEMPTY (1u << 1)

/*
 * Crude busy-wait: no timer peripheral is set up in this minimal bring-up
 * (see startup.S), so there's no calibrated time reference to delay
 * against. This is an approximate "hold this state long enough for a
 * human to see it" loop, not a precise delay — good enough for a visual
 * bring-up test, not something to build real timing on. The volatile
 * asm nop is there so the loop can't be optimized away regardless of
 * what optimization flags this ends up built with.
 */
static void delay(volatile uint32_t count)
{
    while (count--) {
        __asm__ volatile ("nop");
    }
}

/*
 * TXRST/RXRST are self-clearing once the reset completes — this poll loop
 * mirrors the vendor's own XUartPs_ResetHw(), not an assumption. Clears
 * whatever CR state ps7_init.tcl left behind before this app sets its own.
 */
static void uart1_init(void)
{
    UART_CR = UART_CR_TXRST | UART_CR_RXRST;
    while (UART_CR & (UART_CR_TXRST | UART_CR_RXRST)) { }

    /* 8 data bits, no parity, 1 stop bit, normal (non-loopback) mode. */
    UART_MR = UART_MR_PARITY_NONE | UART_MR_CHARLEN_8BIT
            | UART_MR_STOPMODE_1BIT | UART_MR_CHMODE_NORMAL;

    /* 115200 baud @ 100 MHz UART reference clock (PCW_UART_PERIPHERAL_FREQMHZ):
     * baud = ref_clk / (BAUDGEN * (BAUDDIV + 1)) = 100e6 / (124 * 7) ~= 115207 */
    UART_BAUDGEN = 124u;
    UART_BAUDDIV = 6u;

    UART_CR = UART_CR_TX_EN | UART_CR_RX_EN;
}

static void uart1_putc(uint8_t c)
{
    while (UART_SR & UART_SR_TXFULL) { }
    UART_FIFO = c;
}

/*
 * Non-blocking availability check, needed now that the main loop must
 * also service Ethernet each iteration instead of blocking indefinitely
 * on the next UART byte.
 */
static uint8_t uart1_available(void)
{
    return !(UART_SR & UART_SR_RXEMPTY);
}

static uint8_t uart1_getc(void)
{
    while (UART_SR & UART_SR_RXEMPTY) { }
    return (uint8_t)UART_FIFO;
}

/*
 * Command protocol over UART1: host sends 8 raw bytes per command (not
 * ASCII hex text), MSB-first, forming a 64-bit word:
 *   byte 7 (received first) = device select: 0x00 = AXI (axi_registers),
 *                              0x08 = SPI (axi_spi -> AD9361),
 *                              0x0C = CDC (axi_cdc_status, read-only),
 *                              0x10 = GEM (GEM0 core registers),
 *                              0x14 = DESC (TX descriptor, DDR scratch),
 *                              0x18 = SLCR (GEM_SLCR_BASE),
 *                              0x1C = DESC_RX (RX descriptor, DDR scratch),
 *                              0x20 = RXBUF (RX per-slot buffers, DDR
 *                              scratch) -- offsets/bases for GEM/DESC/SLCR/
 *                              DESC_RX/RXBUF are in eth0.h
 *
 * The same 8-byte command shape is also accepted over UDP (port
 * UDP_CMD_PORT, board IP BOARD_IP0-3, both in eth0.h): a request payload
 * is a back-to-back array of these 8-byte commands, executed in order
 * until either a stop command (all 0xFF in bytes 0-3, all 0x00 in bytes
 * 4-7 -- i.e. 0xFFFFFFFF00000000) or the payload runs out, whichever
 * comes first. Every command that would produce a reply over UART still
 * does; those same reply values are ALSO packed in order into one
 * batched UDP reply datagram sent back to the requester once the array
 * finishes (not one UDP frame per command). See eth_process_udp_command_frame().
 *   byte 6                  = 0x00 read, 0x01 write (CDC ignores writes --
 *                               the peripheral itself is read-only from AXI)
 *   bytes [5:4]              = 16-bit address, big-endian — a byte offset
 *                               from the selected device's base, except
 *                               SPI (the AD9361 register address, bits[9:0])
 *   bytes [3:0]              = 32-bit data (write value, ignored on
 *                               read) — for SPI, only the low byte is
 *                               used
 * On a read, the 32-bit value is sent back over UART1 as 4 raw bytes,
 * big-endian, mirroring the request's own data field layout. For SPI
 * reads this is the AD9361 data byte, zero-extended; for CDC reads this
 * is the addressed 8-bit lane, zero-extended.
 */
#define CMD_DEV_AXI  0x00u
#define CMD_DEV_SPI  0x08u
#define CMD_DEV_SYS  0x04u
#define CMD_DEV_CDC  0x0Cu
#define CMD_DEV_GEM  0x10u
#define CMD_DEV_DESC 0x14u
#define CMD_DEV_SLCR 0x18u
#define CMD_DEV_DESC_RX 0x1Cu
#define CMD_DEV_RXBUF   0x20u
#define CMD_RW_WRITE (1u << 0)

/*
 * System-settings device (CMD_DEV_SYS): low 8 bits of the 32-bit data
 * field select a mode (widened from the original 2 bits -- 0x01-0x03
 * still mean exactly what they always did); the remaining 24 bits are
 * reserved for future per-mode configuration.
 */
#define SYS_MODE_TEST            0x01u
#define SYS_MODE_MISSION         0x02u
#define SYS_MODE_ETH_TEST_FRAME  0x03u
#define SYS_MODE_ETH_INJECT_ARP  0x04u

static void uart1_put32(uint32_t v)
{
    uart1_putc((uint8_t)(v >> 24));
    uart1_putc((uint8_t)(v >> 16));
    uart1_putc((uint8_t)(v >> 8));
    uart1_putc((uint8_t)v);
}

/*
 * One full AD9361 SPI register transaction. A single store to axi_spi's
 * window IS the write (AXI blocks on BVALID until the real SPI write
 * completes); a single load IS the read the same way (blocks on RVALID).
 * No separate launch/poll steps anymore. ad9361_addr only uses its low
 * 10 bits (AD9361's ADDR field), matching axi_spi.sv's own
 * r_addr[11:2]/w_addr[11:2] usage -- shifted up by 2 (not placed at
 * [9:0] directly) specifically so the resulting pointer stays word-
 * aligned: AD9361 register numbers are arbitrary 10-bit values, not
 * restricted to multiples of 4, and a plain 32-bit store/load to an
 * unaligned Strongly-Ordered/Device address is an immediate ARM
 * Alignment Fault, not something that reaches the AXI bus at all. Found
 * this the hard way on real hardware: REG_CTRL (0x3DF) faulted on the
 * very first SPI write, landing in _data_abort_handler with
 * DFSR=0x801 (alignment fault, write) -- see the halt-and-inspect
 * procedure in README.md if this class of bug shows up again.
 */
static uint8_t spi_transact(uint16_t ad9361_addr, uint8_t is_write, uint8_t wdata)
{
    volatile uint32_t *reg = (volatile uint32_t *)(SPI_AXI_BASE + ((uint32_t)(ad9361_addr & 0x3FFu) << 2));

    if (is_write) {
        *reg = wdata;
        return 0u; /* not meaningful on a write */
    }

    return (uint8_t)(*reg);
}

/*
 * AD9361 registers/bitfields needed to bring the chip up to the point
 * where either test or mission mode can do anything -- pin release,
 * BBPLL, LVDS parallel port. Values verified register-by-register
 * against ADI's driver (docs/.../AD936X_PS/AD936X/ad9361/ad9361.c:
 * ad9361_setup()/ad9361_bbpll_set_rate()), not guessed.
 */
#define AD9361_REG_CTRL      0x3DFu
#define AD9361_CTRL_ENABLE   (1u << 0)

#define AD9361_REG_BANDGAP_CONFIG0 0x2A6u
#define AD9361_REG_BANDGAP_CONFIG1 0x2A8u

#define AD9361_REG_REF_DIVIDE_CONFIG_1   0x2ABu
#define AD9361_REF_DIVIDE_CONFIG_1_DFLT  (1u << 2) /* POR default -- must stay set, not a config choice */
#define AD9361_RX_REF_RESET_BAR          (1u << 1)

#define AD9361_REG_REF_DIVIDE_CONFIG_2 0x2ACu

#define AD9361_REG_CLOCK_ENABLE   0x009u
#define AD9361_XO_BYPASS          (1u << 4) /* board uses external refclk, not onboard XTAL */
#define AD9361_DIGITAL_POWER_UP   (1u << 2)
#define AD9361_CLOCK_ENABLE_DFLT  (1u << 1)
#define AD9361_BBPLL_ENABLE       (1u << 0)

#define AD9361_REG_CP_CURRENT     0x046u
#define AD9361_REG_LOOP_FILTER_1  0x048u
#define AD9361_REG_LOOP_FILTER_2  0x049u
#define AD9361_REG_LOOP_FILTER_3  0x04Au

#define AD9361_REG_VCO_CTRL              0x04Bu
#define AD9361_FREQ_CAL_ENABLE           (1u << 7)
#define AD9361_FREQ_CAL_COUNT_LENGTH(x)  (((x) & 0x3u) << 5)

#define AD9361_REG_VCO_PROGRAM_1  0x04Cu
#define AD9361_REG_VCO_PROGRAM_2  0x04Du
#define AD9361_REG_SDM_CTRL       0x04Eu

#define AD9361_REG_SDM_CTRL_1   0x03Fu
#define AD9361_INIT_BB_FO_CAL   (1u << 2)
#define AD9361_BBPLL_RESET_BAR  (1u << 0)

#define AD9361_REG_INTEGER_BB_FREQ_WORD  0x044u
#define AD9361_REG_FRACT_BB_FREQ_WORD_1  0x041u
#define AD9361_REG_FRACT_BB_FREQ_WORD_2  0x042u
#define AD9361_REG_FRACT_BB_FREQ_WORD_3  0x043u

#define AD9361_REG_CH_1_OVERFLOW 0x05Eu
#define AD9361_BBPLL_LOCK        (1u << 7)

#define AD9361_REG_PARALLEL_PORT_CONF_1 0x010u
#define AD9361_REG_PARALLEL_PORT_CONF_2 0x011u
#define AD9361_REG_PARALLEL_PORT_CONF_3 0x012u

/*
 * REG_ENSM_CONFIG_1 -- the ENSM state-machine control register, used by
 * ad9361_common_init() to force the real ALERT->RX transition (common to
 * both modes, see below). ad9361_registers.md and ADI's own driver
 * (docs/.../AD936X_PS/AD936X/ad9361/ad9361.c/.h, ad9361_ensm_set_state())
 * document the full bit layout; only the bits actually used here are named.
 */
#define AD9361_REG_ENSM_CONFIG_1 0x014u
#define AD9361_FORCE_RX_ON                 (1u << 6)
#define AD9361_LEVEL_MODE                  (1u << 3)
#define AD9361_FORCE_ALERT_STATE           (1u << 2)
#define AD9361_TO_ALERT                    (1u << 0)

/*
 * REG_OBSERVE_CONFIG / REG_BIST_CONFIG -- test-mode specific: the RX-side
 * BIST/PRBS pattern generator, per ad9361_registers.md and ADI's driver
 * (ad9361_bist_prbs()/ad9361_bist_loopback(), how ad9361_conv.c calls
 * them). Not a full port of ADI's driver -- just the register writes
 * needed to get the RX-side BIST pattern flowing once the chip is already
 * in real RX state (see ad9361_common_init() below for how it gets there).
 */
#define AD9361_REG_OBSERVE_CONFIG 0x3F5u

#define AD9361_REG_BIST_CONFIG 0x3F4u
#define AD9361_BIST_ENABLE            (1u << 0)
#define AD9361_BIST_CTRL_POINT_RX(x)  (((x) & 0x3u) << 2)

/*
 * RX LO synthesizer (98MHz) + coarse RX_DATA_DELAY bring-up. Derived
 * register-by-register from ADI's driver
 * (docs/.../AD936X_PS/AD936X/ad9361/ad9361.c:
 * ad9361_txrx_synth_cp_calib()/ad9361_rfpll_vco_init()/
 * ad9361_calc_rfpll_int_divder()/ad9361_rfpll_int_set_rate()) and hand-
 * verified on real hardware via scripts/_rx_lo_synth_98mhz.tcl and
 * scripts/_clkdata_delay_sweep_live.tcl -- see README.md's "AD9361 RX
 * digital bring-up" section for the full derivation. Common to both test
 * and mission mode: reaching real RX state (REG_STATE=0x08) is what
 * actually unblocks BIST/PRBS data flowing too, not something
 * mission-mode-specific -- see ad9361_common_init() below.
 */
#define AD9361_REG_ENSM_MODE     0x013u
#define AD9361_FDD_MODE          (1u << 0)

#define AD9361_REG_ENSM_CONFIG_2 0x015u
#define AD9361_DUAL_SYNTH_MODE   (1u << 2)

#define AD9361_TX_REG_OFFSET 0x40u /* RX->TX register offset, used throughout the synth cal/VCO block below */

#define AD9361_REG_RX_CP_LEVEL_DETECT   0x24Bu
#define AD9361_REG_RX_DSM_SETUP_1       0x24Du
#define AD9361_REG_RX_LO_GEN_POWER_MODE 0x261u
#define AD9361_REG_RX_VCO_LDO           0x248u
#define AD9361_REG_RX_VCO_PD_OVERRIDES  0x246u
#define AD9361_REG_RX_CP_CURRENT        0x23Bu
#define AD9361_REG_RX_CP_CONFIG         0x23Du
#define AD9361_CP_OFFSET_OFF  (1u << 4)
#define AD9361_CP_CAL_ENABLE  (1u << 2)
#define AD9361_REG_RX_VCO_CAL   0x249u
#define AD9361_VCO_CAL_EN            (1u << 7)
#define AD9361_VCO_CAL_COUNT(x)      (((x) & 0x3u) << 2)
#define AD9361_FB_CLOCK_ADV(x)       (((x) & 0x3u) << 0)
#define AD9361_REG_RX_CAL_STATUS 0x244u
#define AD9361_CP_CAL_VALID   (1u << 7)

#define AD9361_REG_RX_VCO_OUTPUT          0x23Au
#define AD9361_REG_RX_ALC_VARACTOR        0x239u
#define AD9361_REG_RX_VCO_BIAS_1          0x242u
#define AD9361_REG_RX_FORCE_VCO_TUNE_1    0x238u
#define AD9361_REG_RX_VCO_VARACTOR_CTRL_1 0x251u
#define AD9361_REG_RX_VCO_CAL_REF         0x245u
#define AD9361_REG_RX_VCO_VARACTOR_CTRL_0 0x250u
#define AD9361_REG_RX_LOOP_FILTER_1       0x23Eu
#define AD9361_REG_RX_LOOP_FILTER_2       0x23Fu
#define AD9361_REG_RX_LOOP_FILTER_3       0x240u

#define AD9361_REG_RX_INTEGER_BYTE_0 0x231u
#define AD9361_REG_RX_INTEGER_BYTE_1 0x232u
#define AD9361_REG_RX_FRACT_BYTE_0   0x233u
#define AD9361_REG_RX_FRACT_BYTE_1   0x234u
#define AD9361_REG_RX_FRACT_BYTE_2   0x235u
#define AD9361_REG_RFPLL_DIVIDERS    0x005u

#define AD9361_REG_RX_CP_OVERRANGE_VCO_LOCK 0x247u
#define AD9361_VCO_LOCK  (1u << 1)

#define AD9361_REG_RX_CLOCK_DATA_DELAY 0x006u
#define AD9361_RX_DATA_DELAY_DEFAULT   0x0Bu /* RX_DATA_DELAY<3:0>, DATA_CLK_DELAY<7:4>=0 -- hand-verified via scripts/_clkdata_delay_sweep_live.tcl */

static void ad9361_spi_write(uint16_t ad9361_addr, uint8_t val)
{
    spi_transact(ad9361_addr, 1u, val);
}

static uint8_t ad9361_spi_read(uint16_t ad9361_addr)
{
    return spi_transact(ad9361_addr, 0u, 0u);
}

/*
 * Read-modify-write a sub-field of an AD9361 register: clears `mask` bits
 * at `shift`, then ORs in `value` (masked to fit). Direct port of
 * scripts/_rx_lo_synth_98mhz.tcl's spi_rmw proc.
 */
static void ad9361_spi_rmw(uint16_t ad9361_addr, uint8_t mask, uint8_t shift, uint8_t value)
{
    uint8_t cur = ad9361_spi_read(ad9361_addr);
    uint8_t new_val = (uint8_t)((cur & (uint8_t)~(mask << shift)) | ((value & mask) << shift));
    ad9361_spi_write(ad9361_addr, new_val);
}

/*
 * RX/TX synth charge-pump calibration, shared sequence (offs=0 for RX,
 * offs=AD9361_TX_REG_OFFSET for TX) -- mirrors ADI's
 * ad9361_txrx_synth_cp_calib() (ad9361.c:2845). Temporarily forces
 * REG_ENSM_MODE to FDD and REG_ENSM_CONFIG_1 to FORCE_ALERT_STATE|TO_ALERT
 * regardless of the chip's real operating mode -- both callers of this
 * (RX then TX pass) leave that in place; ad9361_rx_lo_synth_98mhz()
 * restores TDD afterward once both passes have run.
 */
static void ad9361_synth_cp_calib(uint16_t offs)
{
    ad9361_spi_write(AD9361_REG_RX_CP_LEVEL_DETECT + offs, 0x17u);
    ad9361_spi_write(AD9361_REG_RX_DSM_SETUP_1 + offs, 0x00u);
    ad9361_spi_write(AD9361_REG_RX_LO_GEN_POWER_MODE + offs, 0x00u);
    ad9361_spi_write(AD9361_REG_RX_VCO_LDO + offs, 0x0Bu);
    ad9361_spi_write(AD9361_REG_RX_VCO_PD_OVERRIDES + offs, 0x02u);
    ad9361_spi_write(AD9361_REG_RX_CP_CURRENT + offs, 0x80u); /* baseline; refined per-band in ad9361_rx_lo_synth_98mhz() */
    ad9361_spi_write(AD9361_REG_RX_CP_CONFIG + offs, AD9361_CP_OFFSET_OFF);

    /* ref clk 40MHz (<= 40MHz), TDD -> VCO_CAL_COUNT(0); see Table 70 in
     * the AD9361 reference manual for the count-vs-ref-rate table. */
    ad9361_spi_write(AD9361_REG_RX_VCO_CAL + offs,
                      AD9361_VCO_CAL_EN | AD9361_VCO_CAL_COUNT(0u) | AD9361_FB_CLOCK_ADV(2u));

    ad9361_spi_write(AD9361_REG_ENSM_CONFIG_2, AD9361_DUAL_SYNTH_MODE);
    ad9361_spi_write(AD9361_REG_ENSM_CONFIG_1, AD9361_FORCE_ALERT_STATE | AD9361_TO_ALERT);
    ad9361_spi_write(AD9361_REG_ENSM_MODE, AD9361_FDD_MODE);

    ad9361_spi_write(AD9361_REG_RX_CP_CONFIG + offs, AD9361_CP_OFFSET_OFF | AD9361_CP_CAL_ENABLE);

    for (uint32_t tries = 0; tries < 50u; tries++) {
        if (ad9361_spi_read(AD9361_REG_RX_CAL_STATUS + offs) & AD9361_CP_CAL_VALID) {
            break;
        }
        delay(1000u);
    }
}

/*
 * RX LO synthesizer bring-up for a 98MHz RX LO, derived register-by-
 * register from ADI's driver (ad9361_rfpll_vco_init()/
 * ad9361_calc_rfpll_int_divder()/ad9361_rfpll_int_set_rate()) with the
 * same rigor as the BBPLL work below, then hand-verified on real hardware
 * via scripts/_rx_lo_synth_98mhz.tcl. Confirmed the RX/TX synth reference
 * is the 40MHz refclk passed straight through (not doubled) -- decoded
 * from REG_REF_DIVIDE_CONFIG_1/2's already-written bits below, no change
 * needed there.
 *
 * Divider math for 98MHz, parent_rate=40MHz (ad9361_calc_rfpll_int_divder's
 * algorithm: double the target until it clears the 6GHz VCO floor, then
 * compute an integer/fractional-N ratio against the reference):
 *   vco_div = 5 (98MHz << 6 = 6.272GHz)
 *   integer = 156 (0x9C)
 *   fract   = 6710874 (0x66665A), well under RFPLL_MODULUS=8388593
 * VCO band: ad9361_rfvco_tableindex(40MHz) selects LUT_FTDD_40; the row
 * where VCO_MHz first drops <= 6272 is
 * {6270,7,2,7,3,15,13,56,12,15,12,4,13} (ad9361.c:280) -- its bias/
 * varactor/charge-pump/loop-filter fields are copied verbatim below, not
 * recomputed (this is ADI's own characterization data).
 */
static void ad9361_rx_lo_synth_98mhz(void)
{
    ad9361_synth_cp_calib(0u);                    /* RX */
    ad9361_synth_cp_calib(AD9361_TX_REG_OFFSET);   /* TX */
    ad9361_spi_write(AD9361_REG_ENSM_MODE, 0u);    /* restore TDD -- cal above forces FDD */

    /* VCO band row {VCO_MHz=6270, Output_Level=7, Varactor=2, Bias_Ref=7,
     * Bias_Tcf=3, Cal_Offset=15, Varactor_Reference=13, CP_Current=56,
     * LF_C2=12, LF_C1=15, LF_R1=12, LF_C3=4, LF_R3=13} */
    ad9361_spi_write(AD9361_REG_RX_VCO_OUTPUT, 0x47u);           /* VCO_OUTPUT_LEVEL(7)|PORB_VCO_LOGIC */
    ad9361_spi_rmw(AD9361_REG_RX_ALC_VARACTOR, 0xFu, 0u, 2u);    /* VCO_VARACTOR<3:0> */
    ad9361_spi_write(AD9361_REG_RX_VCO_BIAS_1, 0x1Fu);           /* VCO_BIAS_REF(7)|VCO_BIAS_TCF(3) */
    ad9361_spi_write(AD9361_REG_RX_FORCE_VCO_TUNE_1, 0x78u);     /* VCO_CAL_OFFSET(15) */
    ad9361_spi_write(AD9361_REG_RX_VCO_VARACTOR_CTRL_1, 0x0Du);  /* VCO_VARACTOR_REFERENCE(13) */
    ad9361_spi_write(AD9361_REG_RX_VCO_CAL_REF, 0x00u);          /* VCO_CAL_REF_TCF(0) */
    ad9361_spi_write(AD9361_REG_RX_VCO_VARACTOR_CTRL_0, 0x70u);  /* VARACTOR_OFFSET(0)|VARACTOR_REFERENCE_TCF(7) */
    ad9361_spi_rmw(AD9361_REG_RX_CP_CURRENT, 0x3Fu, 0u, 56u);    /* CHARGE_PUMP_CURRENT<5:0>, preserves the bit set by cal above */
    ad9361_spi_write(AD9361_REG_RX_LOOP_FILTER_1, 0xCFu);        /* LF_C2(12)|LF_C1(15) */
    ad9361_spi_write(AD9361_REG_RX_LOOP_FILTER_2, 0xC4u);        /* LF_R1(12)|LF_C3(4) */
    ad9361_spi_write(AD9361_REG_RX_LOOP_FILTER_3, 0x0Du);        /* LF_R3(13) */

    /* N-divider: integer=156 (0x9C), fract=6710874 (0x66665A), vco_div=5 */
    ad9361_spi_write(AD9361_REG_RX_FRACT_BYTE_2, 0x66u);
    ad9361_spi_write(AD9361_REG_RX_FRACT_BYTE_1, 0x66u);
    ad9361_spi_write(AD9361_REG_RX_FRACT_BYTE_0, 0x5Au);
    ad9361_spi_rmw(AD9361_REG_RX_INTEGER_BYTE_1, 0x7u, 0u, 0u);  /* SYNTH_INTEGER_WORD<2:0> = (integer>>8)&0x7 */
    ad9361_spi_write(AD9361_REG_RX_INTEGER_BYTE_0, 0x9Cu);
    ad9361_spi_rmw(AD9361_REG_RFPLL_DIVIDERS, 0xFu, 0u, 5u);     /* RX_VCO_DIVIDER<3:0> */

    for (uint32_t tries = 0; tries < 100u; tries++) {
        if (ad9361_spi_read(AD9361_REG_RX_CP_OVERRANGE_VCO_LOCK) & AD9361_VCO_LOCK) {
            break;
        }
        delay(1000u);
    }
}

/*
 * One-time AD9361 bring-up shared by both test and mission mode: release
 * the chip's physical control pins, lock the BBPLL, configure the LVDS
 * parallel port, lock the RX LO synthesizer (98MHz), tune the coarse
 * RX_DATA_DELAY, and force the real ALERT->RX ENSM transition. Neither
 * mode can do anything before this has run. Runs once at boot so a fresh
 * FPGA upload doesn't need this replayed by hand over UART. Does NOT set
 * the RX sample rate (RX clock-divider chain) -- see bring_up_uart.txt for
 * that, kept manual on purpose (see the comment at the end of this
 * function) since it's still an actively-tuned knob, unlike the RX LO
 * synth/delay values below which are now derived and settled.
 */
static void ad9361_common_init(void)
{
    REG_AD9361_PINS = PIN_ENABLE | PIN_RESETB | PIN_R1_MODE; /* txnrx=0 (RX) */

    ad9361_spi_write(AD9361_REG_CTRL, AD9361_CTRL_ENABLE);
    ad9361_spi_write(AD9361_REG_BANDGAP_CONFIG0, 0x0Eu); /* master bias trim */
    ad9361_spi_write(AD9361_REG_BANDGAP_CONFIG1, 0x0Eu); /* bandgap trim */
    ad9361_spi_write(AD9361_REG_REF_DIVIDE_CONFIG_1,
                      AD9361_REF_DIVIDE_CONFIG_1_DFLT | AD9361_RX_REF_RESET_BAR);
    ad9361_spi_write(AD9361_REG_REF_DIVIDE_CONFIG_2, 0x73u); /* TX_REF_RESET_BAR + ref doubler FB delays */
    ad9361_spi_write(AD9361_REG_CLOCK_ENABLE,
                      AD9361_DIGITAL_POWER_UP | AD9361_CLOCK_ENABLE_DFLT |
                      AD9361_BBPLL_ENABLE | AD9361_XO_BYPASS);

    /* BBPLL: 960MHz from the 40MHz refclk (integer=24, fract=0, exact) */
    ad9361_spi_write(AD9361_REG_CP_CURRENT, 0x03u);
    ad9361_spi_write(AD9361_REG_LOOP_FILTER_1, 0xE8u);
    ad9361_spi_write(AD9361_REG_LOOP_FILTER_2, 0x5Bu);
    ad9361_spi_write(AD9361_REG_LOOP_FILTER_3, 0x35u);
    ad9361_spi_write(AD9361_REG_VCO_CTRL,
                      AD9361_FREQ_CAL_ENABLE | AD9361_FREQ_CAL_COUNT_LENGTH(3u));
    ad9361_spi_write(AD9361_REG_SDM_CTRL, 0x10u); /* cal clock = REFCLK/4 */
    ad9361_spi_write(AD9361_REG_INTEGER_BB_FREQ_WORD, 24u);
    ad9361_spi_write(AD9361_REG_FRACT_BB_FREQ_WORD_3, 0x00u);
    ad9361_spi_write(AD9361_REG_FRACT_BB_FREQ_WORD_2, 0x00u);
    ad9361_spi_write(AD9361_REG_FRACT_BB_FREQ_WORD_1, 0x00u);
    ad9361_spi_write(AD9361_REG_SDM_CTRL_1,
                      AD9361_INIT_BB_FO_CAL | AD9361_BBPLL_RESET_BAR); /* start cal pulse */
    ad9361_spi_write(AD9361_REG_SDM_CTRL_1, AD9361_BBPLL_RESET_BAR); /* clear start-cal bit */
    ad9361_spi_write(AD9361_REG_VCO_PROGRAM_1, 0x86u); /* increase BBPLL KV/phase margin */
    ad9361_spi_write(AD9361_REG_VCO_PROGRAM_2, 0x01u);
    ad9361_spi_write(AD9361_REG_VCO_PROGRAM_2, 0x05u);
    /* Lock is verifiable afterward by reading AD9361_REG_CH_1_OVERFLOW, bit AD9361_BBPLL_LOCK. */

    ad9361_spi_write(AD9361_REG_PARALLEL_PORT_CONF_1, 0xC8u); /* IQ swap, pulse-mode frame, 1R1T timing */
    ad9361_spi_write(AD9361_REG_PARALLEL_PORT_CONF_2, 0x00u); /* no inversions (explicit, not assumed default) */
    ad9361_spi_write(AD9361_REG_PARALLEL_PORT_CONF_3, 0x10u); /* LVDS mode enabled */

    ad9361_rx_lo_synth_98mhz();
    ad9361_spi_write(AD9361_REG_RX_CLOCK_DATA_DELAY, AD9361_RX_DATA_DELAY_DEFAULT);
    ad9361_spi_write(AD9361_REG_ENSM_CONFIG_1,
                      AD9361_LEVEL_MODE | AD9361_TO_ALERT | AD9361_FORCE_RX_ON); /* ALERT -> real RX */

    /*
     * Deliberately stops here for the RX clock-divider chain only
     * (REG_BBPLL + REG_RX_ENABLE_FILTER_CTRL) -- it's still an
     * actively-tuned knob (see bring_up_uart.txt), and baking a sample
     * rate in here would mean a full rebuild+reupload every time it
     * changes instead of just a UART command. The RX LO synth/delay
     * tuning above are different: both are now derived and hand-verified
     * settled values, not still being experimentally tuned, so they run
     * unconditionally here rather than waiting on a mode-select command.
     */
}

/*
 * Test mode: the chip is already in real RX state by the time this can be
 * called (ad9361_common_init() forces the ALERT->RX transition
 * unconditionally at boot -- see there for why that's common to both
 * modes now, not mission-mode-specific). All that's left here is the
 * actual test-mode-specific step: enable the RX-side BIST pattern
 * generator (REG_BIST_CONFIG, BIST_CTRL_POINT=2=RX injection), which
 * substitutes a fixed digital pattern ahead of the RF front end. ADI's
 * driver doesn't expose a PRBS7-vs-other-length select -- this is the
 * chip's one fixed BIST pattern, referred to as PRBS7 per how this
 * project intends to use it, not confirmed against the AD9361 datasheet
 * text itself (not present in this repo's docs).
 */
static void enter_test_mode(void)
{
    ad9361_spi_write(AD9361_REG_OBSERVE_CONFIG, 0u);
    ad9361_spi_write(AD9361_REG_BIST_CONFIG,
                      AD9361_BIST_CTRL_POINT_RX(2u) | AD9361_BIST_ENABLE);
}

/*
 * Mission mode: normal RX operation. The chip is already in real RX state
 * by the time this can be called (see ad9361_common_init()) and the real
 * antenna signal flows through unmodified -- the only mission-mode-
 * specific step is clearing any leftover BIST state, in case test mode
 * ran first.
 */
static void enter_mission_mode(void)
{
    ad9361_spi_write(AD9361_REG_BIST_CONFIG, 0u);
}

/*
 * Core of the command protocol, transport-agnostic: given one already-
 * received 8-byte request (regardless of whether it came from UART or a
 * UDP payload), execute it and report whether a reply value exists (not
 * every device/rw combination produces one -- writes and CDC-writes never
 * do, matching the original UART-only behavior exactly). Both
 * process_uart_command() and eth_process_udp_command_frame() are thin
 * wrappers around this.
 */
static uint8_t dispatch_command(const uint8_t *req, uint32_t *reply)
{
    uint8_t dev     = req[0];
    uint8_t rw      = req[1];
    uint8_t addr_hi = req[2];
    uint8_t addr_lo = req[3];
    uint32_t data = 0;
    uint8_t has_reply = 0u;
    for (uint32_t i = 0; i < 4u; i++) {
        data = (data << 8) | req[4u + i];
    }

    uint16_t addr = (uint16_t)(((uint16_t)addr_hi << 8) | addr_lo);

    if (dev == CMD_DEV_AXI) {
        volatile uint32_t *reg = (volatile uint32_t *)(AXI_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_SPI) {
        uint8_t is_write = (rw & CMD_RW_WRITE) ? 1u : 0u;
        uint8_t rdata = spi_transact(addr, is_write, (uint8_t)data);
        if (!is_write) {
            *reply = (uint32_t)rdata;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_CDC) {
        /* axi_cdc_status is read-only from AXI (any write DECERRs) -- reads
         * only here; writes are simply ignored rather than ever issuing a
         * real AXI store that would fault. */
        if (!(rw & CMD_RW_WRITE)) {
            volatile uint32_t *reg = (volatile uint32_t *)(CDC_AXI_BASE + addr);
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_GEM) {
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_CORE_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_SLCR) {
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_SLCR_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_DESC) {
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_DESCRIPTOR_TX + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_DESC_RX) {
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_DESCRIPTOR_RX + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_RXBUF) {
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_RX_BUF_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_SYS) {
        if (rw & CMD_RW_WRITE) {
            uint8_t mode = (uint8_t)(data & 0xFFu);
            if (mode == SYS_MODE_TEST) {
                enter_test_mode();
            } else if (mode == SYS_MODE_MISSION) {
                enter_mission_mode();
            } else if (mode == SYS_MODE_ETH_TEST_FRAME) {
                eth_send_test_frame();
            } else if (mode == SYS_MODE_ETH_INJECT_ARP) {
                eth_test_inject_arp_request();
            }
        }
    }
    return has_reply;
}

static void process_uart_command(void)
{
    uint8_t req[8];
    uint32_t reply;
    for (uint32_t i = 0; i < 8u; i++) {
        req[i] = uart1_getc();
    }
    if (dispatch_command(req, &reply)) {
        uart1_put32(reply);
    }
}

/*
 * Stop sentinel for a UDP command array: dev/rw/addr all 0xFF, data all
 * 0x00 -- a combination no real command uses (every real device select
 * value is well under 0xFF).
 */
static uint8_t is_stop_command(const uint8_t *cmd)
{
    return cmd[0] == 0xFFu && cmd[1] == 0xFFu && cmd[2] == 0xFFu && cmd[3] == 0xFFu
        && cmd[4] == 0x00u && cmd[5] == 0x00u && cmd[6] == 0x00u && cmd[7] == 0x00u;
}

/*
 * Execute a UDP-carried command array (payload of req_frame, already
 * bounded by the caller to the smaller of the UDP header's own claimed
 * length and the RX descriptor's actual received length -- never trust
 * the stop sentinel alone to terminate, since a truncated/malformed
 * payload might never contain one). Every reply still goes to UART, same
 * as if the identical command had arrived there; replies are also packed
 * back-to-back into one batched UDP reply, sent once at the end -- not
 * one UDP frame per command. If packing the next reply would overflow
 * ETH_UDP_MAX_REPLY_BYTES, processing stops there rather than silently
 * dropping that command's reply from an otherwise-complete-looking batch.
 */
static void eth_process_udp_command_frame(const uint8_t *req_frame, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t reply_batch[ETH_UDP_MAX_REPLY_BYTES];
    uint16_t reply_bytes = 0u;
    uint16_t offset = 0u;

    while (offset + 8u <= payload_len) {
        const uint8_t *cmd = payload + offset;
        if (is_stop_command(cmd)) {
            break;
        }

        uint32_t reply;
        if (dispatch_command(cmd, &reply)) {
            uart1_put32(reply);
            if (reply_bytes + 4u > sizeof(reply_batch)) {
                break;
            }
            reply_batch[reply_bytes + 0u] = (uint8_t)(reply >> 24);
            reply_batch[reply_bytes + 1u] = (uint8_t)(reply >> 16);
            reply_batch[reply_bytes + 2u] = (uint8_t)(reply >> 8);
            reply_batch[reply_bytes + 3u] = (uint8_t)reply;
            reply_bytes += 4u;
        }
        offset += 8u;
    }

    if (reply_bytes > 0u) {
        uint8_t *out = (uint8_t *)eth_udp_reply_reserve(req_frame);
        if (out != NULL) {
            for (uint16_t i = 0; i < reply_bytes; i++) {
                out[i] = reply_batch[i];
            }
            eth_udp_reply_commit(reply_bytes);
        }
    }
}

/*
 * Called once per main-loop iteration: services at most one waiting RX
 * frame per call (matching how process_uart_command() only ever handles
 * one command per call), dispatching by EtherType/protocol/port, then
 * always releases the descriptor whether or not anything matched.
 */
static void eth_service(void)
{
    uint16_t len;
    uint8_t *frame = (uint8_t *)eth_rx_poll(&len);
    if (frame == NULL) {
        return;
    }

    uint16_t ethertype = ((uint16_t)frame[12] << 8) | frame[13];
    if (ethertype == 0x0806u) {
        uint16_t oper = ((uint16_t)frame[20] << 8) | frame[21];
        uint8_t tpa_match = frame[38] == BOARD_IP0 && frame[39] == BOARD_IP1
                          && frame[40] == BOARD_IP2 && frame[41] == BOARD_IP3;
        if (oper == 1u && tpa_match) {
            /* TEMP disabled for crash isolation, 2026-08-26 -- see
             * eth0_mdio_bringup_status.md. Still detects/matches the ARP
             * request, just doesn't build/send a reply, to test whether
             * eth_send_arp_reply()+eth_tx_reserve/commit is the trigger
             * or whether the crash survives with only RX-side activity. */
            /* eth_send_arp_reply(frame); */
        }
    } else if (ethertype == 0x0800u) {
        uint8_t proto = frame[23];
        uint8_t dest_ip_match = frame[30] == BOARD_IP0 && frame[31] == BOARD_IP1
                              && frame[32] == BOARD_IP2 && frame[33] == BOARD_IP3;
        uint16_t dest_port = ((uint16_t)frame[36] << 8) | frame[37];
        if (proto == 17u && dest_ip_match && dest_port == UDP_CMD_PORT) {
            uint16_t udp_len = ((uint16_t)frame[38] << 8) | frame[39];
            uint16_t payload_len = (udp_len > 8u) ? (udp_len - 8u) : 0u;
            uint16_t max_from_frame = (len > ETH_UDP_HEADER_LEN) ? (uint16_t)(len - ETH_UDP_HEADER_LEN) : 0u;
            if (payload_len > max_from_frame) {
                payload_len = max_from_frame;
            }
            eth_process_udp_command_frame(frame, frame + ETH_UDP_HEADER_LEN, payload_len);
        }
    }

    eth_rx_release();
}

void main(void)
{
    uart1_init();
    ad9361_common_init();

    gem_slcr_setup();
    gem_setup();

    /* One-shot MDIO sanity check at boot: PHY ID sent over UART before the
     * command loop starts, so a fresh reload gives an immediate pass/fail
     * signal that bring-up got at least this far. Expect 0x001CC916. */
    uart1_put32(phy_get_rtl_identifier());

    /* PHY link status right after the ID check, so a fresh reload shows
     * both without needing an extra UART round-trip. Bits[5:4]
     * speed(10=1000M,01=100M,00=10M), bit3 duplex(1=full), bit2 link. */
    uart1_put32(phy_get_link_status());

    /* Register read/write console: service whichever of UART/Ethernet has
     * work each pass, forever. uart1_available() is a non-blocking check
     * -- process_uart_command() itself still blocks internally once a
     * command has actually started arriving, which is fine since a host
     * writes all 8 bytes as one burst; it just never blocks indefinitely
     * waiting for a *first* byte that isn't coming, which would otherwise
     * starve eth_service() forever. */
    for (;;) {
        if (uart1_available()) {
            process_uart_command();
        }
        eth_service();
    }
}
