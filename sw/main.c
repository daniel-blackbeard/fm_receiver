/*
 * Bare-metal bring-up test for the PL-side AXI peripheral cluster
 * (src/axi_if.sv + src/axi_registers.sv + src/axi_spi.sv), run on the
 * Cortex-A9 over JTAG (no FSBL/BSP -- see startup.S). Raw pointer access
 * only, no HAL/BSP.
 *
 * Addressing: axi_if fans wen/ren out to every peripheral in parallel; each
 * self-selects on addr[23:16] == its own PERIPH_ID (see src/axi_if.sv).
 * addr[31:24] is fixed by GP0's window (0x40-0x7F) so PERIPH_ID can't live
 * there. See README.md's "Zynq-7000 GP0/GP1 address map" before adding a
 * peripheral.
 *
 * axi_registers (PERIPH_ID 0x01), word-aligned:
 *   0x00 CTRL, bit[0] = blink enable
 *   0x04 DIV,  bits[4:0] = which counter bit drives the LED
 */

#include <stddef.h>
#include <stdint.h>

#include "eth0.h"

#define AXI_BASE (0x40000000u | (0x01u << 16)) /* GP0 base | axi_registers' PERIPH_ID */

/*
 * axi_spi (PERIPH_ID 0x02): direct single-transaction bridge, not a
 * register file. One AXI write/read IS one complete AD9361 SPI write/read
 * (AD9361 register address in the low 10 bits of the AXI address); both
 * block on BVALID/RVALID until the SPI transaction finishes. See
 * src/axi_spi.sv's header for the full contract.
 */
#define SPI_AXI_BASE (0x40000000u | (0x02u << 16)) /* GP0 base | axi_spi's PERIPH_ID */

/*
 * axi_cdc_status (PERIPH_ID 0x03): read-only 64 x 8-bit-lane status regmap
 * native to dsp_clk (src/axi_cdc_status.sv). reg0-9 = RX-interface
 * diagnostics, reg10-63 reserved. `addr` is a word-aligned byte offset;
 * one AXI read returns one 8-bit lane, zero-extended.
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
 * PS7 UART1 (hard peripheral, MIO 8/9 = TX/RX), not AXI/GP0. Offsets/bits
 * from Xilinx's BSP header (docs/.../uartps_v3_11/src/xuartps_hw.h). MIO
 * routing + clock gate already done by ps7_init.tcl; this is register
 * setup only.
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
 * Crude busy-wait: no timer peripheral is set up, so this is an
 * uncalibrated approximate delay, not something to build real timing on.
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
 * Command protocol (UART1 and UDP, see below): 8 raw bytes, MSB-first:
 *   byte 0 = device select (AXI/SPI/CDC/GEM/DESC/SLCR/DESC_RX/RXBUF/SYS)
 *   byte 1 = 0x00 read, 0x01 write (CDC ignores writes, read-only from AXI)
 *   bytes [2:3] = 16-bit address, big-endian, byte offset from the
 *                 device's base (SPI: AD9361 register, bits[9:0])
 *   bytes [4:7] = 32-bit data (write value; SPI uses only the low byte)
 * A read's 32-bit reply goes back the same way it arrived, big-endian.
 *
 * The same 8-byte shape is also accepted over UDP (port UDP_CMD_PORT,
 * board IP BOARD_IP0-3, eth0.h), gated by a required 4-byte preamble
 * (UDP_CMD_PREAMBLE) so stray broadcast/multicast traffic can't be
 * executed as commands. After the preamble: a back-to-back array of
 * commands, run until a stop sentinel (8 bytes of 0xFF) or the payload
 * ends. Replies still go to UART as normal, and are also batched into one
 * UDP reply sent after the array finishes.
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
#define CMD_DEV_NOTIF   0x24u
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
 * One AD9361 SPI transaction: a single store/load to axi_spi's window IS
 * the write/read (blocks on BVALID/RVALID). ad9361_addr (10 bits) is
 * shifted up by 2 rather than placed at [9:0] directly so the resulting
 * pointer stays word-aligned -- AD9361 register numbers aren't restricted
 * to multiples of 4, and an unaligned store to Strongly-Ordered memory is
 * an immediate ARM Alignment Fault.
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
 * AD9361 registers/bitfields to bring the chip up far enough for either
 * test or mission mode: pin release, BBPLL, LVDS parallel port. Verified
 * against ADI's driver (docs/.../ad9361.c: ad9361_setup()/
 * ad9361_bbpll_set_rate()).
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

/* REG_ENSM_CONFIG_1: ENSM state-machine control, used to force the real
 * ALERT->RX transition (common to both modes). See ad9361_registers.md /
 * ADI's ad9361_ensm_set_state() for the full bit layout. */
#define AD9361_REG_ENSM_CONFIG_1 0x014u
#define AD9361_FORCE_RX_ON                 (1u << 6)
#define AD9361_LEVEL_MODE                  (1u << 3)
#define AD9361_FORCE_ALERT_STATE           (1u << 2)
#define AD9361_TO_ALERT                    (1u << 0)

/* REG_OBSERVE_CONFIG / REG_BIST_CONFIG: test-mode RX-side BIST/PRBS
 * pattern generator, per ad9361_registers.md / ADI's ad9361_bist_prbs(). */
#define AD9361_REG_OBSERVE_CONFIG 0x3F5u

#define AD9361_REG_BIST_CONFIG 0x3F4u
#define AD9361_BIST_ENABLE            (1u << 0)
#define AD9361_BIST_CTRL_POINT_RX(x)  (((x) & 0x3u) << 2)

/*
 * RX LO synthesizer (98MHz) + coarse RX_DATA_DELAY bring-up. Derived from
 * ADI's ad9361_txrx_synth_cp_calib()/ad9361_rfpll_vco_init()/
 * ad9361_rfpll_int_set_rate(), hand-verified via scripts/_rx_lo_synth_98mhz.tcl
 * and scripts/_clkdata_delay_sweep_live.tcl -- see README.md's "AD9361 RX
 * digital bring-up". Common to both modes: reaching real RX state is what
 * unblocks BIST/PRBS data flowing too.
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
 * One-time AD9361 bring-up shared by test and mission mode: release the
 * control pins, lock the BBPLL, configure the LVDS parallel port, lock
 * the RX LO synth (98MHz), tune RX_DATA_DELAY, force ALERT->RX. Runs once
 * at boot. Does NOT set the RX sample rate/clock-divider chain -- see
 * bring_up_uart.txt, kept manual since it's still an actively-tuned knob.
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

    /* Deliberately stops here: RX sample rate (REG_BBPLL +
     * REG_RX_ENABLE_FILTER_CTRL) stays a runtime UART knob, not baked in
     * -- see bring_up_uart.txt. */
}

/*
 * Test mode: chip is already in real RX state (ad9361_common_init()
 * forces ALERT->RX at boot). Only remaining step: enable the RX-side
 * BIST pattern generator (BIST_CTRL_POINT=2=RX injection).
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
        if (addr & 0x3u) { return 0u; }
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
            if (addr & 0x3u) { return 0u; }
            volatile uint32_t *reg = (volatile uint32_t *)(CDC_AXI_BASE + addr);
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_GEM) {
        if (addr & 0x3u) { return 0u; }
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_CORE_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_SLCR) {
        if (addr & 0x3u) { return 0u; }
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_SLCR_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_DESC) {
        if (addr & 0x3u) { return 0u; }
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_DESCRIPTOR_TX + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_DESC_RX) {
        if (addr & 0x3u) { return 0u; }
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_DESCRIPTOR_RX + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_RXBUF) {
        if (addr & 0x3u) { return 0u; }
        volatile uint32_t *reg = (volatile uint32_t *)(GEM_RX_BUF_BASE + addr);
        if (rw & CMD_RW_WRITE) {
            *reg = data;
        } else {
            *reply = *reg;
            has_reply = 1u;
        }
    } else if (dev == CMD_DEV_NOTIF) {
        /* axi_notifications: raw register peek for debugging --
         * eth_poll_sample_stream() is the real consumer. Writes ignored
         * (same pattern as CMD_DEV_CDC). */
        if (!(rw & CMD_RW_WRITE)) {
            if (addr & 0x3u) { return 0u; }
            volatile uint32_t *reg = (volatile uint32_t *)(NOTIF_AXI_BASE + addr);
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
 * Stop sentinel for a UDP command array: all 8 bytes 0xFF -- a pattern no
 * real command uses (every real device select value is well under 0xFF).
 */
static uint8_t is_stop_command(const uint8_t *cmd)
{
    for (uint32_t i = 0u; i < 8u; i++) {
        if (cmd[i] != 0xFFu) {
            return 0u;
        }
    }
    return 1u;
}

/*
 * Required first 4 bytes of any UDP command payload -- ASCII "COM" + NUL.
 * Without this, any stray UDP packet matching the board's IP:port would
 * have its payload blindly executed as register read/write commands.
 */
static const uint8_t UDP_CMD_PREAMBLE[4] = { 'C', 'O', 'M', 0x00u };

/*
 * Execute a UDP-carried command array. `payload_len` is already bounded
 * to the smaller of the UDP header's claimed length and the RX
 * descriptor's actual length -- never trust the stop sentinel alone,
 * since a truncated payload might never contain one. Replies go to UART
 * as usual and are also batched into one UDP reply sent at the end.
 */
static void eth_process_udp_command_frame(const uint8_t *req_frame, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t reply_batch[ETH_UDP_MAX_REPLY_BYTES];
    uint16_t reply_bytes = 0u;
    uint16_t offset;

    if (payload_len < sizeof(UDP_CMD_PREAMBLE)
        || payload[0] != UDP_CMD_PREAMBLE[0] || payload[1] != UDP_CMD_PREAMBLE[1]
        || payload[2] != UDP_CMD_PREAMBLE[2] || payload[3] != UDP_CMD_PREAMBLE[3]) {
        return;
    }
    offset = (uint16_t)sizeof(UDP_CMD_PREAMBLE);

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
 * Called once per main-loop iteration: drains every RX frame already
 * waiting (not just one), dispatching each by EtherType/protocol/port and
 * always releasing its descriptor. Draining everything per call (rather
 * than one frame at a time) keeps traffic bursts from exhausting the
 * 8-entry ring faster than it's serviced.
 */
static void eth_service(void)
{
    uint16_t len;
    uint8_t *frame;

    while ((frame = (uint8_t *)eth_rx_poll(&len)) != NULL) {
        uint16_t ethertype = ((uint16_t)frame[12] << 8) | frame[13];
        if (ethertype == 0x0806u) {
            /* ARP: 14-byte Ethernet header + 28-byte payload = 42 bytes
             * minimum -- guard against reading past a short frame. */
            if (len >= 42u) {
                uint16_t oper = ((uint16_t)frame[20] << 8) | frame[21];
                uint8_t tpa_match = frame[38] == BOARD_IP0 && frame[39] == BOARD_IP1
                                  && frame[40] == BOARD_IP2 && frame[41] == BOARD_IP3;
                if (oper == 1u && tpa_match) {
                    eth_send_arp_reply(frame);
                }
            }
        } else if (ethertype == 0x0800u) {
            /* Same reasoning as ARP above: header fields read below need
             * ETH_UDP_HEADER_LEN (42) bytes minimum. */
            if (len >= ETH_UDP_HEADER_LEN) {
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
        }

        eth_rx_release();
    }

    /* GEM latches BNA when it finds the current descriptor still
     * software-owned with a frame ready to deposit, and won't resume
     * until software both frees a descriptor (done above) and explicitly
     * acknowledges BNA here (write-1-to-clear). */
    if (GEM_RXSR & 0x1u) {
        GEM_RXSR = 0x1u;
    }
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

    /* Service whichever of UART/Ethernet has work each pass, forever.
     * uart1_available() is non-blocking so a missing UART byte can't
     * starve eth_service(). eth_poll_sample_stream() is a cheap register
     * check when axi_dsp has nothing new, so it costs nothing to poll
     * every iteration alongside the others. */
    for (;;) {
        if (uart1_available()) {
            process_uart_command();
        }
        eth_service();
        eth_poll_sample_stream();
    }
}
