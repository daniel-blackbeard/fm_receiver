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
#define CMD_DEV_RXLO    0x28u /* write-only: data = target RX LO freq in Hz, see ad9361_rx_lo_synth_set() */
#define CMD_DEV_GAIN    0x2Cu /* write-only: addr 0x00 = gain mode select (data = mode, see ad9361_set_rx_gain_mode()), addr 0x04 = manual gain table index (data = 0-76, see ad9361_set_rx_manual_gain()) */
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
#define SYS_MODE_TEST_PATTERN_PRBS  0x05u
#define SYS_MODE_TEST_PATTERN_TONE  0x06u

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

/* RX sample-rate chain (REG_BBPLL's ADC divider + REG_RX_ENABLE_FILTER_CTRL's
 * three half-band/FIR decimation stages) -- distinct from the BBPLL
 * programming registers above, which only lock the 960MHz BBPLL itself.
 * Derived register-by-register from ADI's ad9361_get_clk_scaler()/
 * ad9361_set_clk_scaler() (private/docs/.../ad9361.c), same rigor as the
 * RX LO synth below. See README.md's "AD9361 RX digital bring-up" section
 * for the full chain (BBPLL -> ADC_CLK -> R2_CLK -> R1_CLK -> CLKRF_CLK ->
 * RX_SAMPL_CLK) and how this specific configuration was derived from a
 * live REG_RX_ENABLE_FILTER_CTRL readback (0x5F) that decoded to a
 * completely unconfigured, un-bypassed decimation chain landing at
 * ~7.5MHz -- confirmed two independent ways (CIC-decimated sample-stream
 * rate, and a dsp_clk-domain LED counter) before this fix was written.
 */
#define AD9361_REG_BBPLL                 0x00Au /* BBPLL_DIVIDER<2:0> = bits[2:0]; ADC_CLK = BBPLL_FREQ >> BBPLL_DIVIDER */
#define AD9361_REG_RX_ENABLE_FILTER_CTRL 0x003u

#define AD9361_REG_CH_1_OVERFLOW 0x05Eu
#define AD9361_BBPLL_LOCK        (1u << 7)

/* RX physical port select -- which RX1/RX2 pins (A/B/C, balanced/
 * unbalanced) are actually active on the analog front end. Never written
 * anywhere before 2026-09-13 (this firmware only ever configured the
 * digital/clock side), so the chip ran on its POR default -- harmless to
 * the BIST tone (injected digitally, after this stage entirely) but left
 * real antenna signal badly attenuated/mismatched. 0x03 = RX1A+RX2A
 * balanced (both _N and _P), the standard default on essentially every
 * ADI AD9361 reference board -- confirmed live via SPI poke to fix real-
 * signal reception on this board's RX1 SMA. See ad9361_rf_port_setup()
 * in private/docs/.../ad9361.c for the full encoding if this board turns
 * out to use a different port/pin.
 */
#define AD9361_REG_INPUT_SELECT      0x004u
#define AD9361_INPUT_SELECT_RX1A_RX2A_BALANCED 0x03u

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
 * pattern generator, per ad9361_registers.md / ADI's ad9361_bist_prbs()
 * and ad9361_bist_tone() (private/docs/.../ad9361.c). */
#define AD9361_REG_OBSERVE_CONFIG 0x3F5u

#define AD9361_REG_BIST_CONFIG 0x3F4u
#define AD9361_BIST_ENABLE            (1u << 0)
#define AD9361_TONE_PRBS              (1u << 1) /* 0 = PRBS, 1 = tone */
#define AD9361_BIST_CTRL_POINT_RX(x)  (((x) & 0x3u) << 2)
#define AD9361_TONE_LEVEL(x)          (((x) & 0x3u) << 4) /* 0 = full scale, steps of -6dB */
#define AD9361_TONE_FREQ(x)           (((x) & 0x3u) << 6) /* code -> RX_SAMPL_CLK/32*(code+1) */

/* REG_BIST_AND_DATA_PORT_TEST_CONFIG: per-I/Q-lane BIST mask (1 = exclude
 * that lane from the tone). 0x00 = tone applied to all 4 lanes
 * (ch0_i/q, ch1_i/q). Only meaningful in tone mode. */
#define AD9361_REG_BIST_AND_DATA_PORT_TEST_CONFIG 0x3F6u

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

/*
 * RX gain control (AGC/MGC) -- distinct from the RX LO synth and sample-
 * rate chain above. The AD9361 has no default/built-in RX gain table: it
 * ships out of reset with the internal 77-entry table (REG_GAIN_TABLE_*
 * below) unpopulated, so selecting a gain index -- whether the AGC engine
 * picks it automatically or it's written manually -- looks up meaningless
 * content until the table is host-loaded once. Register set, table
 * content, and load sequence all traced from ADI's ad9361_load_gt() /
 * ad9361_gc_setup() (private/docs/.../ad9361.c) for the 0-1.3GHz band
 * (98MHz falls in it), full-table (not split-table) format, 77 entries --
 * this project's HAVE_SPLIT_GAIN_TABLE equivalent is effectively off, so
 * full-table is the only format that matters here.
 */
#define AD9361_REG_AGC_CONFIG_1             0x0FAu /* RX1 mode = bits[1:0], RX2 mode = bits[3:2]: 0=MGC (manual), 1=fast-attack AGC, 2=slow-attack AGC, 3=hybrid AGC */
#define AD9361_REG_AGC_CONFIG_2             0x0FBu
#define AD9361_AGC_CONFIG_2_VAL             0x08u /* bit0/1 MAN_GAIN_CTRL_RX1/RX2=0 (gain set via SPI register, not external ctrl pins); bit2 DIG_GAIN_EN=0 (digital gain unused); bit3 AGC_USE_FULL_GAIN_TABLE=1 */
#define AD9361_REG_MAX_LMT_FULL_GAIN        0x0FDu

#define AD9361_REG_RX1_MANUAL_LMT_FULL_GAIN      0x109u /* bits[6:0] = gain table index, 0-76 (~ -1dB..73dB for this band, see ADI's abs_gain_tbl) */
#define AD9361_REG_RX1_MANUAL_DIGITALFORCED_GAIN 0x10Bu

#define AD9361_REG_GAIN_TABLE_ADDRESS     0x130u
#define AD9361_REG_GAIN_TABLE_WRITE_DATA1 0x131u
#define AD9361_REG_GAIN_TABLE_WRITE_DATA2 0x132u
#define AD9361_REG_GAIN_TABLE_WRITE_DATA3 0x133u
#define AD9361_REG_GAIN_TABLE_READ_DATA1  0x134u
#define AD9361_REG_GAIN_TABLE_CONFIG      0x137u
#define AD9361_START_GAIN_TABLE_CLOCK     (1u << 1)
#define AD9361_WRITE_GAIN_TABLE           (1u << 2)
#define AD9361_RECEIVER_SELECT_RX1        (1u << 3) /* RECEIVER_SELECT(1) -- RX2 is never loaded, RX2 unused in this design */

/* Pulsed once, right after forcing ENSM into RX while in MGC mode -- ADI's
 * ad9361_ensm_set_state() does this immediately after its own
 * REG_ENSM_CONFIG_1 write, whenever agc_mode==RF_GAIN_MGC. Without it,
 * RX1's analog gain-control block may never actually settle into the
 * loaded table/mode even though the config registers themselves read back
 * correctly -- suspected cause of a real, reproducible sample-rate
 * collapse (dsp_clk-derived CIC decimation strobe dropped from
 * ~8823pkts/s to ~191pkts/s after adding gain control without this pulse,
 * 2026-09-13, see project memory). */
#define AD9361_REG_SMALL_LMT_OVERLOAD_THRESH  0x107u
#define AD9361_SMALL_LMT_OVERLOAD_THRESH_MASK 0x3Fu
#define AD9361_FORCE_PD_RESET_RX1             (1u << 6)

#define AD9361_GAIN_TABLE_SIZE 77u

/* Verbatim from ADI's full_gain_table[TBL_200_1300_MHZ] (ad9361.c) -- per-
 * index {ext/int LNA & mixer gain word, TIA & LPF word, DC-cal bit & digital
 * gain word}. Index i's absolute gain is roughly (i-3)dB for i>=3 (0dB at
 * i=3, ~73dB at i=76), per ADI's full_gain_table_abs_gain for this band. */
static const uint8_t ad9361_gain_table_200_1300mhz[AD9361_GAIN_TABLE_SIZE][3] = {
    {0x00, 0x00, 0x20}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
    {0x00, 0x01, 0x00}, {0x00, 0x02, 0x00}, {0x00, 0x03, 0x00},
    {0x00, 0x04, 0x00}, {0x00, 0x05, 0x00}, {0x01, 0x03, 0x20},
    {0x01, 0x04, 0x00}, {0x01, 0x05, 0x00}, {0x01, 0x06, 0x00},
    {0x01, 0x07, 0x00}, {0x01, 0x08, 0x00}, {0x01, 0x09, 0x00},
    {0x01, 0x0A, 0x00}, {0x01, 0x0B, 0x00}, {0x01, 0x0C, 0x00},
    {0x01, 0x0D, 0x00}, {0x01, 0x0E, 0x00}, {0x02, 0x09, 0x20},
    {0x02, 0x0A, 0x00}, {0x02, 0x0B, 0x00}, {0x02, 0x0C, 0x00},
    {0x02, 0x0D, 0x00}, {0x02, 0x0E, 0x00}, {0x02, 0x0F, 0x00},
    {0x02, 0x10, 0x00}, {0x02, 0x2B, 0x20}, {0x02, 0x2C, 0x00},
    {0x04, 0x28, 0x20}, {0x04, 0x29, 0x00}, {0x04, 0x2A, 0x00},
    {0x04, 0x2B, 0x00}, {0x24, 0x20, 0x20}, {0x24, 0x21, 0x00},
    {0x44, 0x20, 0x20}, {0x44, 0x21, 0x00}, {0x44, 0x22, 0x00},
    {0x44, 0x23, 0x00}, {0x44, 0x24, 0x00}, {0x44, 0x25, 0x00},
    {0x44, 0x26, 0x00}, {0x44, 0x27, 0x00}, {0x44, 0x28, 0x00},
    {0x44, 0x29, 0x00}, {0x44, 0x2A, 0x00}, {0x44, 0x2B, 0x00},
    {0x44, 0x2C, 0x00}, {0x44, 0x2D, 0x00}, {0x44, 0x2E, 0x00},
    {0x44, 0x2F, 0x00}, {0x44, 0x30, 0x00}, {0x44, 0x31, 0x00},
    {0x44, 0x32, 0x00}, {0x64, 0x2E, 0x20}, {0x64, 0x2F, 0x00},
    {0x64, 0x30, 0x00}, {0x64, 0x31, 0x00}, {0x64, 0x32, 0x00},
    {0x64, 0x33, 0x00}, {0x64, 0x34, 0x00}, {0x64, 0x35, 0x00},
    {0x64, 0x36, 0x00}, {0x64, 0x37, 0x00}, {0x64, 0x38, 0x00},
    {0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
    {0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
    {0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
    {0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20}
};

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
 * Generalized RX LO retune, for FM broadcast tuning (78-108MHz-ish) on
 * top of the 98MHz bring-up above. NOT YET hardware-verified -- 98MHz is
 * the only frequency confirmed on real hw so far; this needs a bring-up
 * pass next session (sweep the band edges + a few points in between,
 * check VCO_LOCK and confirm against a known station) before it's
 * trusted the way ad9361_rx_lo_synth_98mhz() is.
 *
 * Derived from ADI's ad9361_calc_rfpll_int_divder() / ad9361_rfpll_vco_init()
 * / ad9361_rfpll_int_set_rate() (private/docs/.../ad9361.c), but unlike
 * the 98MHz bring-up sequence above, this does NOT redo charge-pump
 * calibration (ad9361_txrx_synth_cp_calib()) or touch ENSM state.
 * ad9361_rfpll_int_set_rate() -- ADI's real per-frequency retune path --
 * only rewrites the VCO-band LUT registers and the N-divider, then waits
 * for VCO_LOCK; CP cal is a one-time init step (already done once in
 * ad9361_common_init() via ad9361_rx_lo_synth_98mhz()). So this should be
 * callable at any time after boot, including while already in real RX
 * state -- no ALERT/FDD bounce needed, RX should just glitch briefly
 * while the PLL relocks.
 *
 * The VCO LUT rows below are ADI's SynthLUT_TDD[LUT_FTDD_40] table
 * verbatim (ad9361.c:232) -- the same table the 98MHz sequence pulled
 * its one row from (confirmed TDD not FDD: that row's loop-filter fields
 * only match the TDD table, not the FDD one, at the same VCO_MHz entry).
 * All 53 rows are kept rather than just an FM-band subset -- it's cheap,
 * and avoids having to reason precisely about which rows the divider
 * crossover near ~93.75MHz (where vco_div flips 6->5 inside the FM band)
 * actually touches.
 */
#define AD9361_REFCLK_HZ       40000000UL /* confirmed un-doubled -- see 98MHz comment above */
#define AD9361_RFPLL_MODULUS   8388593UL
#define AD9361_MIN_VCO_FREQ_HZ 6000000000ULL
#define AD9361_RX_LO_MIN_HZ    60000000UL  /* headroom below the 78MHz FM edge */
#define AD9361_RX_LO_MAX_HZ    130000000UL /* headroom above the 108MHz FM edge; also keeps
                                             * the vco_div search below from ever looping on
                                             * a 0 or out-of-range input (external command data) */

struct ad9361_vco_lut_row {
    uint16_t vco_mhz;
    uint8_t  output_level;
    uint8_t  varactor;
    uint8_t  bias_ref;
    uint8_t  bias_tcf;
    uint8_t  cal_offset;
    uint8_t  varactor_ref;
    uint8_t  cp_current;
    uint8_t  lf_c2;
    uint8_t  lf_c1;
    uint8_t  lf_r1;
    uint8_t  lf_c3;
    uint8_t  lf_r3;
};

/* ADI SynthLUT_TDD[LUT_FTDD_40] verbatim, ref clk <=40MHz row (ad9361.c:234-286) */
static const struct ad9361_vco_lut_row ad9361_vco_lut_40mhz[53] = {
    {12605, 13, 1, 4, 2, 15, 12, 27, 12, 15, 12, 4, 13},
    {12245, 13, 1, 4, 2, 15, 12, 27, 12, 15, 12, 4, 13},
    {11906, 13, 1, 4, 2, 15, 12, 26, 11, 15, 12, 4, 13},
    {11588, 13, 1, 4, 2, 15, 12, 28, 12, 15, 12, 4, 13},
    {11288, 13, 1, 4, 2, 15, 12, 30, 12, 15, 12, 4, 13},
    {11007, 13, 1, 4, 2, 15, 12, 32, 12, 15, 12, 4, 13},
    {10742, 13, 1, 4, 2, 15, 12, 33, 12, 15, 12, 4, 13},
    {10492, 13, 1, 6, 2, 15, 12, 35, 12, 15, 12, 4, 13},
    {10258, 13, 1, 6, 2, 15, 12, 37, 12, 15, 12, 4, 13},
    {10036, 13, 1, 6, 2, 15, 12, 38, 12, 15, 12, 4, 13},
    {9827,  13, 1, 6, 2, 14, 12, 40, 12, 15, 12, 4, 13},
    {9631,  13, 1, 6, 2, 13, 12, 42, 12, 15, 12, 4, 13},
    {9445,  13, 1, 6, 2, 12, 12, 44, 12, 15, 12, 4, 13},
    {9269,  13, 1, 6, 2, 12, 12, 45, 12, 15, 12, 4, 13},
    {9103,  13, 1, 6, 2, 12, 12, 47, 12, 15, 12, 4, 13},
    {8946,  13, 1, 6, 2, 12, 12, 49, 12, 15, 12, 4, 13},
    {8797,  12, 1, 7, 2, 12, 12, 48, 12, 15, 12, 4, 13},
    {8655,  12, 1, 7, 2, 12, 12, 50, 12, 15, 12, 4, 13},
    {8520,  12, 1, 7, 2, 12, 12, 51, 12, 15, 12, 4, 13},
    {8392,  12, 1, 7, 2, 12, 12, 53, 12, 15, 12, 4, 13},
    {8269,  12, 1, 7, 2, 12, 12, 55, 12, 15, 12, 4, 13},
    {8153,  12, 1, 7, 2, 12, 12, 56, 12, 15, 12, 4, 13},
    {8041,  12, 1, 7, 2, 13, 12, 58, 12, 15, 12, 4, 13},
    {7934,  11, 1, 7, 2, 12, 12, 57, 12, 15, 12, 4, 13},
    {7831,  11, 1, 7, 2, 12, 12, 58, 12, 15, 12, 4, 13},
    {7733,  10, 1, 7, 3, 13, 12, 56, 12, 15, 12, 4, 13},
    {7638,  10, 1, 7, 2, 12, 12, 58, 12, 15, 12, 4, 13},
    {7547,  10, 1, 7, 2, 12, 12, 59, 12, 15, 12, 4, 13},
    {7459,  10, 1, 7, 2, 12, 12, 61, 12, 15, 12, 4, 13},
    {7374,  10, 2, 7, 3, 14, 13, 49, 12, 15, 12, 4, 13},
    {7291,  10, 2, 7, 3, 14, 13, 50, 12, 15, 12, 4, 13},
    {7212,  10, 2, 7, 3, 14, 13, 51, 12, 15, 12, 4, 13},
    {7135,  10, 2, 7, 3, 14, 13, 52, 12, 15, 12, 4, 13},
    {7061,  10, 2, 7, 3, 14, 13, 53, 12, 15, 12, 4, 13},
    {6988,  10, 1, 7, 3, 12, 14, 63, 11, 14, 12, 3, 13},
    {6918,  9,  2, 7, 3, 14, 13, 52, 12, 15, 12, 4, 13},
    {6850,  9,  2, 7, 3, 14, 13, 53, 12, 15, 12, 4, 13},
    {6784,  9,  2, 7, 2, 13, 13, 54, 12, 15, 12, 4, 13},
    {6720,  9,  2, 7, 2, 13, 13, 56, 12, 15, 12, 4, 13},
    {6658,  8,  2, 7, 3, 14, 13, 53, 12, 15, 12, 4, 13},
    {6597,  8,  2, 7, 2, 13, 13, 54, 12, 15, 12, 4, 13},
    {6539,  8,  2, 7, 2, 13, 13, 55, 12, 15, 12, 4, 13},
    {6482,  8,  2, 7, 2, 13, 13, 56, 12, 15, 12, 4, 13},
    {6427,  7,  2, 7, 3, 14, 13, 54, 12, 15, 12, 4, 13},
    {6373,  7,  2, 7, 3, 15, 13, 54, 12, 15, 12, 4, 13},
    {6321,  7,  2, 7, 3, 15, 13, 55, 12, 15, 12, 4, 13},
    {6270,  7,  2, 7, 3, 15, 13, 56, 12, 15, 12, 4, 13},
    {6222,  7,  2, 7, 3, 15, 13, 57, 12, 15, 12, 4, 13},
    {6174,  6,  2, 7, 3, 15, 13, 54, 12, 15, 12, 4, 13},
    {6128,  6,  2, 7, 3, 15, 13, 55, 12, 15, 12, 4, 13},
    {6083,  6,  2, 7, 3, 15, 13, 56, 12, 15, 12, 4, 13},
    {6040,  6,  2, 7, 3, 15, 13, 57, 12, 15, 12, 4, 13},
    {5997,  6,  2, 7, 3, 15, 13, 58, 12, 15, 12, 4, 13},
};

/*
 * Self-contained unsigned 64-by-32 division via restoring binary long
 * division (64 fixed shift/compare/subtract iterations -- negligible
 * cost, this only runs a couple of times per retune). Needed because
 * sw/build.bat links with `arm-none-eabi-ld` directly, not through gcc,
 * so libgcc's __aeabi_uidiv/__aeabi_uldivmod are NOT linked in -- a
 * plain `/` or `%` anywhere in this file (even 32-bit, even at compile-
 * time-constant divisors, since -O0 skips the strength-reduction that
 * would otherwise avoid the library call) would fail to link. This
 * routine uses only shifts/compares/subtracts, which the compiler
 * always emits as native instruction sequences, never a library call.
 */
static uint32_t ad9361_udiv64_32(uint64_t num, uint32_t den, uint32_t *rem_out)
{
    uint64_t remainder = 0;
    uint64_t quotient = 0;
    for (int32_t i = 63; i >= 0; i--) {
        remainder = (remainder << 1) | ((num >> i) & 1u);
        quotient <<= 1;
        if (remainder >= den) {
            remainder -= den;
            quotient |= 1u;
        }
    }
    if (rem_out) { *rem_out = (uint32_t)remainder; }
    return (uint32_t)quotient;
}

static void ad9361_rx_lo_synth_set(uint32_t freq_hz)
{
    if (freq_hz < AD9361_RX_LO_MIN_HZ || freq_hz > AD9361_RX_LO_MAX_HZ) {
        return;
    }

    /* ad9361_calc_rfpll_int_divder(): double the target until it clears
     * the 6GHz VCO floor, tracking the divider count. */
    uint64_t vco_hz = (uint64_t)freq_hz;
    int32_t vco_div = -1;
    while (vco_hz <= AD9361_MIN_VCO_FREQ_HZ) {
        vco_hz <<= 1;
        vco_div++;
    }

    uint32_t rem;
    uint32_t integer = ad9361_udiv64_32(vco_hz, AD9361_REFCLK_HZ, &rem);
    uint64_t fract_num = (uint64_t)rem * (uint64_t)AD9361_RFPLL_MODULUS
                          + (uint64_t)(AD9361_REFCLK_HZ >> 1); /* round to nearest */
    uint32_t fract = ad9361_udiv64_32(fract_num, AD9361_REFCLK_HZ, NULL);

    /* ad9361_rfpll_vco_init(): vco_freq in MHz (truncating, matching the
     * reference driver) picks the LUT row. */
    uint32_t vco_mhz = ad9361_udiv64_32(vco_hz, 1000000UL, NULL);
    uint32_t idx = 0;
    while (idx < 52u && (uint32_t)ad9361_vco_lut_40mhz[idx].vco_mhz > vco_mhz) {
        idx++;
    }
    const struct ad9361_vco_lut_row *row = &ad9361_vco_lut_40mhz[idx];

    ad9361_spi_write(AD9361_REG_RX_VCO_OUTPUT,
                      (uint8_t)((row->output_level & 0xFu) | 0x40u)); /* PORB_VCO_LOGIC */
    ad9361_spi_rmw(AD9361_REG_RX_ALC_VARACTOR, 0xFu, 0u, row->varactor);
    ad9361_spi_write(AD9361_REG_RX_VCO_BIAS_1,
                      (uint8_t)(((row->bias_tcf & 0x3u) << 3) | (row->bias_ref & 0x7u)));
    ad9361_spi_write(AD9361_REG_RX_FORCE_VCO_TUNE_1,
                      (uint8_t)((row->cal_offset & 0xFu) << 3));
    ad9361_spi_write(AD9361_REG_RX_VCO_VARACTOR_CTRL_1, (uint8_t)(row->varactor_ref & 0xFu));
    ad9361_spi_write(AD9361_REG_RX_VCO_CAL_REF, 0x00u);
    ad9361_spi_write(AD9361_REG_RX_VCO_VARACTOR_CTRL_0, 0x70u);
    ad9361_spi_rmw(AD9361_REG_RX_CP_CURRENT, 0x3Fu, 0u, row->cp_current);
    ad9361_spi_write(AD9361_REG_RX_LOOP_FILTER_1,
                      (uint8_t)(((row->lf_c2 & 0xFu) << 4) | (row->lf_c1 & 0xFu)));
    ad9361_spi_write(AD9361_REG_RX_LOOP_FILTER_2,
                      (uint8_t)(((row->lf_r1 & 0xFu) << 4) | (row->lf_c3 & 0xFu)));
    ad9361_spi_write(AD9361_REG_RX_LOOP_FILTER_3, (uint8_t)(row->lf_r3 & 0xFu));

    ad9361_spi_write(AD9361_REG_RX_FRACT_BYTE_2, (uint8_t)((fract >> 16) & 0x7Fu));
    ad9361_spi_write(AD9361_REG_RX_FRACT_BYTE_1, (uint8_t)((fract >> 8) & 0xFFu));
    ad9361_spi_write(AD9361_REG_RX_FRACT_BYTE_0, (uint8_t)(fract & 0xFFu));
    ad9361_spi_rmw(AD9361_REG_RX_INTEGER_BYTE_1, 0x7u, 0u, (uint8_t)((integer >> 8) & 0x7u));
    ad9361_spi_write(AD9361_REG_RX_INTEGER_BYTE_0, (uint8_t)(integer & 0xFFu));
    ad9361_spi_rmw(AD9361_REG_RFPLL_DIVIDERS, 0xFu, 0u, (uint8_t)vco_div);

    for (uint32_t tries = 0; tries < 100u; tries++) {
        if (ad9361_spi_read(AD9361_REG_RX_CP_OVERRANGE_VCO_LOCK) & AD9361_VCO_LOCK) {
            break;
        }
        delay(1000u);
    }
}

/*
 * One-time load of the internal RX gain table (RX1 only -- RX2 is never
 * connected/used in this design). Common to both AGC and MGC, since both
 * just index into this same table -- doesn't need re-running when the gain
 * mode is switched, only if the RX LO ever moved to a different ADI gain-
 * table band (never happens here, fixed at 98MHz). See the register block
 * above for the full derivation.
 */
static void ad9361_rx_gain_table_load(void)
{
    ad9361_spi_write(AD9361_REG_AGC_CONFIG_2, AD9361_AGC_CONFIG_2_VAL);
    ad9361_spi_write(AD9361_REG_MAX_LMT_FULL_GAIN, (uint8_t)(AD9361_GAIN_TABLE_SIZE - 1u));

    ad9361_spi_write(AD9361_REG_GAIN_TABLE_CONFIG,
                      AD9361_START_GAIN_TABLE_CLOCK | AD9361_RECEIVER_SELECT_RX1);
    for (uint32_t i = 0; i < AD9361_GAIN_TABLE_SIZE; i++) {
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_ADDRESS, (uint8_t)i);
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_WRITE_DATA1, ad9361_gain_table_200_1300mhz[i][0]);
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_WRITE_DATA2, ad9361_gain_table_200_1300mhz[i][1]);
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_WRITE_DATA3, ad9361_gain_table_200_1300mhz[i][2]);
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_CONFIG,
                          AD9361_START_GAIN_TABLE_CLOCK | AD9361_WRITE_GAIN_TABLE |
                          AD9361_RECEIVER_SELECT_RX1);
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_READ_DATA1, 0u); /* dummy write, delay */
        ad9361_spi_write(AD9361_REG_GAIN_TABLE_READ_DATA1, 0u); /* dummy write, delay */
    }
    ad9361_spi_write(AD9361_REG_GAIN_TABLE_CONFIG,
                      AD9361_START_GAIN_TABLE_CLOCK | AD9361_RECEIVER_SELECT_RX1); /* clear write bit */
    ad9361_spi_write(AD9361_REG_GAIN_TABLE_READ_DATA1, 0u); /* dummy write, delay */
    ad9361_spi_write(AD9361_REG_GAIN_TABLE_READ_DATA1, 0u); /* dummy write, delay */
    ad9361_spi_write(AD9361_REG_GAIN_TABLE_CONFIG, 0u); /* stop gain table clock */

    ad9361_spi_write(AD9361_REG_RX1_MANUAL_DIGITALFORCED_GAIN, 0u); /* digital gain index unused */
}

/*
 * RX gain control mode select: 0=MGC (manual), 1=fast-attack AGC, 2=slow-
 * attack AGC, 3=hybrid AGC (ADI's rf_gain_ctrl_mode enum). RX2 mirrors
 * RX1's mode (RX2 unused). Only the mode-select bits are touched here --
 * a real tuned AGC response also needs step-size/overload-threshold
 * registers (ADI's ad9361_gc_setup()) this project has never configured,
 * so picking an AGC mode selects the *behavior* but not a calibrated one;
 * that's a separate, bigger one-time setup if ever needed. Sufficient on
 * its own for manual gain testing and for toggling back to whatever AGC
 * mode was last selected.
 */
static void ad9361_set_rx_gain_mode(uint8_t mode)
{
    uint8_t m = mode & 0x3u;
    ad9361_spi_write(AD9361_REG_AGC_CONFIG_1, (uint8_t)(m | (uint8_t)(m << 2)));
}

/*
 * Manual gain table index, 0-76 (~ -1dB..73dB for this band). Only takes
 * effect while in MGC mode -- ADI's own driver (ad9361_set_rx_gain())
 * refuses this write outside MGC, since the AGC engine drives the same
 * index register itself in every other mode.
 */
static void ad9361_set_rx_manual_gain(uint8_t idx)
{
    if (idx > (uint8_t)(AD9361_GAIN_TABLE_SIZE - 1u)) { idx = (uint8_t)(AD9361_GAIN_TABLE_SIZE - 1u); }
    ad9361_spi_rmw(AD9361_REG_RX1_MANUAL_LMT_FULL_GAIN, 0x7Fu, 0u, idx);
}

/*
 * One-time AD9361 bring-up shared by test and mission mode: release the
 * control pins, lock the BBPLL, configure the LVDS parallel port, lock
 * the RX LO synth (98MHz), set the RX sample-rate chain, load the RX gain
 * table and default to manual gain, tune RX_DATA_DELAY, force ALERT->RX.
 * Runs once at boot.
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

    ad9361_spi_write(AD9361_REG_INPUT_SELECT, AD9361_INPUT_SELECT_RX1A_RX2A_BALANCED);

    ad9361_rx_lo_synth_98mhz();

    /* RX sample-rate chain: target RX_SAMPL_FREQ (= dsp_clk, confirmed
     * 1:1 -- see README.md) = 30MHz, RX FIR bypassed. BBPLL_DIVIDER=5
     * -> ADC_CLK = 960MHz>>5 = 30MHz exactly; DEC3/RHB2_EN/RHB1_EN/
     * RX_FIR_ENABLE_DECIMATION all cleared (0x40 preserves only
     * RX_CHANNEL_ENABLE=RX_1) so nothing downstream of ADC_CLK divides
     * further. Previously left as a manual UART knob (never applied),
     * which is why dsp_clk had been running at the chip's unconfigured
     * default (~7.5MHz) the whole time -- see project history for how
     * that was diagnosed. Hardware-confirmed 2026-09-13 via the CIC-
     * decimated sample-stream rate (rate_counter.py): dsp_clk lands
     * dead-on 30MHz (9.6MB/s) -- an earlier reading that session
     * (~28.23MHz-equivalent, 9.035MB/s) turned out to be measured after
     * a string of live-ELF-only JTAG reloads that never actually reset
     * the AD9361 (REG_AD9361_PINS/RESETB is PL-fabric state, untouched
     * by dow+con -- see project memory), not a genuine BBPLL/REFCLK
     * tolerance limit. Trust dsp_clk/rate measurements only after a real
     * bitstream reprogram or power cycle. constraints.xdc's rx_clk
     * period (33.33ns/30MHz) matches this confirmed rate directly, not
     * just as conservative margin. Note RX_DATA_DELAY below was
     * originally hand-tuned at the old, wrong sample rate -- may need
     * re-sweeping now that dsp_clk actually changes by ~4x. */
    ad9361_spi_rmw(AD9361_REG_BBPLL, 0x7u, 0u, 5u);
    ad9361_spi_write(AD9361_REG_RX_ENABLE_FILTER_CTRL, 0x40u);

    ad9361_rx_gain_table_load();
    ad9361_set_rx_gain_mode(0u); /* default: manual gain, so mission mode boots to a
                                   * known state rather than an unconfigured/undefined
                                   * AGC response -- override live via CMD_DEV_GAIN */
    ad9361_set_rx_manual_gain(60u); /* moderate default (~57dB); live-adjustable */

    ad9361_spi_write(AD9361_REG_RX_CLOCK_DATA_DELAY, AD9361_RX_DATA_DELAY_DEFAULT);
    ad9361_spi_write(AD9361_REG_ENSM_CONFIG_1,
                      AD9361_LEVEL_MODE | AD9361_TO_ALERT | AD9361_FORCE_RX_ON); /* ALERT -> real RX */

    /* MGC settle pulse -- see the register block above for why. */
    {
        uint8_t tmp = ad9361_spi_read(AD9361_REG_SMALL_LMT_OVERLOAD_THRESH);
        ad9361_spi_write(AD9361_REG_SMALL_LMT_OVERLOAD_THRESH,
                          (tmp & AD9361_SMALL_LMT_OVERLOAD_THRESH_MASK) | AD9361_FORCE_PD_RESET_RX1);
        ad9361_spi_write(AD9361_REG_SMALL_LMT_OVERLOAD_THRESH,
                          tmp & AD9361_SMALL_LMT_OVERLOAD_THRESH_MASK);
    }
}

/* Set once test mode is actually active (and cleared on mission mode) --
 * gates set_test_pattern_prbs()/set_test_pattern_tone() below, so a stray
 * SYS_MODE_TEST_PATTERN_* command can't inject a synthetic BIST pattern
 * over a real antenna signal while in mission mode. */
static uint8_t g_test_mode_active = 0u;

/*
 * Test mode: chip is already in real RX state (ad9361_common_init()
 * forces ALERT->RX at boot). Only remaining step: enable the RX-side
 * BIST pattern generator (BIST_CTRL_POINT=2=RX injection), defaulting to
 * PRBS -- see set_test_pattern_prbs()/set_test_pattern_tone() below to
 * switch the pattern afterward.
 */
static void enter_test_mode(void)
{
    ad9361_spi_write(AD9361_REG_OBSERVE_CONFIG, 0u);
    ad9361_spi_write(AD9361_REG_BIST_CONFIG,
                      AD9361_BIST_CTRL_POINT_RX(2u) | AD9361_BIST_ENABLE);
    g_test_mode_active = 1u;
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
    g_test_mode_active = 0u;
}

/*
 * Test-mode-only pattern select for the RX BIST generator, no-op outside
 * test mode. prbs() restores the default pseudo-random sequence (see
 * enter_test_mode()). tone() switches to a full-scale ~931kHz sine
 * (RX_SAMPL_CLK/32, TONE_FREQ code 0) -- inside the sample-stream FFT's
 * ~3.725MHz Nyquist edge, so it shows up as a clean known-frequency
 * spike. Traced from ADI's ad9361_bist_tone() (private/docs/.../ad9361.c).
 */
static void set_test_pattern_prbs(void)
{
    if (!g_test_mode_active) { return; }
    ad9361_spi_write(AD9361_REG_BIST_CONFIG,
                      AD9361_BIST_CTRL_POINT_RX(2u) | AD9361_BIST_ENABLE);
}

static void set_test_pattern_tone(void)
{
    if (!g_test_mode_active) { return; }
    ad9361_spi_write(AD9361_REG_BIST_AND_DATA_PORT_TEST_CONFIG, 0u);
    ad9361_spi_write(AD9361_REG_BIST_CONFIG,
                      AD9361_BIST_CTRL_POINT_RX(2u) | AD9361_BIST_ENABLE |
                      AD9361_TONE_PRBS | AD9361_TONE_LEVEL(0u) | AD9361_TONE_FREQ(0u));
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
    } else if (dev == CMD_DEV_RXLO) {
        if (rw & CMD_RW_WRITE) {
            ad9361_rx_lo_synth_set(data);
        }
    } else if (dev == CMD_DEV_GAIN) {
        if (rw & CMD_RW_WRITE) {
            if (addr == 0x00u) {
                ad9361_set_rx_gain_mode((uint8_t)(data & 0xFFu));
            } else if (addr == 0x04u) {
                ad9361_set_rx_manual_gain((uint8_t)(data & 0xFFu));
            }
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
            } else if (mode == SYS_MODE_TEST_PATTERN_PRBS) {
                set_test_pattern_prbs();
            } else if (mode == SYS_MODE_TEST_PATTERN_TONE) {
                set_test_pattern_tone();
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
        eth_udp_reply_send(req_frame, reply_batch, reply_bytes);
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
     * every iteration alongside the others. eth_tx_recover() is the same
     * kind of cheap check -- detects and clears a halted GEM TX DMA (see
     * its own header comment in eth0.h) before it can silently kill both
     * UDP command replies and sample-stream packets for good. */
    for (;;) {
        if (uart1_available()) {
            process_uart_command();
        }
        eth_service();
        eth_poll_sample_stream();
        eth_tx_recover();
        eth_arp_retry_poll();
        eth_udp_retry_poll();
    }
}
