/* Small convention: phy_* for MDIO, gem_* for MAC/DMA, eth_* for the frame-building layer */
#include <stddef.h>
#include "eth0.h"

/* Shared TX ring producer index -- the only slot software will arm next.
 * Both eth_send_test_frame() and any future sender (a real UDP reply, or
 * a PL-fed sample producer) must go through eth_tx_reserve()/commit()
 * rather than touching descriptors directly, so there's exactly one
 * place that decides "which slot is next" for the whole ring. */
static uint32_t tx_next = 0u;

/* RX ring consumer index -- the only slot eth_rx_poll()/eth_rx_release()
 * ever look at, mirroring tx_next's role on the TX side. */
static uint32_t rx_next = 0u;

/* Between a eth_udp_reply_reserve() call and its matching commit() --
 * safe as file-local state since this whole stack is polled from one
 * single-threaded main loop, never re-entered. */
static uint8_t *udp_reply_buf = NULL;

/* Matches GEM_SPEC_ADDR1_BOT/TOP below (02:00:de:ad:be:ef) -- kept as one
 * array so eth_send_test_frame(), eth_send_arp_reply(), and
 * eth_udp_reply_reserve() can't drift apart on the station's own MAC. */
static const uint8_t g_board_mac[6] = { 0x02u, 0x00u, 0xdeu, 0xadu, 0xbeu, 0xefu };

#define GEM_CLKACT   0x1u
#define GEM_SRCSEL   0x1u
/* RGMII TXC needs 125MHz for gigabit (link confirmed 1000Mbps via
 * phy_get_link_status()). Source is the 1000MHz IO PLL
 * (PCW_IO_IO_PLL_FREQMHZ, ps7_parameters.xml):
 * 1000MHz / (DIVISOR * DIVISOR1) = 125MHz -> DIVISOR=8, DIVISOR1=1. */
#define GEM_DIVISOR  0x8u
#define GEM_DIVISOR1 0x1u
/* pclk (GEM's own AHB/APB register clock, what MDCCLKDIV divides down) is
 * CPU_1x, confirmed 133.333MHz on this board from ps7_parameters.xml
 * (PCW_TTC0_CLK0_PERIPHERAL_FREQMHZ, which is CPU_1x-sourced) -- this is a
 * DIFFERENT clock than GEM0_CLK_CTRL's IO-PLL-derived RGMII data clock
 * above, don't conflate the two. MDC must stay <=2.5MHz (double check
 * against docs/RTL8211F_datasheet.pdf), so the divide ratio needs to be
 * >= 133.33/2.5 ~= 53.3.
 * MDCCLKDIV is a 3-bit *enum* field, not a raw divisor -- you can't write
 * an arbitrary number like 40 into it. TODO: verify this code table
 * against UG585's NWCFG description before trusting it (from memory, not
 * re-derived from this project's own TRM copy yet):
 *   000=/8 001=/16 010=/32 011=/48 100=/64 101=/96 110=/128 111=/224
 * code 100 (0x4) -> /64 -> MDC = 133.33/64 ~= 2.08MHz, under the limit. */
#define GEM_MDC_DIVISOR  0x4u /* TODO: verify against UG585 before trusting */

void gem_slcr_setup(void){
    GEM_SLCR_UNLOCK    = 0xDF0Du;

#if 0
    /* GEM0 reset pulse (SLCR GEM_RST_CTRL) -- disabled: reproducibly hung
     * the CPU/DAP on a genuine cold boot (JTAG halt timed out, UART
     * unresponsive). Not confirmed to be this pulse specifically vs. a
     * missing bitstream reprogram on the same test run -- see
     * eth0_mdio_bringup_status.md before re-enabling. */
    GEM_RST_CTRL = GEM_RST_CTRL | GEM_RST_CTRL_MASK;
    { volatile uint32_t i; for (i = 0; i < 100u; i++) { } } /* reset pulse width */
    GEM_RST_CTRL = GEM_RST_CTRL & ~GEM_RST_CTRL_MASK;
#endif

    /* Clean disable -> reprogram -> re-enable, not a live divisor change:
     * CLKACT was already 1 at reset, so writing SRCSEL/DIVISOR/DIVISOR1
     * in the same write that keeps CLKACT=1 would change the divide
     * ratio while the divided clock is still toggling -- a classic way
     * to glitch or stall a clock divider's internal counter. */
    GEM_SLCR_CLK_CTRL  = (GEM_SRCSEL << 4) | (GEM_DIVISOR << 8) | (GEM_DIVISOR1 << 20); /* CLKACT=0 */
    GEM_SLCR_CLK_CTRL  = GEM_SLCR_CLK_CTRL | GEM_CLKACT; /* now enable, ratio already settled */
    GEM_SLCR_RCLK_CTRL = GEM_SLCR_RCLK_CTRL | GEM_CLKACT;
    GEM_SLCR_LOCK      = 0x767Bu;
}

void gem_slcr_lock(void){
    GEM_SLCR_LOCK    = 0x767Bu;
}

void gem_setup(void){
    /* UG585 16.3.1 "Initialize the Controller" -- start from a
     * guaranteed-known state before configuring anything. Statistics-
     * register addresses aren't in this project's reference material
     * yet, so that sub-step is skipped; the other four are done. */
    GEM_NWCTRL  = 0x0u;             /* 1. clear NWCTRL */
    GEM_RXSR    = 0x0Fu;            /* 3. clear status regs (write-1-clear) */
    GEM_TXSR    = 0xFFu;
    GEM_IDR     = 0xFFFFFFFFu;      /* 4. disable all interrupts */
    GEM_RXQBASE = 0x0u;             /* 5. clear rx_qbar/tx_qbar */
    GEM_TXQBASE = 0x0u;

    GEM_NWCFG  = GEM_NWCFG  | (GEM_MDC_DIVISOR << 18);
    GEM_NWCTRL = GEM_NWCTRL | GEM_NWCTRL_MDEN;

    GEM_NWCFG  = GEM_NWCFG  | (0x1u << 10) | (0x1u << 1); /* gigabit and full duplex */

    /* UG585 16.3.2 "Configure the Controller" -- DMACR was never written
     * before (reset default 0x00020784), missing two things:
     * - bit7 ahb_endian_swp_pkt_en resets to 1 (swapped); the TRM's own
     *   worked example (step 3e) says write 0 for a little-endian
     *   system, which this ARM CPU is.
     * - bits[4:0] BLENGTH reset default 00100 requests INCR4 AHB bursts.
     *   Forced to 00001 (SINGLE), the one burst type every AHB
     *   interconnect must support -- AHB has no error/timeout path for a
     *   burst type the interconnect won't grant, so requesting one this
     *   fabric doesn't support could hang the request phase silently. */
    GEM_DMACR = (GEM_DMACR & ~(0x1u << 7) & ~0x1Fu  & ~(0xFFu << 16)) | 0x1u | 4u<<16;

    /* Station address (RX-filtering only, confirmed via UG585 16.2.3 --
     * has no bearing on TX, but never set at all until now, and needed
     * eventually for Step 4's RX ring). BOT = octets 0-3 as a
     * little-endian 32-bit word, TOP = octets 4-5 in the low 16 bits --
     * per the TRM's own worked example, no byte-swap needed on this
     * little-endian CPU. MAC 02:00:de:ad:be:ef -> BOT=0xADDE0002,
     * TOP=0x0000EFBE. */
    GEM_SPEC_ADDR1_BOT = 0xADDE0002u;
    GEM_SPEC_ADDR1_TOP = 0x0000EFBEu;

    /* Ring init: every slot starts USED=1 (not armed, safe to leave
     * dormant) -- LAST_BUFFER/length don't matter while USED=1, only set
     * for real when a slot is actually armed via eth_tx_commit(). WRAP=1
     * only on the last slot, closing the ring back to slot 0. */
    {
        uint32_t i;
        for (i = 0u; i < GEM_TX_RING_SIZE; i++) {
            uint32_t wrap = (i == GEM_TX_RING_SIZE - 1u) ? (1u << 30) : 0u;
            GEM_DESC_BUF(i)   = FRAME_BASE_ADDR + i * sizeof(eth_frame);
            GEM_DESC_FLAGS(i) = (1u << 31) | wrap;
        }
    }
    /* Replicating the same structure for RX descriptor ring*/
    {
        uint32_t i;
        for (i = 0u; i < GEM_RX_RING_SIZE; i++) {
            uint32_t wrap = (i == GEM_RX_RING_SIZE - 1u) ? (1u << 1) : 0u;
            GEM_DESC_BUF_RX(i) = (GEM_RX_BUF_BASE + i * GEM_RX_BUF_STRIDE) | wrap;
        }
    }

    GEM_TXQBASE = GEM_DESCRIPTOR_TX;
    GEM_RXQBASE = GEM_DESCRIPTOR_RX;

    /* Both rings and both queue bases are fully written by this point
     * (Normal Cacheable DDR stores) -- this barrier makes sure they've
     * actually reached DDR before RXEN goes live below and GEM starts
     * consuming the RX ring on its own, unprompted by any software kick
     * (unlike TX, which only ever gets touched right after
     * eth_tx_commit()'s own dsb). */
    __asm__ volatile ("dsb" ::: "memory");
    GEM_NWCTRL  = GEM_NWCTRL | GEM_NWCTRL_TXEN | GEM_NWCTRL_RXEN;
}

uint32_t phy_query_mdio(uint32_t addr){
    while (!(GEM_NWSR & 0x04)){}
    return (*(volatile uint32_t *) (GEM_CORE_BASE + addr));
}

void phy_write_mdio(uint32_t addr, uint32_t data){
    while (!(GEM_NWSR & 0x04)){}
    (*(volatile uint32_t *) (GEM_CORE_BASE + addr)) = data;
}

uint32_t phy_get_rtl_identifier(void){
    uint32_t id = 0;
    phy_write_mdio(0x34, 0x600a << 16);
    id = (phy_query_mdio(0x34) & 0xffff) << 16;
    phy_write_mdio(0x34, 0x600e << 16);
    id += (phy_query_mdio(0x34) & 0xffff);

    return id;
}

/*
 * RTL8211F PHYSR (PHY Specific Status Register, MDIO reg 26) -- only
 * valid while extended page 0xa43 is selected via PAGSR (reg 31); the
 * PHY defaults to page 0xa42 after reset/link changes, so this switches
 * to 0xa43, reads PHYSR, then switches back before returning, per the
 * datasheet ("PHYSR is only valid while page 0xa43 is selected").
 * PHYSR bits[5:4]=speed(10=1000M,01=100M,00=10M), bit3=duplex(1=full),
 * bit2=link(real-time, 1=up).
 */
uint32_t phy_get_link_status(void){
    phy_write_mdio(0x34, 0x507e0a43); /* PHY addr 0, write reg 31 (PAGSR) = 0xa43 */
    phy_write_mdio(0x34, 0x606a0000); /* PHY addr 0, request read of reg 26 (PHYSR) */
    uint32_t physr = phy_query_mdio(0x34) & 0xffff;
    phy_write_mdio(0x34, 0x507e0a42); /* restore default page 0xa42 */
    return physr;
}

/*
 * Reserve the next TX ring slot for filling. Returns a pointer to that
 * slot's buffer if GEM is done with it (USED==1), or NULL if GEM hasn't
 * drained it yet (ring full). No blocking, no retry -- for this
 * application, failing to keep the ring ahead of GEM means dropping
 * whatever didn't fit, not stalling the producer.
 */
void *eth_tx_reserve(void){
    if (!(GEM_DESC_FLAGS(tx_next) & (1u << 31))) {
        return NULL;
    }
    return (void *)(FRAME_BASE_ADDR + tx_next * sizeof(eth_frame));
}

/*
 * Arm the slot handed out by the last eth_tx_reserve() call with `len`
 * bytes already written into it, and kick GEM. WRAP is recomputed fresh
 * from the ring position rather than preserved from the dormant init
 * value -- cheap, and avoids relying on a read-modify-write here.
 */
void eth_tx_commit(uint16_t len){
    uint32_t wrap = (tx_next == GEM_TX_RING_SIZE - 1u) ? (1u << 30) : 0u;
    GEM_DESC_FLAGS(tx_next) = (0u << 31) | wrap | (1u << 15) | len;
    __asm__ volatile ("dsb" ::: "memory"); /* descriptor write must reach DDR before GEM is kicked */
    GEM_NWCTRL = GEM_NWCTRL | (0x1u << 9);
    tx_next = (tx_next + 1u) % GEM_TX_RING_SIZE;
}

void eth_send_test_frame(void){
    eth_frame *frame = (eth_frame *) eth_tx_reserve();
    if (frame == NULL) {
        return;
    }

    frame->des_mac[0] = 0xff; frame->des_mac[1] = 0xff; frame->des_mac[2] = 0xff;
    frame->des_mac[3] = 0xff; frame->des_mac[4] = 0xff; frame->des_mac[5] = 0xff;

    /* locally administered (bit1 of the first octet set), matches
     * GEM_SPEC_ADDR1_BOT/TOP in gem_setup() */
    for (int i = 0; i < 6; i++) { frame->src_mac[i] = g_board_mac[i]; }

    frame->ether_type[0] = 0x08;
    frame->ether_type[1] = 0x00;

    frame->payload[0] = 0x5a;
    frame->payload[1] = 0x5a;
    frame->payload[2] = 0x0f;
    frame->payload[3] = 0x0f;

    eth_tx_commit(sizeof(eth_frame));
}

/*
 * Poll the current RX slot's ownership bit (word0 bit0, NEW) without
 * blocking. NEW==1 means hardware already wrote a frame here and handed
 * it back to software; NEW==0 means it's still hardware's turn. Never
 * touches rx_next -- only eth_rx_release() advances the ring, so a poll
 * can be called repeatedly with no side effects until the caller is done
 * with the buffer.
 */
void *eth_rx_poll(uint16_t *len_out)
{
    uint32_t word0 = GEM_DESC_BUF_RX(rx_next);
    if (!(word0 & 0x1u)) {
        return NULL;
    }
    if (len_out != NULL) {
        uint32_t word1 = GEM_DESC_FLAGS_RX(rx_next);
        *len_out = (uint16_t)(word1 & 0x1FFFu);
    }
    return (void *)(GEM_RX_BUF_BASE + rx_next * GEM_RX_BUF_STRIDE);
}

/*
 * Hand the current RX slot back to hardware (NEW=0, recomputed fresh from
 * the ring position rather than preserved -- same reasoning as
 * eth_tx_commit()'s WRAP handling) and advance to the next slot. Call
 * exactly once per successful eth_rx_poll(), after its buffer is no
 * longer needed.
 */
void eth_rx_release(void)
{
    uint32_t wrap = (rx_next == GEM_RX_RING_SIZE - 1u) ? (1u << 1) : 0u;
    GEM_DESC_BUF_RX(rx_next) = ((GEM_RX_BUF_BASE + rx_next * GEM_RX_BUF_STRIDE) & ~0x3u) | wrap;
    rx_next = (rx_next + 1u) % GEM_RX_RING_SIZE;
}

/*
 * Standard IPv4/UDP one's-complement checksum: sum 16-bit big-endian
 * words, fold any carry out of bit16 back in, then complement. Caller
 * must zero the checksum field itself in `hdr` before calling.
 */
static uint16_t ip_checksum(const uint8_t *hdr, uint32_t len)
{
    uint32_t sum = 0u;
    uint32_t i;
    for (i = 0u; i + 1u < len; i += 2u) {
        sum += ((uint32_t)hdr[i] << 8) | hdr[i + 1u];
    }
    if (len & 1u) {
        sum += (uint32_t)hdr[len - 1u] << 8;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    return (uint16_t)(~sum & 0xFFFFu);
}

/*
 * Byte offsets below (0-5 dest MAC, 6-11 src MAC, 12-13 EtherType, then
 * the ARP payload from 14) match the standard Ethernet+ARP wire layout,
 * cross-checked against the real captured frame decoded by hand during
 * Step 4's milestone (that one was IPv4/UDP, but the Ethernet header
 * portion -- offsets 0-13 -- is identical either way).
 */
void eth_send_arp_reply(const uint8_t *req)
{
    uint8_t *buf = (uint8_t *)eth_tx_reserve();
    uint32_t i;
    if (buf == NULL) {
        return;
    }

    for (i = 0u; i < 6u; i++) { buf[i] = req[6u + i]; }         /* dest MAC = requester's src MAC */
    for (i = 0u; i < 6u; i++) { buf[6u + i] = g_board_mac[i]; } /* src MAC = board */
    buf[12] = 0x08u; buf[13] = 0x06u;                           /* EtherType = ARP */

    buf[14] = 0x00u; buf[15] = 0x01u; /* HTYPE = Ethernet */
    buf[16] = 0x08u; buf[17] = 0x00u; /* PTYPE = IPv4 */
    buf[18] = 6u;                     /* HLEN */
    buf[19] = 4u;                     /* PLEN */
    buf[20] = 0x00u; buf[21] = 0x02u; /* OPER = reply */
    for (i = 0u; i < 6u; i++) { buf[22u + i] = g_board_mac[i]; } /* SHA = board MAC */
    buf[28] = BOARD_IP0; buf[29] = BOARD_IP1; buf[30] = BOARD_IP2; buf[31] = BOARD_IP3; /* SPA = board IP */
    for (i = 0u; i < 6u; i++) { buf[32u + i] = req[22u + i]; }  /* THA = requester's SHA */
    for (i = 0u; i < 4u; i++) { buf[38u + i] = req[28u + i]; }  /* TPA = requester's SPA */

    /* GEM does not auto-pad short TX frames -- 42 raw bytes here is well
     * under the 60-byte Ethernet minimum. */
    for (i = 42u; i < ETH_MIN_FRAME_LEN; i++) { buf[i] = 0u; }

    eth_tx_commit(ETH_MIN_FRAME_LEN);
}

/*
 * Reserve a TX slot and pre-fill every UDP reply header field that
 * doesn't depend on payload length -- src/dst MAC (unicast back to the
 * requester), EtherType, IP version/IHL/TTL/protocol, src/dst IP,
 * src/dst UDP port. Total length and IP checksum are filled in by
 * eth_udp_reply_commit() once the caller knows how much payload it wrote
 * starting at the returned pointer.
 */
void *eth_udp_reply_reserve(const uint8_t *req)
{
    uint8_t *buf = (uint8_t *)eth_tx_reserve();
    uint32_t i;
    if (buf == NULL) {
        return NULL;
    }

    for (i = 0u; i < 6u; i++) { buf[i] = req[6u + i]; }         /* dest MAC = requester's src MAC */
    for (i = 0u; i < 6u; i++) { buf[6u + i] = g_board_mac[i]; } /* src MAC = board */
    buf[12] = 0x08u; buf[13] = 0x00u;                           /* EtherType = IPv4 */

    buf[14] = 0x45u; /* version 4, IHL 5 (20-byte header, no options) */
    buf[15] = 0x00u; /* DSCP/ECN */
    buf[16] = 0x00u; buf[17] = 0x00u; /* total length -- filled in at commit */
    buf[18] = 0x00u; buf[19] = 0x00u; /* identification -- unfragmented, 0 is fine */
    buf[20] = 0x00u; buf[21] = 0x00u; /* flags/fragment offset */
    buf[22] = 64u;   /* TTL */
    buf[23] = 17u;   /* protocol = UDP */
    buf[24] = 0x00u; buf[25] = 0x00u; /* header checksum -- filled in at commit */
    buf[26] = BOARD_IP0; buf[27] = BOARD_IP1; buf[28] = BOARD_IP2; buf[29] = BOARD_IP3; /* src IP */
    for (i = 0u; i < 4u; i++) { buf[30u + i] = req[26u + i]; }  /* dest IP = requester's src IP */

    buf[34] = (uint8_t)(UDP_CMD_PORT >> 8); buf[35] = (uint8_t)UDP_CMD_PORT; /* src port */
    buf[36] = req[34]; buf[37] = req[35];                                   /* dest port = requester's src port */
    buf[38] = 0x00u; buf[39] = 0x00u; /* UDP length -- filled in at commit */
    buf[40] = 0x00u; buf[41] = 0x00u; /* UDP checksum = 0 (optional, per roadmap) */

    udp_reply_buf = buf;
    return (void *)(buf + ETH_UDP_HEADER_LEN);
}

/*
 * Finalize a UDP reply started by eth_udp_reply_reserve(): write the real
 * IP total length and UDP length now that payload_len is known, compute
 * the IP header checksum (mandatory -- unlike UDP's, which stays zero),
 * zero-pad up to the Ethernet minimum frame size if needed, and commit.
 */
/*
 * Test-only: hand-crafted ARP request "from" a fake requester
 * (00:11:22:33:44:55, 192.168.3.99) targeting this board's own IP,
 * written directly into the current RX slot's buffer. Deliberately
 * bypasses GEM entirely -- this is a software-only substitute for real
 * received traffic, letting eth_service()'s ARP path be exercised
 * on demand, immediately, and repeatably.
 */
void eth_test_inject_arp_request(void)
{
    uint8_t *buf = (uint8_t *)(GEM_RX_BUF_BASE + rx_next * GEM_RX_BUF_STRIDE);
    uint32_t i;
    uint32_t wrap;

    for (i = 0u; i < 6u; i++) { buf[i] = 0xFFu; }              /* dest MAC: broadcast */
    buf[6] = 0x00u; buf[7] = 0x11u; buf[8] = 0x22u;             /* src MAC: fake requester */
    buf[9] = 0x33u; buf[10] = 0x44u; buf[11] = 0x55u;
    buf[12] = 0x08u; buf[13] = 0x06u;                           /* EtherType = ARP */

    buf[14] = 0x00u; buf[15] = 0x01u; /* HTYPE */
    buf[16] = 0x08u; buf[17] = 0x00u; /* PTYPE */
    buf[18] = 6u; buf[19] = 4u;       /* HLEN/PLEN */
    buf[20] = 0x00u; buf[21] = 0x01u; /* OPER = request */
    buf[22] = 0x00u; buf[23] = 0x11u; buf[24] = 0x22u;          /* SHA = fake requester */
    buf[25] = 0x33u; buf[26] = 0x44u; buf[27] = 0x55u;
    buf[28] = 192u; buf[29] = 168u; buf[30] = 3u; buf[31] = 99u; /* SPA = fake requester IP */
    for (i = 0u; i < 6u; i++) { buf[32u + i] = 0x00u; }         /* THA: unknown, per spec */
    buf[38] = BOARD_IP0; buf[39] = BOARD_IP1; buf[40] = BOARD_IP2; buf[41] = BOARD_IP3; /* TPA = us */

    for (i = 42u; i < ETH_MIN_FRAME_LEN; i++) { buf[i] = 0u; }

    GEM_DESC_FLAGS_RX(rx_next) = (1u << 15) | (1u << 14) | ETH_MIN_FRAME_LEN; /* SOF, EOF, length */
    wrap = (rx_next == GEM_RX_RING_SIZE - 1u) ? (1u << 1) : 0u;
    __asm__ volatile ("dsb" ::: "memory");
    GEM_DESC_BUF_RX(rx_next) = ((GEM_RX_BUF_BASE + rx_next * GEM_RX_BUF_STRIDE) & ~0x3u) | wrap | 0x1u; /* NEW=1 */
}

void eth_udp_reply_commit(uint16_t payload_len)
{
    uint8_t *buf = udp_reply_buf;
    uint16_t ip_total_len = (uint16_t)(20u + 8u + payload_len);
    uint16_t udp_len = (uint16_t)(8u + payload_len);
    uint16_t total_frame_len = (uint16_t)(14u + ip_total_len);
    uint16_t checksum;

    buf[16] = (uint8_t)(ip_total_len >> 8); buf[17] = (uint8_t)ip_total_len;
    buf[38] = (uint8_t)(udp_len >> 8);      buf[39] = (uint8_t)udp_len;

    buf[24] = 0x00u; buf[25] = 0x00u; /* zero before summing, per the algorithm */
    checksum = ip_checksum(buf + 14, 20u);
    buf[24] = (uint8_t)(checksum >> 8); buf[25] = (uint8_t)checksum;

    if (total_frame_len < ETH_MIN_FRAME_LEN) {
        uint32_t i;
        for (i = total_frame_len; i < ETH_MIN_FRAME_LEN; i++) { buf[i] = 0u; }
        total_frame_len = ETH_MIN_FRAME_LEN;
    }

    eth_tx_commit(total_frame_len);
}