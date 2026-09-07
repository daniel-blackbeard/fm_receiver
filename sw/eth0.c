/* Small convention: phy_* for MDIO, gem_* for MAC/DMA, eth_* for the frame-building layer */
#include <stddef.h>
#include "eth0.h"

/* Shared TX ring producer index, in FRAME units (each frame = 2
 * descriptors at [tx_next*2, tx_next*2+1]) -- both single-buffer and
 * scatter-gather senders draw from this same rotating pool (see the
 * GEM_TX_RING_SIZE comment in eth0.h). */
static uint32_t tx_next = 0u;

/* RX ring consumer index, mirroring tx_next on the RX side. */
static uint32_t rx_next = 0u;

/* Between an eth_udp_reply_reserve() call and its matching commit() --
 * safe as file-local state since this stack is single-threaded, never
 * re-entered. */
static uint8_t *udp_reply_buf = NULL;

/* Matches GEM_SPEC_ADDR1_BOT/TOP below (02:00:de:ad:be:ef). */
static const uint8_t g_board_mac[6] = { 0x02u, 0x00u, 0xdeu, 0xadu, 0xbeu, 0xefu };

#define GEM_CLKACT   0x1u
#define GEM_SRCSEL   0x1u
/* RGMII TXC needs 125MHz for gigabit. Source is the 1000MHz IO PLL:
 * 1000MHz / (DIVISOR * DIVISOR1) = 125MHz -> DIVISOR=8, DIVISOR1=1. */
#define GEM_DIVISOR  0x8u
#define GEM_DIVISOR1 0x1u
/* pclk (CPU_1x, 133.333MHz) divided down by MDCCLKDIV, a 3-bit enum, not
 * a raw divisor. MDC must stay <=2.5MHz (RTL8211F_datasheet.pdf).
 *   000=/8 001=/16 010=/32 011=/48 100=/64 101=/96 110=/128 111=/224
 * code 100 (0x4) -> /64 -> MDC ~= 2.08MHz. TODO: verify against UG585. */
#define GEM_MDC_DIVISOR  0x4u

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
    /* UG585 16.3.1 "Initialize the Controller" -- known state before
     * configuring anything (statistics registers not used here). */
    GEM_NWCTRL  = 0x0u;             /* 1. clear NWCTRL */
    GEM_RXSR    = 0x0Fu;            /* 3. clear status regs (write-1-clear) */
    GEM_TXSR    = 0xFFu;
    GEM_IDR     = 0xFFFFFFFFu;      /* 4. disable all interrupts */
    GEM_RXQBASE = 0x0u;             /* 5. clear rx_qbar/tx_qbar */
    GEM_TXQBASE = 0x0u;

    GEM_NWCFG  = GEM_NWCFG  | (GEM_MDC_DIVISOR << 18);
    GEM_NWCTRL = GEM_NWCTRL | GEM_NWCTRL_MDEN;

    GEM_NWCFG  = GEM_NWCFG  | (0x1u << 10) | (0x1u << 1); /* gigabit and full duplex */

    /* UG585 16.3.2 "Configure the Controller". DMACR reset default
     * (0x00020784) needs two fixes: bit7 ahb_endian_swp_pkt_en -> 0 (this
     * CPU is little-endian); BLENGTH -> 00001 SINGLE (the one AHB burst
     * type every interconnect must support). */
    GEM_DMACR = (GEM_DMACR & ~(0x1u << 7) & ~0x1Fu  & ~(0xFFu << 16)) | 0x1u | 4u<<16;

    /* Station address (RX-filtering only, UG585 16.2.3). MAC
     * 02:00:de:ad:be:ef -> BOT=0xADDE0002, TOP=0x0000EFBE, no byte-swap
     * needed on this little-endian CPU. */
    GEM_SPEC_ADDR1_BOT = 0xADDE0002u;
    GEM_SPEC_ADDR1_TOP = 0x0000EFBEu;

    /* Ring init: GEM_TX_FRAME_SLOTS frames, 2 descriptors each -- both
     * start USED=1 (dormant, safe), only armed for real via
     * eth_tx_commit()/eth_tx_sg_commit(). Every slot is fully rewritten
     * (both descriptors) on every commit regardless of sender, since any
     * slot can hold a single-buffer frame's trailer one time and a
     * sample-stream payload the next -- see the GEM_TX_RING_SIZE comment
     * in eth0.h. WRAP=1 only on the true last physical descriptor. */
    {
        uint32_t i;
        for (i = 0u; i < GEM_TX_FRAME_SLOTS; i++) {
            uint32_t content_idx = i * 2u;
            uint32_t trailer_idx = content_idx + 1u;
            uint32_t trailer_wrap = (trailer_idx == GEM_TX_RING_SIZE - 1u) ? (1u << 30) : 0u;

            GEM_DESC_BUF(content_idx)   = FRAME_BASE_ADDR + i * sizeof(eth_frame);
            GEM_DESC_FLAGS(content_idx) = (1u << 31);

            GEM_DESC_BUF(trailer_idx)   = FRAME_BASE_ADDR + i * sizeof(eth_frame);
            GEM_DESC_FLAGS(trailer_idx) = (1u << 31) | trailer_wrap;
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

    /* Both rings/queue bases must reach DDR before RXEN goes live below
     * and GEM starts consuming the RX ring unprompted. */
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
 * RTL8211F PHYSR (MDIO reg 26) is only valid on extended page 0xa43
 * (PAGSR, reg 31); PHY defaults to page 0xa42, so switch, read, switch
 * back. bits[5:4]=speed(10=1000M,01=100M,00=10M), bit3=duplex, bit2=link.
 */
uint32_t phy_get_link_status(void){
    phy_write_mdio(0x34, 0x507e0a43); /* PHY addr 0, write reg 31 (PAGSR) = 0xa43 */
    phy_write_mdio(0x34, 0x606a0000); /* PHY addr 0, request read of reg 26 (PHYSR) */
    uint32_t physr = phy_query_mdio(0x34) & 0xffff;
    phy_write_mdio(0x34, 0x507e0a42); /* restore default page 0xa42 */
    return physr;
}

/*
 * Reserve the next TX frame slot. Returns a pointer to its content
 * buffer if GEM is done with it (content descriptor's USED==1), or NULL
 * if the ring is full (no blocking/retry -- a full ring drops the frame).
 * Checking only the content descriptor is sufficient: hardware only ever
 * reports completion there (UG585 Table 16-3), and both descriptors of a
 * slot always move together.
 */
void *eth_tx_reserve(void){
    if (!(GEM_DESC_FLAGS(tx_next * 2u) & (1u << 31))) {
        return NULL;
    }
    return (void *)(FRAME_BASE_ADDR + tx_next * sizeof(eth_frame));
}

/*
 * Arm both descriptors of the slot from the last eth_tx_reserve() call
 * and kick GEM. Trailer (zero-length, LAST=1) is written first, still
 * safely dormant since GEM reaches the content descriptor first; the
 * content descriptor's USED bit clears last, releasing the frame. Both
 * descriptors are rewritten every time since this slot may have last
 * held a sample-stream frame whose trailer pointed elsewhere.
 */
void eth_tx_commit(uint16_t len){
    uint32_t content_idx = tx_next * 2u;
    uint32_t trailer_idx = content_idx + 1u;
    uint32_t trailer_wrap = (trailer_idx == GEM_TX_RING_SIZE - 1u) ? (1u << 30) : 0u;

    GEM_DESC_BUF(trailer_idx)   = FRAME_BASE_ADDR + tx_next * sizeof(eth_frame); /* unused, 0-length */
    GEM_DESC_FLAGS(trailer_idx) = (0u << 31) | trailer_wrap | (1u << 15) | 0u;

    GEM_DESC_BUF(content_idx)   = FRAME_BASE_ADDR + tx_next * sizeof(eth_frame);
    GEM_DESC_FLAGS(content_idx) = (0u << 31) | (0u << 15) | len; /* LAST=0: trailer completes the frame */

    __asm__ volatile ("dsb" ::: "memory"); /* descriptor writes must reach DDR before GEM is kicked */
    GEM_NWCTRL = GEM_NWCTRL | (0x1u << 9);
    tx_next = (tx_next + 1u) % GEM_TX_FRAME_SLOTS;
}

/*
 * Reserve one scatter-gather frame slot -- shares tx_next/the same
 * physical ring as eth_tx_reserve() (see the GEM_TX_RING_SIZE comment in
 * eth0.h). Returns a buffer to build the Ethernet+IP+UDP header into
 * (the payload itself is zero-copy), or NULL if the ring is full.
 */
void *eth_tx_sg_reserve(void){
    if (!(GEM_DESC_FLAGS(tx_next * 2u) & (1u << 31))) {
        return NULL;
    }
    return (void *)(FRAME_BASE_ADDR + tx_next * sizeof(eth_frame));
}

/*
 * Arms both descriptors of the slot from the last eth_tx_sg_reserve()
 * call and kicks GEM once, same ordering discipline as eth_tx_commit().
 * `payload_addr` points directly at the real data -- never copied.
 */
void eth_tx_sg_commit(uint16_t hdr_len, uint32_t payload_addr, uint16_t payload_len){
    uint32_t content_idx = tx_next * 2u;
    uint32_t trailer_idx = content_idx + 1u;
    uint32_t trailer_wrap = (trailer_idx == GEM_TX_RING_SIZE - 1u) ? (1u << 30) : 0u;

    GEM_DESC_BUF(trailer_idx)   = payload_addr;
    GEM_DESC_FLAGS(trailer_idx) = (0u << 31) | trailer_wrap | (1u << 15) | payload_len;

    GEM_DESC_BUF(content_idx)   = FRAME_BASE_ADDR + tx_next * sizeof(eth_frame); /* header buffer */
    GEM_DESC_FLAGS(content_idx) = (0u << 31) | (0u << 15) | hdr_len; /* LAST=0: trailer continues this frame */

    __asm__ volatile ("dsb" ::: "memory"); /* both descriptors must reach DDR before GEM is kicked */
    GEM_NWCTRL = GEM_NWCTRL | (0x1u << 9);
    tx_next = (tx_next + 1u) % GEM_TX_FRAME_SLOTS;
}

/*
 * GEM's TX DMA halts outright when it walks into a not-ready (USED=1)
 * descriptor mid-frame, and does not resume on its own -- matches the
 * reference XEmacPs driver's own comment on this condition ("it is
 * expected that the user will reset the device in nearly all
 * instances"). Recovery: stop TX, rewrite TXQBASE to firmware's current
 * tx_next slot (resyncing GEM's hardware pointer to firmware's own
 * bookkeeping), re-enable TX, pulse TSTART. Frames in flight between
 * GEM's old position and tx_next are dropped -- same "drop rather than
 * block" stance as the rest of this design.
 *
 * Only triggered on GEM_TXSR_HALT_MASK, not bare USEDREAD: USEDREAD
 * alone is routine (GEM catching up to the ring's frontier, see its own
 * comment in eth0.h) and every commit's TSTART pulse already handles it.
 * Force-resyncing on every routine blip skips past whatever frame GEM
 * was about to send next -- invisible for high-rate sample-stream
 * traffic but drops rare, latency-sensitive replies before GEM reaches
 * them.
 *
 * A second detector below (TXQBASE frozen with real work pending) covers
 * a halt that leaves TXSR completely clean -- GEM can stop dead with
 * zero error bits latched, invisible to the TXSR check alone.
 */
#define ETH_TX_STALL_CHECK_INTERVAL 65536u /* main-loop iterations between checks -- not hardware-timed, just infrequent enough to be nearly free */
static uint32_t tx_stall_last_txqbase = 0xFFFFFFFFu; /* sentinel: first check only seeds this, never trips */
static uint32_t tx_stall_counter = 0u;

static void eth_tx_force_resync(void){
    GEM_NWCTRL = GEM_NWCTRL & ~GEM_NWCTRL_TXEN;
    GEM_TXQBASE = GEM_DESCRIPTOR_TX + tx_next * 2u * 8u;
    __asm__ volatile ("dsb" ::: "memory"); /* TXQBASE must land before TXEN comes back */
    GEM_NWCTRL = GEM_NWCTRL | GEM_NWCTRL_TXEN;
    GEM_NWCTRL = GEM_NWCTRL | GEM_NWCTRL_TSTART;
}

void eth_tx_recover(void){
    uint32_t status = GEM_TXSR;
    uint8_t  stalled_silent = 0u;

    if (status != 0u) {
        GEM_TXSR = status; /* write-1-to-clear whatever's set -- harmless housekeeping either way */
    }

    tx_stall_counter++;
    if ((tx_stall_counter & (ETH_TX_STALL_CHECK_INTERVAL - 1u)) == 0u) {
        uint32_t cur_txqbase = GEM_TXQBASE;
        uint8_t  work_pending = !(GEM_DESC_FLAGS(tx_next * 2u) & (1u << 31));
        if (cur_txqbase == tx_stall_last_txqbase && work_pending) {
            stalled_silent = 1u;
        }
        tx_stall_last_txqbase = cur_txqbase;
    }

    if ((status & GEM_TXSR_HALT_MASK) || stalled_silent) {
        eth_tx_force_resync();
    }
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
 * blocking. Never touches rx_next -- only eth_rx_release() advances the
 * ring, so poll can be called repeatedly with no side effects.
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
    /* Release must reach DDR before GEM's own async re-check of this
     * slot, same reasoning as eth_tx_commit()'s barrier -- GEM polls the
     * RX ring on its own, no kick register to wait on instead. */
    __asm__ volatile ("dsb" ::: "memory");
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

/* Byte offsets: 0-5 dest MAC, 6-11 src MAC, 12-13 EtherType, ARP payload
 * from 14 -- standard Ethernet+ARP wire layout. */
/*
 * Deferred retry state for eth_send_arp_reply(): eth_tx_reserve() can
 * transiently fail under heavy sample-stream ring contention even though
 * GEM itself is healthy. Unlike a dropped sample packet (an accepted
 * tradeoff elsewhere in this design), losing the only attempt at an ARP
 * reply breaks resolution outright -- nothing else prompts a retry from
 * this side. Preserves just the fields eth_send_arp_reply() reads
 * (requester's SHA at req[6:12], THA+SPA at req[22:32]) and retries once
 * per main-loop pass via eth_arp_retry_poll() until it succeeds or a
 * newer request supersedes it.
 */
static uint8_t arp_retry_pending = 0u;
static uint8_t arp_retry_req[32];

void eth_send_arp_reply(const uint8_t *req)
{
    uint8_t *buf = (uint8_t *)eth_tx_reserve();
    uint32_t i;
    if (buf == NULL) {
        for (i = 0u; i < 32u; i++) { arp_retry_req[i] = req[i]; }
        arp_retry_pending = 1u;
        return;
    }
    arp_retry_pending = 0u; /* this attempt succeeded -- drop any older pending retry */

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
 * Retries a deferred ARP reply if one is pending -- call once per
 * main-loop iteration, same cheap-when-idle idiom as
 * eth_poll_sample_stream()/eth_tx_recover(). Re-enters
 * eth_send_arp_reply(), which either succeeds (clearing the pending flag)
 * or re-saves the identical bytes and stays pending for the next pass.
 */
void eth_arp_retry_poll(void)
{
    if (!arp_retry_pending) {
        return;
    }
    eth_send_arp_reply(arp_retry_req);
}

/*
 * Reserve a TX slot and pre-fill every UDP reply header field that
 * doesn't depend on payload length. Total length and IP checksum are
 * filled in by eth_udp_reply_commit() once the payload size is known.
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

/* Test-only: hand-crafted ARP request "from" a fake requester
 * (00:11:22:33:44:55, 192.168.3.99) targeting this board's IP, written
 * directly into the current RX slot -- bypasses GEM entirely, so
 * eth_service()'s ARP path can be exercised on demand. */
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

/* Finalize a UDP reply: write the real IP/UDP length fields now that
 * payload_len is known, compute the IP checksum, zero-pad to the
 * Ethernet minimum if needed, and commit. */
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

/*
 * Deferred retry state for the UDP command reply -- same ring-contention
 * gap as eth_send_arp_reply() (see its comment), just on the command
 * console's reply path instead. Preserves the header fields
 * eth_udp_reply_reserve() reads (up through req[35], dest port) plus the
 * already-computed reply payload, and retries once per main-loop pass via
 * eth_udp_retry_poll() until it succeeds or a newer reply supersedes it.
 */
static uint8_t  udp_retry_pending = 0u;
static uint8_t  udp_retry_req[38];
static uint8_t  udp_retry_batch[ETH_UDP_MAX_REPLY_BYTES];
static uint16_t udp_retry_bytes = 0u;

void eth_udp_reply_send(const uint8_t *req, const uint8_t *batch, uint16_t bytes)
{
    uint8_t *out = (uint8_t *)eth_udp_reply_reserve(req);
    uint16_t i;

    if (out == NULL) {
        for (i = 0u; i < 38u; i++) { udp_retry_req[i] = req[i]; }
        for (i = 0u; i < bytes; i++) { udp_retry_batch[i] = batch[i]; }
        udp_retry_bytes = bytes;
        udp_retry_pending = 1u;
        return;
    }
    udp_retry_pending = 0u; /* this attempt succeeded -- drop any older pending retry */

    for (i = 0u; i < bytes; i++) { out[i] = batch[i]; }
    eth_udp_reply_commit(bytes);
}

/*
 * Retries a deferred UDP command reply if one is pending -- call once per
 * main-loop iteration, same cheap-when-idle idiom as
 * eth_arp_retry_poll()/eth_poll_sample_stream()/eth_tx_recover().
 */
void eth_udp_retry_poll(void)
{
    if (!udp_retry_pending) {
        return;
    }
    eth_udp_reply_send(udp_retry_req, udp_retry_batch, udp_retry_bytes);
}

/*
 * Build and send one sample-stream packet: Ethernet+IP+UDP header,
 * broadcast Ethernet destination (no ARP client to resolve the PC's real
 * MAC), followed by a payload descriptor pointing directly at
 * `payload_addr` -- never copied. No padding-to-minimum needed here: a
 * sample packet's payload is always >=1KB. Returns 0 on success, nonzero
 * if the SG ring is full.
 */
uint8_t eth_send_sample_packet(uint32_t payload_addr, uint16_t payload_len)
{
    uint8_t *buf = (uint8_t *)eth_tx_sg_reserve();
    uint16_t ip_total_len;
    uint16_t udp_len;
    uint16_t checksum;
    uint32_t i;

    if (buf == NULL) {
        return 1u;
    }

    for (i = 0u; i < 6u; i++) { buf[i] = 0xFFu; }               /* dest MAC: broadcast, see header comment */
    for (i = 0u; i < 6u; i++) { buf[6u + i] = g_board_mac[i]; } /* src MAC = board */
    buf[12] = 0x08u; buf[13] = 0x00u;                            /* EtherType = IPv4 */

    ip_total_len = (uint16_t)(20u + 8u + payload_len);
    udp_len      = (uint16_t)(8u + payload_len);

    buf[14] = 0x45u; /* version 4, IHL 5 (20-byte header, no options) */
    buf[15] = 0x00u; /* DSCP/ECN */
    buf[16] = (uint8_t)(ip_total_len >> 8); buf[17] = (uint8_t)ip_total_len;
    buf[18] = 0x00u; buf[19] = 0x00u; /* identification -- unfragmented, 0 is fine */
    buf[20] = 0x00u; buf[21] = 0x00u; /* flags/fragment offset */
    buf[22] = 64u;   /* TTL */
    buf[23] = 17u;   /* protocol = UDP */
    buf[24] = 0x00u; buf[25] = 0x00u; /* header checksum -- filled in below */
    buf[26] = BOARD_IP0; buf[27] = BOARD_IP1; buf[28] = BOARD_IP2; buf[29] = BOARD_IP3; /* src IP */
    buf[30] = SAMPLE_DEST_IP0; buf[31] = SAMPLE_DEST_IP1;
    buf[32] = SAMPLE_DEST_IP2; buf[33] = SAMPLE_DEST_IP3; /* dest IP */

    buf[34] = (uint8_t)(SAMPLE_DEST_PORT >> 8); buf[35] = (uint8_t)SAMPLE_DEST_PORT; /* src port */
    buf[36] = (uint8_t)(SAMPLE_DEST_PORT >> 8); buf[37] = (uint8_t)SAMPLE_DEST_PORT; /* dest port */
    buf[38] = (uint8_t)(udp_len >> 8); buf[39] = (uint8_t)udp_len;
    buf[40] = 0x00u; buf[41] = 0x00u; /* UDP checksum = 0 (optional, per roadmap) */

    checksum = ip_checksum(buf + 14, 20u);
    buf[24] = (uint8_t)(checksum >> 8); buf[25] = (uint8_t)checksum;

    eth_tx_sg_commit(ETH_UDP_HEADER_LEN, payload_addr, payload_len);
    return 0u;
}

/*
 * Checks axi_notifications' register 0 for a new sample-stream batch and,
 * if ready, sends it via eth_send_sample_packet() -- zero-copy, straight
 * from wherever axi_dsp wrote it. Acks unconditionally once handled, even
 * if the SG ring was full and the send got dropped: packet loss when PS
 * falls behind is an accepted risk, and leaving the ready bit set would
 * just stall axi_dsp's PL-side bookkeeping instead of helping.
 */
void eth_poll_sample_stream(void)
{
    uint32_t status = REG_SAMPLE_NOTIF;
    uint32_t index;
    uint32_t payload_addr;

    if (!(status & NOTIF_READY_MASK)) {
        return;
    }

    index        = (status & NOTIF_INDEX_MASK) >> NOTIF_INDEX_SHIFT;
    payload_addr = SAMPLE_STREAM_BASE + index * SAMPLE_STREAM_NOTIF_BYTES;

    (void)eth_send_sample_packet(payload_addr, (uint16_t)SAMPLE_STREAM_NOTIF_BYTES);

    REG_SAMPLE_NOTIF = status & ~NOTIF_READY_MASK;
}