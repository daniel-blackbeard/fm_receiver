#include <stdint.h>

#ifndef ETH0_H
#define ETH0_H

#define GEM_SLCR_BASE 0xF8000000u /* GEM SLCR Address */
#define GEM_SLCR_UNLOCK    (*(volatile uint32_t *) (GEM_SLCR_BASE + 0x008u))
#define GEM_SLCR_LOCK      (*(volatile uint32_t *) (GEM_SLCR_BASE + 0x004u))
#define GEM_SLCR_RCLK_CTRL (*(volatile uint32_t *) (GEM_SLCR_BASE + 0x138u))
#define GEM_SLCR_CLK_CTRL  (*(volatile uint32_t *) (GEM_SLCR_BASE + 0x140u))
#define GEM_RST_CTRL       (*(volatile uint32_t *) (GEM_SLCR_BASE + 0x214u))
#define GEM_RST_CTRL_MASK  ((1u << 6) | (1u << 4) | (1u << 0)) /* GEM0_REF_RST, GEM0_RX_RST, GEM0_CPU1X_RST */

#define GEM_CORE_BASE 0xE000B000u  /* GEM0 Address */
#define GEM_NWCTRL         (*(volatile uint32_t *) (GEM_CORE_BASE + 0x000u))
#define GEM_NWCFG          (*(volatile uint32_t *) (GEM_CORE_BASE + 0x004u))
#define GEM_NWSR           (*(volatile uint32_t *) (GEM_CORE_BASE + 0x008u))
#define GEM_DMACR          (*(volatile uint32_t *) (GEM_CORE_BASE + 0x010u))
#define GEM_TXSR           (*(volatile uint32_t *) (GEM_CORE_BASE + 0x014u))
#define GEM_RXQBASE        (*(volatile uint32_t *) (GEM_CORE_BASE + 0x018u))
#define GEM_TXQBASE        (*(volatile uint32_t *) (GEM_CORE_BASE + 0x01cu))
#define GEM_RXSR           (*(volatile uint32_t *) (GEM_CORE_BASE + 0x020u))
#define GEM_ISR            (*(volatile uint32_t *) (GEM_CORE_BASE + 0x024u))
#define GEM_IER            (*(volatile uint32_t *) (GEM_CORE_BASE + 0x028u))
#define GEM_IDR            (*(volatile uint32_t *) (GEM_CORE_BASE + 0x02cu))
#define GEM_IMR            (*(volatile uint32_t *) (GEM_CORE_BASE + 0x030u))
#define GEM_PHYMNTNC       (*(volatile uint32_t *) (GEM_CORE_BASE + 0x034u))
#define GEM_SPEC_ADDR1_BOT (*(volatile uint32_t *) (GEM_CORE_BASE + 0x088u))
#define GEM_SPEC_ADDR1_TOP (*(volatile uint32_t *) (GEM_CORE_BASE + 0x08cu))

#define GEM_NWCTRL_MDEN   (1u << 4) /* management port enable -- must be set before any MDIO transaction */
#define GEM_NWCTRL_RXEN   (1u << 2)
#define GEM_NWCTRL_TXEN   (1u << 3)
#define GEM_NWCTRL_TSTART (1u << 9) /* re-examine/resume the TX queue at its current pointer */

/*
 * GEM_TXSR (TX status) bits, values from Xilinx's XEmacPs driver
 * (private/docs/.../xemacps_hw.h). GEM's TX DMA halts outright the
 * instant it walks into a not-ready (USED=1) descriptor mid-frame, and
 * does NOT resume on its own -- matches the reference driver's own
 * comment ("it is expected that the user will reset the device in nearly
 * all instances"). See eth_tx_recover() in eth0.c for the recovery.
 */
#define GEM_TXSR_HRESPNOK  (1u << 8) /* AHB bus error */
#define GEM_TXSR_URUN      (1u << 6) /* TX underrun */
#define GEM_TXSR_TXCOMPL   (1u << 5) /* a frame completed OK -- not an error, but sticky like the rest */
#define GEM_TXSR_BUFEXH    (1u << 4) /* buffers exhausted mid-frame -- GEM already started a multi-descriptor frame and got stuck partway through it */
#define GEM_TXSR_TXGO      (1u << 3) /* status of go flag -- not an error */
#define GEM_TXSR_RETRY     (1u << 2) /* retry limit exceeded */
#define GEM_TXSR_COLLISION (1u << 1) /* collision on TX frame */
/*
 * USEDREAD (bit0) alone -- GEM scanned for the next frame and found the
 * ring's frontier not armed yet -- is the normal idle condition, not a
 * halt: every eth_tx_commit()/eth_tx_sg_commit() already pulses TSTART
 * unconditionally, which is all GEM needs to pick back up. Force-
 * resyncing TXQBASE on every occurrence would skip whatever legitimate
 * frame GEM was about to reach next -- harmless for sample-stream traffic
 * (tx_next is always fresh) but drops sparser UDP command replies before
 * GEM gets to them. GEM_TXSR_HALT_MASK deliberately excludes it -- only
 * BUFEXH/HRESPNOK/URUN/RETRY/COLLISION, which mean GEM is genuinely stuck
 * or hit a real error, trigger the aggressive resync.
 */
#define GEM_TXSR_USEDREAD  (1u << 0)
#define GEM_TXSR_HALT_MASK (GEM_TXSR_HRESPNOK | GEM_TXSR_URUN | GEM_TXSR_BUFEXH | \
                             GEM_TXSR_RETRY | GEM_TXSR_COLLISION)

/* Valid DDR scratch range starts at 0x00200000 -- below that is either
 * this program's own load region or outside the PS7-declared usable
 * RAM window (see linker.ld). */
#define GEM_DESCRIPTOR_TX  0x00800000u  /* GEM0 TX descriptor ring base */
#define GEM_DESCRIPTOR_RX  0x00900000u  /* GEM0 TX descriptor ring base */
#define FRAME_BASE_ADDR    0x01000000u  /* first of GEM_TX_RING_SIZE per-slot buffers */

/*
 * TX ring: GEM_TX_RING_SIZE physical descriptors, 8 bytes each
 * (word0=buffer address, word1=control/flags), contiguous from
 * GEM_DESCRIPTOR_TX. Every frame -- test frame, ARP reply, UDP command
 * reply, and zero-copy sample-stream packets -- always consumes exactly
 * 2 consecutive descriptors: a "content" descriptor (LAST=0) followed by
 * a "trailer" descriptor (LAST=1, zero-length filler or the real
 * sample-stream payload pointed at directly). GEM_TX_RING_SIZE/2 =
 * GEM_TX_FRAME_SLOTS is the real usable frame count, all sharing one
 * producer index (tx_next, in eth0.c).
 *
 * Uniform 2-descriptor occupancy is load-bearing: hardware only sets the
 * USED bit back on a frame's *first* descriptor (UG585 Table 16-3), so a
 * ring mixing 1- and 2-descriptor allocations can't reliably tell a
 * "second descriptor" slot is free; and GEM walks the ring strictly
 * sequentially with no way to skip a dormant region, so a separate,
 * rarely-used ring for sample-stream traffic can permanently stall GEM's
 * queue pointer the first time it reaches it. See
 * private/sample_streaming_plan.md for the full history.
 */
#define GEM_TX_RING_SIZE   64u                    /* physical descriptor count */
#define GEM_TX_FRAME_SLOTS (GEM_TX_RING_SIZE / 2u) /* usable frames -- 2 descriptors each */
#define GEM_RX_RING_SIZE 8u
#define GEM_DESC_BUF(i)   (*(volatile uint32_t *) (GEM_DESCRIPTOR_TX + (uint32_t)(i) * 8u))
#define GEM_DESC_FLAGS(i) (*(volatile uint32_t *) (GEM_DESCRIPTOR_TX + (uint32_t)(i) * 8u + 4u))
#define GEM_DESC_BUF_RX(i)   (*(volatile uint32_t *) (GEM_DESCRIPTOR_RX + (uint32_t)(i) * 8u))
#define GEM_DESC_FLAGS_RX(i) (*(volatile uint32_t *) (GEM_DESCRIPTOR_RX + (uint32_t)(i) * 8u + 4u))

/* First of GEM_TX_FRAME_SLOTS per-slot buffers, right after the TX
 * descriptor ring. Named (not inlined) so gem_setup()'s ring-init loop
 * and eth_rx_poll()/eth_rx_release() can't drift apart on the same
 * value. Slot i's buffer lives at FRAME_BASE_ADDR + i*sizeof(eth_frame)
 * -- shared by every sender (test frame, ARP reply, UDP reply, and the
 * sample-stream header) regardless of who's using frame slot i this
 * time; only the sample-stream payload itself is ever zero-copy (its
 * descriptor points directly into SAMPLE_STREAM_BASE instead). */
#define GEM_RX_BUF_BASE (FRAME_BASE_ADDR + GEM_TX_FRAME_SLOTS * sizeof(eth_frame))
/* Real 1518-byte Ethernet MTU (rounded up), not a small placeholder --
 * keeps one frame within one RX descriptor. */
#define GEM_RX_BUF_STRIDE 1536u

/*
 * PL sample-streaming buffer (axi_dsp -> DDR -> ETH0), see
 * private/sample_streaming_plan.md for the full design. A 1MB circular
 * buffer axi_dsp fills via S_AXI_HP0; firmware never writes or copies
 * it, only reads the notification register below and points a TX
 * descriptor at the right offset (zero-copy). Marked Strongly Ordered
 * in startup.S's MMU table so no cache maintenance is needed.
 */
#define SAMPLE_STREAM_BASE        0x02000000u
#define SAMPLE_STREAM_SIZE        0x00100000u /* 1 MByte */
/* One notification = 8 AXI bursts x 128 bytes/burst (axi_dsp.sv's
 * BANK_SAMPLES x 8 bytes/beat), confirmed to divide SAMPLE_STREAM_SIZE
 * evenly (1MB / 1KB = 1024 exactly) -- a notification never straddles
 * the circular buffer's wrap point, by construction. */
#define SAMPLE_STREAM_NOTIF_BYTES 1024u
#define SAMPLE_STREAM_NOTIF_COUNT (SAMPLE_STREAM_SIZE / SAMPLE_STREAM_NOTIF_BYTES) /* 1024 */

/*
 * axi_notifications (PERIPH_ID 0x04): PL-to-PS status regmap, see
 * src/axi_notifications.sv's header comment. Register 0 is axi_dsp's
 * notification word -- bit0 (NOTIF_READY_MASK) is set by PL when a new
 * 1KB slice has landed, bits[10:1] (NOTIF_INDEX_MASK) are which slice
 * (SAMPLE_STREAM_BASE + index*SAMPLE_STREAM_NOTIF_BYTES); firmware acks
 * by writing the register back with bit0 cleared (plain read-modify-
 * write, per the agreed PL-always-wins collision priority). Registers
 * 1-3 are spare for future PL-side status.
 */
#define NOTIF_AXI_BASE (0x40000000u | (0x04u << 16)) /* GP0 base | axi_notifications' PERIPH_ID */
#define REG_SAMPLE_NOTIF (*(volatile uint32_t *)(NOTIF_AXI_BASE + 0x00u))
#define NOTIF_READY_MASK  0x00000001u
#define NOTIF_INDEX_MASK  0x000007FEu /* bits [10:1] */
#define NOTIF_INDEX_SHIFT 1u

/* Board identity for the UDP command protocol -- MAC matches
 * GEM_SPEC_ADDR1_BOT/TOP in gem_setup() (02:00:de:ad:be:ef). Fixed, no
 * DHCP/ARP-learned addressing. */
#define BOARD_IP0 192u
#define BOARD_IP1 168u
#define BOARD_IP2 3u
#define BOARD_IP3 50u
#define UDP_CMD_PORT 5555u

/* Sample-stream destination -- static, matching this project's
 * no-DHCP/no-dynamic-ARP-resolution approach (same spirit as BOARD_IP*
 * above). Ethernet destination itself is broadcast (see
 * eth_send_sample_packet()) since this stack has no ARP client, only a
 * responder. Distinct port from UDP_CMD_PORT so sample traffic never
 * blocks behind the command console. */
#define SAMPLE_DEST_IP0  192u
#define SAMPLE_DEST_IP1  168u
#define SAMPLE_DEST_IP2  3u
#define SAMPLE_DEST_IP3  9u
#define SAMPLE_DEST_PORT 5556u

/* Ethernet minimum frame size (source bytes, before the 4-byte FCS GEM
 * appends itself) -- GEM does NOT auto-pad short TX frames, so anything
 * built shorter than this (the ARP reply is 42 bytes, some UDP replies
 * could be too) must be zero-padded by software before eth_tx_commit(). */
#define ETH_MIN_FRAME_LEN 60u

/* Ethernet+IP+UDP header size preceding a UDP reply's payload (14+20+8).
 * Bounds how many 4-byte command replies eth_process_udp_command_frame()
 * can batch into one reply frame, since replies share the same
 * sizeof(eth_frame)-sized TX buffers as everything else on this ring. */
#define ETH_UDP_HEADER_LEN 42u
#define ETH_UDP_MAX_REPLY_BYTES (sizeof(eth_frame) - ETH_UDP_HEADER_LEN)

void gem_slcr_setup(void);
void gem_slcr_lock(void);
void gem_setup(void);
uint32_t phy_query_mdio(uint32_t addr);
void phy_write_mdio(uint32_t addr, uint32_t data);
uint32_t phy_get_rtl_identifier(void);
uint32_t phy_get_link_status(void);
void *eth_tx_reserve(void);
void eth_tx_commit(uint16_t len);

/*
 * Checks GEM_TXSR for a halted TX DMA (see GEM_TXSR_HALT_MASK above) and,
 * if found, clears it and re-kicks the queue. Call once per main-loop
 * iteration, same cadence as eth_service()/eth_poll_sample_stream() -- a
 * single cheap register read when TX is healthy, which is nearly always.
 */
void eth_tx_recover(void);
void eth_send_test_frame(void);

/* Scatter-gather TX for zero-copy sample-stream sends -- shares the same
 * ring/producer index as eth_tx_reserve()/eth_tx_commit() (see the
 * GEM_TX_RING_SIZE comment in this header for why that sharing is safe).
 * reserve() returns a buffer to build the Ethernet+IP+UDP header into (or
 * NULL if the ring is full); commit() arms both descriptors and kicks
 * GEM once, with `payload_addr` pointing directly at the real data. */
void *eth_tx_sg_reserve(void);
void eth_tx_sg_commit(uint16_t hdr_len, uint32_t payload_addr, uint16_t payload_len);

/* Builds the Ethernet+IP+UDP header for one sample-stream packet and
 * sends it via the scatter-gather path, with `payload_addr`/
 * `payload_len` describing the real sample data (zero-copy, never
 * touched here). Returns 0 on success, nonzero if the ring is full. */
uint8_t eth_send_sample_packet(uint32_t payload_addr, uint16_t payload_len);

/* Checks axi_notifications for a new sample-stream batch and sends it if
 * one is ready, acking unconditionally once handled (see eth0.c for the
 * full reasoning). Call once per main-loop iteration, same cadence as
 * eth_service(). */
void eth_poll_sample_stream(void);

/* RX ring consumer, mirroring eth_tx_reserve()/eth_tx_commit(): poll
 * returns a pointer to the current RX slot's buffer and length if a frame
 * is ready (NEW==1), or NULL otherwise -- never blocks. release() hands
 * the slot back to hardware and advances; call exactly once per
 * successful poll(), after the frame is no longer needed. */
void *eth_rx_poll(uint16_t *len_out);
void eth_rx_release(void);

/* Minimal ARP responder: given a received frame already confirmed to be
 * an ARP request for this board's IP, builds and sends the reply. If the
 * TX ring is momentarily full (real, confirmed on hardware under heavy
 * sample-stream contention), the attempt is saved and retried
 * automatically -- see eth_arp_retry_poll(). */
void eth_send_arp_reply(const uint8_t *req_frame);

/* Retries a deferred ARP reply if eth_send_arp_reply() couldn't get a TX
 * slot on its first attempt. Call once per main-loop iteration, same
 * cadence as eth_service()/eth_poll_sample_stream()/eth_tx_recover(). */
void eth_arp_retry_poll(void);

/* UDP reply builder, split reserve/commit like the TX ring, since the
 * final length fields and checksum can't be written until the caller
 * knows the payload size. reserve() takes the *request* frame (to swap
 * src/dst for the reply); commit() finalizes and sends. At most
 * ETH_UDP_MAX_REPLY_BYTES fits -- caller's responsibility. */
void *eth_udp_reply_reserve(const uint8_t *req_frame);
void eth_udp_reply_commit(uint16_t payload_len);

/* Sends a UDP command reply (payload already computed into `batch`,
 * `bytes` long), retrying automatically via eth_udp_retry_poll() if the
 * TX ring is momentarily full (confirmed on real hardware under heavy
 * sample-stream contention -- same class of gap eth_send_arp_reply() had,
 * see its own comment in eth0.c). Preferred over calling
 * eth_udp_reply_reserve()/eth_udp_reply_commit() directly. */
void eth_udp_reply_send(const uint8_t *req_frame, const uint8_t *batch, uint16_t bytes);

/* Retries a deferred UDP command reply if eth_udp_reply_send() couldn't
 * get a TX slot on its first attempt. Call once per main-loop iteration,
 * same cadence as the other eth_*_poll()/eth_tx_recover() calls. */
void eth_udp_retry_poll(void);

/* Test-only: synthesizes an ARP request for this board's IP directly into
 * the current RX slot, as if hardware had just received it. */
void eth_test_inject_arp_request(void);

/* packed: wire fields must be byte-contiguous. uint8_t arrays (not
 * uint16_t/uint32_t) sidestep endianness -- this CPU is little-endian,
 * the wire format isn't, and a byte written lands in memory in that
 * exact order. */
typedef struct __attribute__((packed)) {
    uint8_t des_mac[6];
    uint8_t src_mac[6];
    uint8_t ether_type[2];
    uint8_t payload[128];
} eth_frame;

#endif
