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

#define GEM_NWCTRL_MDEN (1u << 4) /* management port enable -- must be set before any MDIO transaction */
#define GEM_NWCTRL_RXEN (1u << 2)
#define GEM_NWCTRL_TXEN (1u << 3)

/* Valid DDR scratch range starts at 0x00200000 -- below that is either
 * this program's own load region or outside the PS7-declared usable
 * RAM window (see linker.ld). */
#define GEM_DESCRIPTOR_TX  0x00800000u  /* GEM0 TX descriptor ring base */
#define GEM_DESCRIPTOR_RX  0x00900000u  /* GEM0 TX descriptor ring base */
#define FRAME_BASE_ADDR    0x01000000u  /* first of GEM_TX_RING_SIZE per-slot buffers */

/* TX ring: GEM_TX_RING_SIZE descriptors, 8 bytes each (word0=buffer
 * address, word1=control/flags), contiguous from GEM_DESCRIPTOR. Slot i's
 * buffer lives at FRAME_BASE_ADDR + i*sizeof(eth_frame). */
#define GEM_TX_RING_SIZE 64u
#define GEM_RX_RING_SIZE 8u
#define GEM_DESC_BUF(i)   (*(volatile uint32_t *) (GEM_DESCRIPTOR_TX + (uint32_t)(i) * 8u))
#define GEM_DESC_FLAGS(i) (*(volatile uint32_t *) (GEM_DESCRIPTOR_TX + (uint32_t)(i) * 8u + 4u))
#define GEM_DESC_BUF_RX(i)   (*(volatile uint32_t *) (GEM_DESCRIPTOR_RX + (uint32_t)(i) * 8u))
#define GEM_DESC_FLAGS_RX(i) (*(volatile uint32_t *) (GEM_DESCRIPTOR_RX + (uint32_t)(i) * 8u + 4u))

/* First of GEM_RX_RING_SIZE per-slot RX buffers, right after the TX
 * buffer region (FRAME_BASE_ADDR .. FRAME_BASE_ADDR + 64*sizeof(eth_frame)).
 * Exposed here (rather than left as an inline expression) so a UART debug
 * peek path can reach it too, not just gem_setup()'s own ring-init loop. */
#define GEM_RX_BUF_BASE (FRAME_BASE_ADDR + 64u * sizeof(eth_frame))
/* Named so gem_setup()'s ring-init loop and eth_rx_poll()/eth_rx_release()
 * can't silently drift apart on the same magic number. */
#define GEM_RX_BUF_STRIDE 256u

/* Board identity for the UDP command protocol -- MAC matches
 * GEM_SPEC_ADDR1_BOT/TOP in gem_setup() (02:00:de:ad:be:ef). No DHCP/ARP-
 * learned addressing anywhere in this minimal stack; both ends are fixed,
 * known values. */
#define BOARD_IP0 192u
#define BOARD_IP1 168u
#define BOARD_IP2 3u
#define BOARD_IP3 50u
#define UDP_CMD_PORT 5555u

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
void eth_send_test_frame(void);

/* RX ring consumer, mirroring eth_tx_reserve()/eth_tx_commit(): poll
 * returns a pointer to the current RX slot's buffer and its received
 * length if hardware has a frame ready (NEW==1), or NULL if not (still
 * hardware's turn) -- never blocks. release() hands that same slot back
 * to hardware (clears NEW) and advances to the next slot; call it exactly
 * once per successful poll(), after the frame's contents have been fully
 * used, since hardware may overwrite the buffer as soon as it's released. */
void *eth_rx_poll(uint16_t *len_out);
void eth_rx_release(void);

/* Minimal ARP responder: given a pointer to a received frame already
 * confirmed to be an ARP request for this board's own IP, builds and
 * sends the matching ARP reply. */
void eth_send_arp_reply(const uint8_t *req_frame);

/* UDP reply builder, split into reserve/commit like the TX ring itself,
 * since the final IP/UDP length fields and IP checksum can't be written
 * until the caller knows how much payload it wrote. reserve() takes the
 * *request* frame (to swap src/dst MAC/IP/port for the reply) and returns
 * a pointer to where payload should be written, or NULL if the TX ring is
 * full; commit() finalizes headers/checksum/padding and sends it. At most
 * ETH_UDP_MAX_REPLY_BYTES of payload fits -- caller's responsibility to
 * respect that bound before calling commit(). */
void *eth_udp_reply_reserve(const uint8_t *req_frame);
void eth_udp_reply_commit(uint16_t payload_len);

/* Test-only: synthesizes a valid ARP request for this board's own IP
 * directly into the current RX slot (as if hardware had just received
 * it) and sets that slot's NEW bit, so eth_service() picks it up on its
 * very next call. Exists to reproduce the ARP-path crash on demand from
 * a single UART command, instead of depending on real, unpredictably-
 * timed network traffic to trigger it. */
void eth_test_inject_arp_request(void);

/* packed: wire fields must be byte-contiguous, no compiler-inserted
 * padding -- byte arrays already have 1-byte alignment so this is
 * redundant today, but it stays correct if a multi-byte field (e.g. a
 * checksum) is ever added later. uint8_t arrays instead of uint16_t/
 * uint32_t also sidesteps endianness: this CPU is little-endian, the
 * wire format isn't, and a byte you write is a byte that lands in
 * memory in that exact order, no swap needed. */
typedef struct __attribute__((packed)) {
    uint8_t des_mac[6];
    uint8_t src_mac[6];
    uint8_t ether_type[2];
    uint8_t payload[128];
} eth_frame;

#endif
