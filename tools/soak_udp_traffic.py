#!/usr/bin/env python3
"""
30-minute UDP traffic generator for the eth_rx_release() dsb-fix soak test.

Sends one batched AXI regmap read (offsets 0x00/0x04/0x08) every few
seconds for the given duration, logging every send/reply/timeout with a
timestamp. Exists purely to keep real RX frames landing on the board so
eth_rx_release() actually gets exercised repeatedly -- without traffic,
this fix would never be stressed at all during the soak.

Run: python soak_udp_traffic.py [duration_seconds] [interval_seconds]
"""
import socket
import struct
import sys
import time

BOARD_IP = "192.168.3.50"
UDP_CMD_PORT = 5555
UDP_CMD_PREAMBLE = b"COM\x00"
STOP_SENTINEL = b"\xff" * 8


def pack_command(dev, rw, addr, data):
    return bytes([dev & 0xFF, rw & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]) \
        + struct.pack(">I", data & 0xFFFFFFFF)


def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 1800
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)

    dev_axi = 0x00
    cmds = [pack_command(dev_axi, 0x00, off, 0) for off in (0x00, 0x04, 0x08)]
    payload = UDP_CMD_PREAMBLE + b"".join(cmds) + STOP_SENTINEL

    start = time.time()
    sent = 0
    ok = 0
    timeouts = 0
    errors = 0

    print(f"=== UDP traffic generator started, duration={duration}s interval={interval}s ===", flush=True)
    while time.time() - start < duration:
        sent += 1
        ts = time.strftime("%H:%M:%S")
        elapsed = int(time.time() - start)
        try:
            sock.sendto(payload, (BOARD_IP, UDP_CMD_PORT))
            data, _ = sock.recvfrom(4096)
            if len(data) >= 12:
                vals = struct.unpack(">III", data[:12])
                ok += 1
                print(f"[{ts}] t+{elapsed}s send #{sent}: OK -> {[hex(v) for v in vals]}", flush=True)
            else:
                errors += 1
                print(f"[{ts}] t+{elapsed}s send #{sent}: SHORT REPLY ({len(data)} bytes)", flush=True)
        except socket.timeout:
            timeouts += 1
            print(f"[{ts}] t+{elapsed}s send #{sent}: TIMEOUT (no reply)", flush=True)
        except OSError as exc:
            errors += 1
            print(f"[{ts}] t+{elapsed}s send #{sent}: ERROR {exc}", flush=True)
        time.sleep(interval)

    print(f"=== UDP traffic generator finished: {sent} sent, {ok} ok, {timeouts} timeouts, {errors} errors ===", flush=True)


if __name__ == "__main__":
    main()
