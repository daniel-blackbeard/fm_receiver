#!/usr/bin/env python3
"""Bare-minimum sample-stream rate counter -- no GUI, no plotting, no
FFT, not even snapshot(). Just SampleReceiver's background recv thread
(recvfrom + struct.unpack + deque.extend, see sample_stream_view.py)
and a print loop reading its counters. Use this to get a ground-truth
ETH rate reading with zero tool-side rendering overhead in the loop.

Usage: python3 tools/rate_counter.py
Ctrl+C to stop.
"""
import time

from sample_stream_view import SampleReceiver, SAMPLE_PORT


def main():
    recv = SampleReceiver(fft_size=16)  # buffers unused here, keep them tiny
    recv.start()
    print(f"Listening for sample-stream UDP packets on :{SAMPLE_PORT} ... Ctrl+C to stop")

    last_pkts = 0
    last_bytes = 0
    last_t = time.monotonic()
    try:
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            dt = now - last_t
            pkts = recv.packets_received
            byts = recv.bytes_received
            pkt_rate = (pkts - last_pkts) / dt
            byte_rate_mb = (byts - last_bytes) / dt / 1e6
            print(f"{pkt_rate:6.0f} pkts/s  {byte_rate_mb:6.3f} MB/s  "
                  f"(total {pkts} pkts, {recv.packets_dropped} dropped)")
            last_pkts, last_bytes, last_t = pkts, byts, now
    except KeyboardInterrupt:
        pass
    finally:
        recv.stop()


if __name__ == "__main__":
    main()
