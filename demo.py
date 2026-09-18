#!/usr/bin/env python3
"""BitWhisper demo: simulate a thermal transfer between two air-gapped PCs.

Usage:
    PYTHONPATH=. python3 demo.py --payload "hi" --distance 10
"""
from __future__ import annotations

import argparse
import time

from bitwhisper.modem import encode_frame, modulate, demodulate, FrameError
from bitwhisper.physics import ThermalChannel
from bitwhisper.protocol import run_schedule, thermal_ping


def main() -> None:
    ap = argparse.ArgumentParser(description="BitWhisper thermal-channel demo")
    ap.add_argument("--payload", default="hi", help="message to send")
    ap.add_argument("--distance", type=float, default=10.0, help="cm between PCs")
    ap.add_argument("--layout", default="parallel",
                    choices=["parallel", "stacked", "face-away", "quadrature"])
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    payload = args.payload.encode()
    ch = ThermalChannel(args.distance, layout=args.layout, seed=args.seed)

    print(f"[*] pinging for a neighbor at {args.distance} cm ({args.layout})...")
    ping = thermal_ping(ch)
    print(f"    present={ping['present']} rise={ping['rise_c']:.1f}C "
          f"delay={ping['delay_min'] and round(ping['delay_min'], 1)}min")

    # Fresh channel for the transfer: in the real attack the machines cool
    # back down between the ping and the actual transmission.
    ch = ThermalChannel(args.distance, layout=args.layout, seed=args.seed + 1)

    bits = encode_frame(payload)
    sched = modulate(bits)
    print(f"[*] transmitting {len(payload)} bytes as {len(bits)} bits "
          f"({len(sched[-1]) and sched[-1][1] / 60:.1f} h air time)...")
    t0 = time.time()
    samples = run_schedule(ch, sched)
    print(f"    simulated in {time.time() - t0:.1f}s "
          f"({samples[-1][0] / 60:.1f} sim-hours)")

    print("[*] receiving...")
    try:
        rx = demodulate(samples, 7.5, args.distance, args.layout)
    except FrameError as e:
        print(f"    FAILED: {e}")
        return
    ok = "OK" if rx == payload else "MISMATCH"
    print(f"    received {rx!r} [{ok}]")


if __name__ == "__main__":
    main()
