"""Pretend to be the ESP32: replay a recording over the network.

Use it to test the whole chain (network -> record.py -> SQLite -> SLAM)
without the robot.  Two terminals:

    python -m slam2d.record --db data/test.db          # terminal 1
    python -m slam2d.replay --db data/demo.db --delay 0.05   # terminal 2

It sends exactly the protocol the firmware sends, so if this works your
ESP32 will work too.
"""

from __future__ import annotations

import argparse
import math
import socket
import time
from pathlib import Path

import numpy as np

from . import config
from .db import connect, fetch_odom, fetch_scans


def rebin(scan, bins: int) -> np.ndarray:
    """Put a recorded scan into the (index -> 0..2*pi) convention the ESP32
    uses.  Works whatever the source angle convention was (-pi..pi, 0..2pi...)."""
    out = np.zeros(bins, dtype=np.float32)
    if len(scan.ranges) == bins and abs(float(scan.angles[0])) < 1e-6:
        return np.asarray(scan.ranges, dtype=np.float32)      # already fine
    idx = np.round(np.asarray(scan.angles) * bins / (2 * np.pi)).astype(int) % bins
    for k, i in enumerate(idx):
        r = float(scan.ranges[k])
        if r > 0 and (out[i] == 0 or r < out[i]):
            out[i] = r
    return out


def format_scan(stamp: float, ranges: np.ndarray, bins: int) -> str:
    parts = [f"SCAN {int(stamp * 1000)} {bins}\n"]
    row = []
    for i in range(bins):
        mm = 0 if not np.isfinite(ranges[i]) else int(round(ranges[i] * 1000))
        row.append(f"{i}:{mm}")
        if len(row) == 16:
            parts.append(" ".join(row) + "\n")
            row = []
    if row:
        parts.append(" ".join(row) + "\n")
    parts.append("END\n")
    return "".join(parts)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", default=str(Path(config.DEFAULT_DB).parent / "demo.db"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=config.TCP_PORT)
    ap.add_argument("--delay", type=float, default=0.12,
                    help="seconds between scans (0 = as fast as possible)")
    ap.add_argument("--limit", type=int, default=0, help="stop after N scans")
    ap.add_argument("--with-odom", action="store_true",
                    help="also send the recorded odometry")
    args = ap.parse_args(argv)

    conn = connect(args.db, read_only=True)
    scans = fetch_scans(conn, limit=args.limit or None)
    odom = fetch_odom(conn) if args.with_odom else []
    conn.close()
    if not scans:
        print(f"no scans in {args.db} - run: python -m slam2d.simulate_room")
        return 2

    bins = len(scans[0].ranges)
    with socket.create_connection((args.host, args.port)) as sock:
        print(f"connected to {args.host}:{args.port}, "
              f"replaying {len(scans)} scans of {bins} beams")
        oi = 0
        for n, scan in enumerate(scans):
            sock.sendall(format_scan(scan.stamp, rebin(scan, bins), bins).encode())
            if odom:
                while oi < len(odom) and odom[oi][0] <= scan.stamp:
                    _, x, y, th = odom[oi]
                    sock.sendall(f"ODOM {int(odom[oi][0]*1000)} {x:.4f} {y:.4f} {th:.4f}\n".encode())
                    oi += 1
            if n % 20 == 0:
                print(f"\r  {n}/{len(scans)}", end="", flush=True)
            if args.delay:
                time.sleep(args.delay)
        print(f"\rdone: {len(scans)} scans sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
