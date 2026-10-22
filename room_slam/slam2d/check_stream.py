"""Check the data coming out of the ESP32 - no database, no SLAM.

Run this the first time the ESP32 connects, and whenever the map looks wrong:

    python -m slam2d.check_stream --host 192.168.1.23

It prints how many sweeps per second arrive, how many beams each sweep has,
the distance range, and a crude ASCII picture of the room.  If you can see the
walls in the ASCII picture, your lidar and wiring are fine.
"""

from __future__ import annotations

import argparse
import math
import socket
import time

import numpy as np

from . import config
from .record import ScanParser


def ascii_view(ranges: np.ndarray, width: int = 61, height: int = 21,
               max_range: float = 6.0) -> str:
    """Draw the scan as characters, robot in the middle."""
    grid = [[" "] * width for _ in range(height)]
    cx, cy = width // 2, height // 2
    grid[cy][cx] = "o"
    n = len(ranges)
    for i, r in enumerate(ranges):
        if not np.isfinite(r) or r <= 0 or r > max_range:
            continue
        a = 2 * math.pi * i / n
        px = cx + int(round((r / max_range) * (width // 2 - 1) * math.cos(a)))
        py = cy - int(round((r / max_range) * (height // 2 - 1) * math.sin(a)))
        if 0 <= px < width and 0 <= py < height and grid[py][px] == " ":
            grid[py][px] = "#"
    return "\n".join("".join(row) for row in grid)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0",
                    help="0.0.0.0 = wait for the ESP32 to connect to you")
    ap.add_argument("--port", type=int, default=config.TCP_PORT)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args(argv)

    last = {"ranges": None, "stamp": None}
    counter = {"n": 0}

    def on_scan(scan):
        counter["n"] += 1
        if counter["n"] % 3 == 0:
            last["ranges"] = scan.ranges.copy()
            last["stamp"] = scan.stamp

    parser = ScanParser(bins=360, on_scan=on_scan)

    if args.host in ("0.0.0.0", ""):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", args.port))
        srv.listen(1)
        print(f"listening on 0.0.0.0:{args.port} - power up the ESP32 now ...")
        conn, addr = srv.accept()
        print(f"connected: {addr[0]}")
        srv.close()
    else:
        conn = socket.create_connection((args.host, args.port), timeout=5)
        print(f"connected to {args.host}:{args.port}")

    conn.settimeout(2.0)
    t0 = time.time()
    n0 = 0
    try:
        while time.time() - t0 < args.seconds:
            try:
                data = conn.recv(8192)
            except socket.timeout:
                continue
            if not data:
                print("connection closed by the ESP32")
                break
            parser.feed(data)
    finally:
        conn.close()

    dt = time.time() - t0
    got = counter["n"] - n0
    print(f"\n{got} sweeps in {dt:.1f} s  ->  {got / dt:.2f} Hz")
    if got == 0:
        print("nothing arrived. Check: ESP32 WiFi connected? PC_HOST/PC_PORT "
              "correct? firewall? (see docs/05_troubleshooting.md)")
        return 1

    ranges = last["ranges"]
    if ranges is None:
        return 1
    valid = ranges[(ranges > 0) & np.isfinite(ranges)]
    print(f"beams per sweep : {(ranges > 0).sum()} of {len(ranges)}")
    if len(valid):
        print(f"nearest object  : {valid.min():.2f} m")
        print(f"farthest object : {valid.max():.2f} m")
        print(f"median          : {np.median(valid):.2f} m")
    print("\nview from above (o = robot, # = something the lidar saw):\n")
    print(ascii_view(ranges))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
