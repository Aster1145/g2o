"""Record an RPLIDAR plugged straight into a USB port - no ESP32, no WiFi.

    python -m slam2d.record_lidar_usb --port COM8 --db data/room.db
    python -m slam2d.record_lidar_usb --port COM8 --preview      # just look
    python -m slam2d.record_lidar_usb --port /dev/ttyUSB0 --limit 200

This talks to the lidar with the official Slamtec SDK (``pip install rplidar``),
which also drives the motor (it toggles DTR, which is how the A1's USB adapter
turns the motor on).  Sweeps are written to the *same* SQLite schema as the
ESP32 path, so the rest of the pipeline is unchanged:

    python -m slam2d.run_slam --db data/room.db --name room

Because a USB lidar gives us no wheel odometry, SLAM automatically switches to
the likelihood-field pose search (see ``slam2d/likelihood.py``), which does not
need a motion prior.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

from . import config
from .db import Scan, connect, init_db, insert_scan, stats

__all__ = ["measurments_to_scan", "record", "main"]


def measurments_to_scan(measurments, bins: int = 360, angle_sign: float = 1.0,
                        stamp: float = 0.0) -> Scan:
    """Convert SDK measure tuples ``(quality, angle_deg, dist_mm)`` to a Scan.

    The SDK reports angles in degrees, clockwise from the front.  We bin them
    into ``bins`` evenly spaced directions (1 degree at the default 360) and
    keep the *nearest* return per direction: if two surfaces are visible along
    the same bearing, the closer one is the obstacle the robot could hit.
    """
    ranges = np.zeros(bins, dtype=np.float32)
    quality = np.zeros(bins, dtype=np.uint8)
    for q, angle_deg, dist_mm in measurments:
        if dist_mm is None or dist_mm <= 0:
            continue
        r = dist_mm / 1000.0
        if r < config.MIN_RANGE_M or r > config.MAX_RANGE_M:
            continue
        idx = int(round(angle_deg * bins / 360.0)) % bins
        if ranges[idx] == 0.0 or r < ranges[idx]:
            ranges[idx] = r
            quality[idx] = int(q) & 0xFF

    angles = 2.0 * math.pi * np.arange(bins) / bins
    if angle_sign < 0:                     # lidar mounted upside down
        angles = -angles[::-1].copy()
    return Scan(stamp=stamp, angles=angles.astype(np.float32),
                ranges=ranges, quality=quality)


def record(lidar, db_path, bins: int = 360, angle_sign: float = 1.0,
           limit: int = 0, on_scan=None):
    """Consume ``lidar.iter_scans()`` and store each sweep.

    Returns ``(number of sweeps stored, seconds elapsed)``.
    """
    conn = connect(db_path)
    init_db(conn)
    n = 0
    t0 = time.time()
    last_report = 0.0
    last_stamp = -1.0
    try:
        for measurments in lidar.iter_scans(min_len=int(bins * 0.4)):
            stamp = round(time.time() - t0, 3)
            # the database keys scans by timestamp; never write twice the same
            if stamp <= last_stamp:
                stamp = round(last_stamp + 0.001, 3)
            last_stamp = stamp
            scan = measurments_to_scan(measurments, bins=bins,
                                       angle_sign=angle_sign, stamp=stamp)
            if scan.valid.sum() < 20:      # motor still spinning up
                continue
            insert_scan(conn, scan)
            n += 1
            if on_scan:
                on_scan(scan, n)
            now = time.time()
            if now - last_report > 1.0:
                last_report = now
                rate = n / max(now - t0, 1e-6)
                print(f"\r{n:6d} sweeps  {rate:5.2f} Hz  "
                      f"beams={int(scan.valid.sum()):3d}/{bins}   ",
                      end="", flush=True)
            if limit and n >= limit:
                break
    finally:
        conn.close()
        print()
    return n, time.time() - t0


def list_ports() -> list[str]:
    """Show the serial ports the OS knows about (helps find COM8)."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    out = []
    for p in lp.comports():
        out.append(f"{p.device}  {p.description}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COM8, /dev/ttyUSB0, ... (see --list-ports)")
    ap.add_argument("--db", default=config.DEFAULT_DB)
    ap.add_argument("--bins", type=int, default=360)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--angle-sign", type=float, default=1.0,
                    help="-1 if the finished map comes out mirrored")
    ap.add_argument("--limit", type=int, default=0,
                    help="stop after N sweeps (0 = until Ctrl-C)")
    ap.add_argument("--preview", action="store_true",
                    help="grab a few sweeps and draw the room as ASCII, then exit")
    ap.add_argument("--list-ports", action="store_true",
                    help="print the serial ports and exit")
    args = ap.parse_args(argv)

    if args.list_ports or not args.port:
        ports = list_ports()
        if ports:
            print("serial ports found:")
            for p in ports:
                print("  " + p)
        else:
            print("no serial ports found (is the lidar plugged in? "
                  "is the CP210x / CH340 driver installed?)")
        if not args.port:
            print("\nusage: python -m slam2d.record_lidar_usb --port COM8")
            return 2 if not ports else 0

    try:
        from rplidar import RPLidar
    except ImportError:
        print("the Slamtec SDK is missing:  pip install rplidar", file=sys.stderr)
        return 2

    limit = 5 if args.preview else args.limit
    latest = {}

    def on_scan(scan, n):
        latest["scan"] = scan

    print(f"opening {args.port} (Ctrl-C to stop) ...")
    lidar = RPLidar(args.port, baudrate=args.baud, timeout=3)
    n = 0
    try:
        try:
            print(f"  info   : {lidar.get_info()}")
            health = lidar.get_health()
            print(f"  health : {health[0]}")
            if health[0] != "Good":
                print("  WARNING: health is not 'Good' - check the 5 V supply "
                      "and the USB cable")
        except Exception as exc:
            print(f"  (could not query the device: {exc})")
        lidar.start_motor()                # A1: pulls DTR low -> motor spins
        time.sleep(1.0)                    # let it reach speed
        n, _ = record(lidar, args.db, bins=args.bins,
                      angle_sign=args.angle_sign, limit=limit, on_scan=on_scan)
    except KeyboardInterrupt:
        print("\nstopped by user")
    except Exception as exc:
        print(f"\nerror talking to the lidar: {exc}")
        print("  * is the port really COM8?  try: --list-ports")
        print("  * is another program (Arduino IDE, RPLIDAR tool) holding it?")
        print("  * is the USB cable a data cable, not a charge-only one?")
        return 1
    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass

    if args.preview:
        if "scan" in latest:
            from .check_stream import ascii_view
            print("\nview from above (o = robot, # = something the lidar saw):\n")
            print(ascii_view(latest["scan"].ranges))
            print("\nIf this does not look like your room, fix the hardware "
                  "before recording a walk.")
        return 0 if n else 1

    print(f"{n} sweeps written to {args.db}")
    if n:
        conn = connect(args.db, read_only=True)
        try:
            print(f"database now holds: {stats(conn)}")
        finally:
            conn.close()
        print(f"\nnow build the map:\n"
              f"  python -m slam2d.run_slam --db {args.db} --name room --show")
    return 0 if n else 1


if __name__ == "__main__":
    raise SystemExit(main())
