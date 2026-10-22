"""Receive lidar scans from the ESP32 and store them in SQLite.

Two ways to get the data in:

    WiFi (recommended)          USB cable / USB-TTL adapter
    ------------------          ---------------------------
    python -m slam2d.record     python -m slam2d.record --source serial \\
        --db data/room.db           --port COM5 --db data/room.db

The ESP32 speaks a tiny text protocol (firmware/esp32_rplidar_bridge):

    SCAN <stamp_ms> <n>
      <idx0>:<dist_mm> <idx1>:<dist_mm> ...
      (16 pairs per line, so the parser is line-buffered and cheap)
    END
    ODOM <stamp_ms> <x_m> <y_m> <theta_rad>     <- optional
    # anything else is a comment / status line

``idx`` is the beam index 0..357 (clockwise or counter-clockwise, whichever the
firmware says - see LIDAR_BINS and the docs), ``dist_mm`` is 0 for "no return".

Press Ctrl-C to stop.  The database is safe to read while this is running.
"""

from __future__ import annotations

import argparse
import math
import signal
import socket
import socketserver
import sys
import threading
import time
from pathlib import Path

import numpy as np

from . import config
from .db import Scan, connect, init_db, insert_odom, insert_scan

LIDAR_BINS = 360          # angular resolution of the stored scan


# --------------------------------------------------------------------------
# protocol parsing
# --------------------------------------------------------------------------
class ScanParser:
    """Turns the ESP32 byte stream into Scan objects."""

    def __init__(self, bins: int = LIDAR_BINS, on_scan=None, on_odom=None):
        self.bins = bins
        self.on_scan = on_scan
        self.on_odom = on_odom
        self.buf = ""
        self.state = "idle"
        self.stamp = 0.0
        self.pairs: list[str] = []
        self.expected = 0

    # ----------------------------------------------------------------
    def feed(self, data: bytes) -> None:
        try:
            text = data.decode("ascii", errors="ignore")
        except Exception:
            return
        self.buf += text
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self._line(line.strip())

    def flush(self) -> None:
        if self.state == "collect" and self.pairs:
            self._finish()

    # ----------------------------------------------------------------
    def _line(self, line: str) -> None:
        if not line:
            return
        if line.startswith("#"):
            return
        tokens = line.split()

        for tok in tokens:
            if tok == "SCAN":
                self._abort()
                self.state = "header"
                self._header = []
                continue

            if tok == "END":
                if self.state == "collect":
                    self._finish()
                continue

            if tok == "ODOM":
                self._abort()
                self.state = "odom"
                continue

            if self.state == "header":
                self._header.append(tok)
                if len(self._header) == 2:
                    self.stamp = float(self._header[0]) / 1000.0
                    self.expected = int(self._header[1])
                    self.pairs = []
                    self.state = "collect"
                continue

            if self.state == "odom":
                self._odom_tokens = getattr(self, "_odom_tokens", []) + [tok]
                if len(self._odom_tokens) == 4:
                    st, x, y, th = (float(v) for v in self._odom_tokens)
                    st /= 1000.0            # protocol sends milliseconds
                    self._odom_tokens = []
                    self.state = "idle"
                    if self.on_odom:
                        self.on_odom(st, x, y, th)
                continue

            if self.state == "collect":
                self.pairs.append(tok)
                if self.expected and len(self.pairs) >= self.expected:
                    self._finish()

    # ----------------------------------------------------------------
    def _abort(self) -> None:
        self.state = "idle"
        self.pairs = []
        self.expected = 0

    def _finish(self) -> None:
        pairs, self.pairs = self.pairs, []
        self.state = "idle"
        self.expected = 0
        if not pairs:
            return
        ranges = np.zeros(self.bins, dtype=np.float32)
        seen = np.zeros(self.bins, dtype=bool)
        for token in pairs:
            try:
                idx_s, dist_s = token.split(":")
                idx = int(idx_s) % self.bins
                mm = int(dist_s)
            except ValueError:
                continue
            if mm <= 0:
                continue
            ranges[idx] = mm / 1000.0
            seen[idx] = True
        if seen.sum() < 20:
            return
        angles = (2.0 * math.pi * np.arange(self.bins) / self.bins)
        if config.LIDAR_ANGLE_SIGN < 0:                 # mirrored mounting
            angles = -angles[::-1]
        if config.LIDAR_ANGLE_OFFSET_DEG:
            angles = angles + math.radians(config.LIDAR_ANGLE_OFFSET_DEG)
        angles = angles.astype(np.float32)
        scan = Scan(stamp=self.stamp, angles=angles, ranges=ranges)
        scan.extra["coverage"] = int(seen.sum())
        if self.on_scan:
            self.on_scan(scan)


# --------------------------------------------------------------------------
# recorder
# --------------------------------------------------------------------------
class Recorder:
    def __init__(self, db_path: str | Path, bins: int = LIDAR_BINS,
                 snapshot_every: int = 0, snapshot_dir: str = "maps/live"):
        self.db_path = Path(db_path)
        self.conn = connect(self.db_path)
        init_db(self.conn)
        self.bins = bins
        self.n_scans = 0
        self.n_odom = 0
        self.t0 = time.time()
        self.last_stamp = None      # first scan sets the clock
        self.offset = 0.0
        self.last_print = 0.0
        self.lock = threading.Lock()
        self.snapshot_every = snapshot_every
        self.snapshot_dir = snapshot_dir
        self.parser = ScanParser(bins=bins, on_scan=self.add_scan,
                                 on_odom=self.add_odom)

    # ----------------------------------------------------------------
    def add_scan(self, scan: Scan) -> None:
        with self.lock:
            stamp = scan.stamp + self.offset
            if self.last_stamp is not None and stamp <= self.last_stamp:
                # the ESP32 restarted (millis() back to 0): keep the database
                # timestamps monotonically increasing
                self.offset += self.last_stamp - stamp + 0.01
                stamp = scan.stamp + self.offset
            self.last_stamp = stamp
            scan.stamp = stamp
            row = insert_scan(self.conn, scan)
            if row is not None:
                self.n_scans += 1
            self._maybe_report(scan)
            if self.snapshot_every and self.n_scans % self.snapshot_every == 0:
                self._snapshot()

    def add_odom(self, stamp: float, x: float, y: float, theta: float) -> None:
        with self.lock:
            insert_odom(self.conn, stamp + self.offset, x, y, theta, source="esp32")
            self.n_odom += 1

    # ----------------------------------------------------------------
    def _maybe_report(self, scan: Scan) -> None:
        now = time.time()
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        rate = self.n_scans / max(now - self.t0, 1e-6)
        cov = scan.extra.get("coverage", 0)
        print(f"\r{self.n_scans:6d} scans  {rate:5.2f} Hz  "
              f"beams={cov:3d}/{self.bins}  odom={self.n_odom:5d}   ",
              end="", flush=True)

    def _snapshot(self) -> None:
        """Rebuild the map in the background while recording."""
        print(f"\n[snapshot] rebuilding map from {self.n_scans} scans ...")
        try:
            from .slam import run_slam, save_results
            res = run_slam(self.db_path, use_odom=self.n_odom > 0,
                           verbose=False)
            files = save_results(res, out_dir=self.snapshot_dir,
                                 db_path=None, name="live")
            print(f"[snapshot] {files['png']} "
                  f"({res.stats['keyframes']} keyframes)\n")
        except SystemExit as exc:
            print(f"[snapshot] skipped: {exc}\n")
        except Exception as exc:                       # pragma: no cover
            print(f"[snapshot] failed: {exc}\n")

    def close(self) -> None:
        self.parser.flush()
        self.conn.close()


# --------------------------------------------------------------------------
# transports
# --------------------------------------------------------------------------
def serve_tcp(recorder: Recorder, host: str, port: int) -> None:
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            peer = f"{self.client_address[0]}:{self.client_address[1]}"
            print(f"[tcp] ESP32 connected from {peer}")
            self.request.settimeout(30.0)
            try:
                while True:
                    data = self.request.recv(4096)
                    if not data:
                        break
                    recorder.parser.feed(data)
            except socket.timeout:
                print(f"\n[tcp] {peer} idle for 30 s, closing")
            except ConnectionResetError:
                pass
            finally:
                recorder.parser.flush()
                print(f"\n[tcp] {peer} disconnected")

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    with Server((host, port), Handler) as server:
        print(f"[tcp] listening on {host}:{port}")
        print(f"[tcp] put this in the ESP32 firmware:  PORT = {port}")
        print(f"[tcp] (your laptop's LAN IP is what the ESP32 needs - "
              f"run `ipconfig` / `ip addr` if unsure)")
        server.serve_forever()


def serve_serial(recorder: Recorder, port: str, baud: int) -> None:
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is not installed:  pip install pyserial")
    with serial.Serial(port, baud, timeout=1.0) as ser:
        print(f"[serial] reading {port} @ {baud} baud")
        while True:
            data = ser.read(4096)
            if data:
                recorder.parser.feed(data)


# --------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", default=config.DEFAULT_DB, help="SQLite file to write")
    ap.add_argument("--source", default="tcp", choices=["tcp", "serial"])
    ap.add_argument("--host", default="0.0.0.0", help="bind address for --source tcp")
    ap.add_argument("--port", default=str(config.TCP_PORT),
                    help="TCP port, or serial device name for --source serial")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bins", type=int, default=LIDAR_BINS,
                    help="number of angular bins in the stream (default 360)")
    ap.add_argument("--snapshot-every", type=int, default=0,
                    help="rebuild maps/live/live.png every N scans (0 = off)")
    args = ap.parse_args(argv)

    recorder = Recorder(args.db, bins=args.bins,
                        snapshot_every=args.snapshot_every)

    def _stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _stop)

    print(f"[db] writing to {Path(args.db).resolve()}")
    try:
        if args.source == "tcp":
            serve_tcp(recorder, args.host, int(args.port))
        else:
            serve_serial(recorder, args.port, args.baud)
    except KeyboardInterrupt:
        print("\n[stop] shutting down")
    finally:
        recorder.close()
        print(f"\n[done] {recorder.n_scans} scans, {recorder.n_odom} odometry "
              f"samples -> {Path(args.db).resolve()}")
        print("       now run:  python -m slam2d.run_slam --db "
              f"{args.db} --show")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
