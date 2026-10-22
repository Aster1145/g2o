"""SQLite storage: a tiny recorder for lidar scans and odometry.

The database is the *contract* between the three parts of the system:

    ESP32  --TCP-->  bridge/record.py  --SQL-->  room.db  --SQL-->  run_slam.py

That means you can move files around, replay yesterday's walk, or test SLAM
changes without touching the robot again.
"""

from __future__ import annotations

import math
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SCHEMA_PATH = Path(__file__).resolve().parents[1] / "sql" / "schema.sql"


@dataclass
class Scan:
    """One 360 degree sweep, in the lidar's own frame."""

    stamp: float
    angles: np.ndarray          # radians, one per beam
    ranges: np.ndarray          # metres, 0 or NaN = no return
    quality: np.ndarray | None = None
    id: int | None = None
    node_id: int | None = None
    extra: dict = field(default_factory=dict)

    @property
    def valid(self) -> np.ndarray:
        r = self.ranges
        return np.isfinite(r) & (r > 0.0)

    def points_in_lidar_frame(self) -> np.ndarray:
        """Beam end points in the lidar frame, shape (N, 2)."""
        m = self.valid
        r = self.ranges[m].astype(float)
        a = self.angles[m].astype(float)
        return np.column_stack((r * np.cos(a), r * np.sin(a)))


# --------------------------------------------------------------------------
# connection helpers
# --------------------------------------------------------------------------
def connect(path: str | Path, read_only: bool = False) -> sqlite3.Connection:
    if read_only:
        uri = f"file:{Path(path).resolve()}?mode=ro"
        conn = sqlite3.connect(uri, uri=True, check_same_thread=False)
    else:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(str(path), check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")
    return conn


def init_db(conn: sqlite3.Connection) -> None:
    conn.executescript(SCHEMA_PATH.read_text())
    conn.commit()


# --------------------------------------------------------------------------
# writing
# --------------------------------------------------------------------------
def insert_scan(conn: sqlite3.Connection, scan: Scan) -> int | None:
    """Insert a scan.  Returns the new row id, or None if it was a duplicate."""
    ranges = np.asarray(scan.ranges, dtype=np.float32)
    angles = np.asarray(scan.angles, dtype=np.float32)
    quality = (np.asarray(scan.quality, dtype=np.uint8)
               if scan.quality is not None else None)
    cur = conn.execute(
        "INSERT OR IGNORE INTO scans (stamp, n_points, ranges, angles, quality) "
        "VALUES (?, ?, ?, ?, ?)",
        (float(scan.stamp), int(len(ranges)), ranges.tobytes(),
         angles.tobytes(), quality.tobytes() if quality is not None else None),
    )
    conn.commit()
    return cur.lastrowid if cur.rowcount else None


def insert_odom(conn: sqlite3.Connection, stamp: float, x: float, y: float,
                theta: float, source: str = "wheel") -> None:
    conn.execute(
        "INSERT INTO odom (stamp, x, y, theta, source) VALUES (?, ?, ?, ?, ?)",
        (float(stamp), float(x), float(y), float(theta), source),
    )
    conn.commit()


# --------------------------------------------------------------------------
# reading
# --------------------------------------------------------------------------
def _row_to_scan(row: sqlite3.Row) -> Scan:
    ranges = np.frombuffer(row["ranges"], dtype=np.float32).copy()
    angles = np.frombuffer(row["angles"], dtype=np.float32).copy()
    quality = None
    if row["quality"] is not None:
        quality = np.frombuffer(row["quality"], dtype=np.uint8).copy()
    key = "node_id" if "node_id" in row.keys() else None
    return Scan(stamp=row["stamp"], angles=angles, ranges=ranges,
                quality=quality, id=row["id"],
                node_id=row[key] if key else None)


def fetch_scans(conn: sqlite3.Connection, limit: int | None = None) -> list[Scan]:
    sql = "SELECT id, stamp, ranges, angles, quality FROM scans ORDER BY stamp"
    if limit:
        sql += f" LIMIT {int(limit)}"
    return [_row_to_scan(r) for r in conn.execute(sql)]


def fetch_odom(conn: sqlite3.Connection) -> list[tuple[float, float, float, float]]:
    """Return [(stamp, x, y, theta), ...] ordered by time."""
    return [(r["stamp"], r["x"], r["y"], r["theta"])
            for r in conn.execute("SELECT stamp, x, y, theta FROM odom ORDER BY stamp")]


def fetch_graph(conn: sqlite3.Connection):
    """Return (poses dict, edges list, meta dict) from a previous SLAM run."""
    poses = {}
    for r in conn.execute(
            "SELECT node_id, scan_id, stamp, x, y, theta FROM poses ORDER BY node_id"):
        poses[r["node_id"]] = (r["x"], r["y"], r["theta"], r["scan_id"], r["stamp"])
    edges = [(r["from_node"], r["to_node"], r["kind"])
             for r in conn.execute(
                 "SELECT from_node, to_node, kind FROM graph_edges ORDER BY id")]
    meta = {r["key"]: r["value"] for r in conn.execute("SELECT key, value FROM map_meta")}
    return poses, edges, meta


# --------------------------------------------------------------------------
# writing results back
# --------------------------------------------------------------------------
def save_graph(conn: sqlite3.Connection, poses: list[tuple], edges: list[tuple],
               meta: dict | None = None) -> None:
    """Replace the SLAM result tables.

    poses : [(node_id, scan_id, stamp, x, y, theta), ...]
    edges : [(from_node, to_node, dx, dy, dtheta, kind, chi2), ...]
    """
    with conn:
        conn.execute("DELETE FROM poses")
        conn.execute("DELETE FROM graph_edges")
        conn.executemany(
            "INSERT INTO poses (node_id, scan_id, stamp, x, y, theta, kind) "
            "VALUES (?, ?, ?, ?, ?, ?, 'slam')", poses)
        conn.executemany(
            "INSERT INTO graph_edges (from_node, to_node, dx, dy, dtheta, kind, chi2) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)", edges)
        if meta:
            conn.executemany(
                "INSERT INTO map_meta (key, value) VALUES (?, ?) "
                "ON CONFLICT(key) DO UPDATE SET value = excluded.value",
                [(str(k), str(v)) for k, v in meta.items()])


def set_meta(conn: sqlite3.Connection, **kwargs) -> None:
    with conn:
        conn.executemany(
            "INSERT INTO map_meta (key, value) VALUES (?, ?) "
            "ON CONFLICT(key) DO UPDATE SET value = excluded.value",
            [(str(k), str(v)) for k, v in kwargs.items()])


def stats(conn: sqlite3.Connection) -> dict:
    n_scans = conn.execute("SELECT COUNT(*) FROM scans").fetchone()[0]
    n_odom = conn.execute("SELECT COUNT(*) FROM odom").fetchone()[0]
    return {"scans": n_scans, "odom": n_odom}


def synthetic_demo(path: str | Path, seed: int = 3) -> None:
    """Write a tiny simulated room into ``path`` so you can try the toolchain
    before the hardware arrives.  See ``slam2d/simulate_room.py``."""
    from . import simulate_room  # local import: only needed for the demo

    simulate_room.write_demo_scan(Path(path), seed=seed)
