"""Simulated room + robot, so you can run the whole pipeline before the
hardware arrives (and so you know what "good" looks like).

    python -m slam2d.simulate_room --out data/demo.db

It writes a normal recording: lidar scans in ``scans`` and *drifting* wheel
odometry in ``odom``, exactly like the ESP32 bridge would.  Then run

    python -m slam2d.run_slam --db data/demo.db
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from . import config
from .db import Scan, connect, init_db, insert_odom, insert_scan

# A 6 m x 5 m room, origin at the bottom-left corner, plus a table and a sofa.
ROOM = [
    ((0.0, 0.0), (6.0, 0.0)),
    ((6.0, 0.0), (6.0, 5.0)),
    ((6.0, 5.0), (0.0, 5.0)),
    ((0.0, 5.0), (0.0, 0.0)),
    # furniture / clutter (closed boxes)
    ((2.6, 1.9), (3.7, 1.9)), ((3.7, 1.9), (3.7, 2.7)),
    ((3.7, 2.7), (2.6, 2.7)), ((2.6, 2.7), (2.6, 1.9)),
    ((1.3, 3.0), (2.2, 3.0)), ((2.2, 3.0), (2.2, 3.6)),
    ((2.2, 3.6), (1.3, 3.6)), ((1.3, 3.6), (1.3, 3.0)),
]


def raycast(origin: np.ndarray, angles: np.ndarray,
            segments=ROOM, max_range: float = 12.0) -> np.ndarray:
    """Distance from ``origin`` to the nearest wall along each angle."""
    n = len(angles)
    best = np.full(n, np.inf)
    dx, dy = np.cos(angles), np.sin(angles)
    for (ax, ay), (bx, by) in segments:
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex                     # cross(d, e)
        ok = np.abs(denom) > 1e-12
        if not np.any(ok):
            continue
        ox, oy = ax - origin[0], ay - origin[1]
        t = np.full(n, np.inf)
        u = np.full(n, np.inf)
        t[ok] = (ox * ey - oy * ex) / denom[ok]
        u[ok] = (ox * dy[ok] - oy * dx[ok]) / denom[ok]
        hit = ok & (t > 1e-6) & (u >= 0.0) & (u <= 1.0) & (t < max_range)
        best = np.where(hit & (t < best), t, best)
    best[~np.isfinite(best)] = 0.0
    return best


def _closed_rounded_path(corners, radius: float, n: int) -> np.ndarray:
    """Sample a closed polygon with rounded corners at n equally spaced points."""
    corners = np.asarray(corners, dtype=float)
    m = len(corners)
    dense = []
    for i in range(m):
        p_prev, p, p_next = corners[i - 1], corners[i], corners[(i + 1) % m]
        d_in = p - p_prev
        d_in /= np.linalg.norm(d_in)
        d_out = p_next - p
        d_out /= np.linalg.norm(d_out)
        a = p - d_in * radius
        b = p + d_out * radius
        for t in np.linspace(0.0, 1.0, 14)[:-1]:
            # quadratic Bezier: cuts the corner smoothly
            dense.append((1 - t) ** 2 * a + 2 * (1 - t) * t * p + t ** 2 * b)
    dense = np.asarray(dense)
    dense = np.vstack([dense, dense[0]])
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    want = np.linspace(0.0, arc[-1], n, endpoint=False)
    xy = np.column_stack((np.interp(want, arc, dense[:, 0]),
                          np.interp(want, arc, dense[:, 1])))
    return xy


def robot_path(n: int = 240):
    """A closed lap around the room, with the robot looking where it drives.

    A *closed* lap matters: the last scan is taken from the first pose with the
    same heading, which is exactly the situation a loop closure has to detect.
    Returns (xy, yaw).
    """
    xy = _closed_rounded_path([(0.9, 0.8), (5.2, 0.8),
                               (5.2, 4.2), (0.9, 4.2)], radius=0.55, n=n)
    nxt = np.roll(xy, -1, axis=0)
    yaw = np.arctan2(nxt[:, 1] - xy[:, 1], nxt[:, 0] - xy[:, 0])
    return xy, yaw


def write_demo_scan(path: Path, seed: int = 3, n_scans: int = 160,
                    n_beams: int = 360) -> Path:
    rng = np.random.default_rng(seed)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()
    for suffix in ("-wal", "-shm"):
        extra = Path(str(path) + suffix)
        if extra.exists():
            extra.unlink()

    conn = connect(path)
    init_db(conn)

    path_xy, yaws = robot_path(n_scans)

    angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
    dt = 0.12                                   # ~8 Hz
    odom_x = odom_y = odom_yaw = 0.0
    prev = path_xy[0]

    for k, (xy, yaw) in enumerate(zip(path_xy, yaws)):
        stamp = round(k * dt, 3)
        ranges = raycast(xy, angles + yaw)

        noisy = ranges.copy()
        hit = noisy > 0
        noisy[hit] += rng.normal(0.0, 0.012, hit.sum())
        # realistic RPLIDAR behaviour: a few beams drop out
        dropout = rng.random(len(noisy)) < 0.02
        noisy[dropout & hit] = 0.0
        noisy[(noisy < config.MIN_RANGE_M) | (noisy > config.MAX_RANGE_M)] = 0.0

        insert_scan(conn, Scan(stamp=stamp,
                               angles=angles.astype(np.float32),
                               ranges=noisy.astype(np.float32)))

        # deliberately imperfect odometry: 3 % scale error + heading drift
        step = np.hypot(*(xy - prev))
        turn = (yaws[k] - yaws[k - 1]) if k else 0.0
        odom_yaw += turn + rng.normal(0.0, 0.004)
        odom_x += step * 1.03 * math.cos(odom_yaw)
        odom_y += step * 1.03 * math.sin(odom_yaw)
        prev = xy
        insert_odom(conn, stamp, odom_x, odom_y, odom_yaw, source="sim")

    conn.close()
    print(f"wrote {n_scans} simulated scans to {path}")
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(Path(config.DEFAULT_DB).parent / "demo.db"))
    ap.add_argument("--scans", type=int, default=160)
    ap.add_argument("--beams", type=int, default=360)
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()
    write_demo_scan(Path(args.out), seed=args.seed, n_scans=args.scans,
                    n_beams=args.beams)


if __name__ == "__main__":
    main()
