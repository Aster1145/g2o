"""The SLAM pipeline: scan matching -> pose graph -> occupancy map.

Read this top to bottom and you have the whole algorithm:

    1. every keyframe scan is matched (ICP) against a local submap of the
       recent scans -> a refined pose and a "odometry" edge;
    2. when the robot comes back near an old pose, the scan is also matched
       against that old neighbourhood -> a "loop closure" edge;
    3. g2o spreads the loop-closure error back over the whole trajectory, which
       un-bends the map;
    4. all scans are re-drawn at the optimised poses into one occupancy grid.

You can run it on recorded data as many times as you like - the recording is
in SQLite, the SLAM is stateless.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import config, graph, occupancy_map
from .db import Scan, connect, fetch_odom, fetch_scans
from .geometry import (angle_diff, heading, inverse, normalize_angle, relative,
                       se2, transform_points, translation)
from .icp import icp


@dataclass
class SlamResult:
    poses: list[np.ndarray]
    scans: list[Scan]
    edges: list[graph.Edge]
    grid: occupancy_map.OccMap
    stats: dict = field(default_factory=dict)

    def world_points(self) -> list[np.ndarray]:
        out = []
        for T, scan in zip(self.poses, self.scans):
            pts = scan.points_in_lidar_frame()
            out.append(transform_points(T, pts))
        return out

    def trajectory(self) -> np.ndarray:
        return np.array([translation(T) for T in self.poses])


# --------------------------------------------------------------------------
def _sensor_transform(x: float, y: float, yaw: float) -> np.ndarray:
    """Robot pose -> lidar pose (where the beams actually start)."""
    return se2(x, y, yaw) @ se2(config.LIDAR_OFFSET_X, config.LIDAR_OFFSET_Y,
                                config.LIDAR_OFFSET_YAW)


def _odom_lookup(odom, stamp: float):
    """Linear interpolation of the odometry pose at ``stamp``."""
    if not odom:
        return None
    times = [o[0] for o in odom]
    if stamp <= times[0]:
        o = odom[0]
        return se2(o[1], o[2], o[3])
    if stamp >= times[-1]:
        o = odom[-1]
        return se2(o[1], o[2], o[3])
    k = int(np.searchsorted(times, stamp))
    t0, x0, y0, a0 = odom[k - 1]
    t1, x1, y1, a1 = odom[k]
    w = 0.0 if t1 == t0 else (stamp - t0) / (t1 - t0)
    return se2(x0 + w * (x1 - x0), y0 + w * (y1 - y0),
               a0 + w * normalize_angle(a1 - a0))


def _crop(cloud: np.ndarray, centre: np.ndarray, radius: float) -> np.ndarray:
    if len(cloud) == 0:
        return cloud
    d = cloud[:, :2] - centre[:2]
    return cloud[(d[:, 0] ** 2 + d[:, 1] ** 2) < radius * radius]


def _keyframe_wanted(last_pose, pose, min_dist, min_rot) -> bool:
    d = np.hypot(pose[0, 2] - last_pose[0, 2], pose[1, 2] - last_pose[1, 2])
    a = abs(angle_diff(heading(pose), heading(last_pose)))
    return d >= min_dist or a >= min_rot


# --------------------------------------------------------------------------
def run_slam(db_path: str | Path, use_odom: bool = True,
             max_scans: int | None = None, loop_closure: bool = True,
             optimize_graph: bool = True, backend: str = "auto",
             verbose: bool = True, lidar_offsets: np.ndarray | None = None,
             progress=None) -> SlamResult:
    """Run the full pipeline on the scans stored in ``db_path``."""
    t_start = time.time()
    conn = connect(db_path, read_only=True)
    scans = fetch_scans(conn, limit=max_scans)
    odom = fetch_odom(conn) if use_odom else []
    conn.close()
    if len(scans) < 2:
        raise SystemExit(f"Only {len(scans)} scan(s) in {db_path}. "
                         "Record more data first (see docs/02_record_data.md).")

    if lidar_offsets is None:
        lidar_offsets = se2(config.LIDAR_OFFSET_X, config.LIDAR_OFFSET_Y,
                            config.LIDAR_OFFSET_YAW)

    poses: list[np.ndarray] = []                 # optimised-so-far robot poses
    keys: list[Scan] = []                        # keyframe scans
    keys_clouds: list[np.ndarray] = []           # their points in world frame
    edges: list[graph.Edge] = []
    loops: list[tuple[int, int, float]] = []
    last_motion = np.eye(3)

    def report(msg):
        if verbose:
            print(msg, flush=True)
        if progress:
            progress(msg)

    # ---------------- 1. sequential scan matching ----------------
    n_matched = n_rejected = 0
    first = scans[0]
    T = np.eye(3)
    poses.append(T)
    keys.append(first)
    keys_clouds.append(transform_points(T @ lidar_offsets,
                                        first.points_in_lidar_frame()))

    for k, scan in enumerate(scans[1:], start=1):
        if max_scans and len(keys) >= max_scans:
            break

        # --- predict the new pose -------------------------------------
        T_guess = poses[-1] @ last_motion
        odom_pose = _odom_lookup(odom, scan.stamp) if odom else None
        if odom_pose is not None and len(keys) >= 2:
            # odometry gives us the incremental motion since the previous scan
            prev_odom = _odom_lookup(odom, keys[-1].stamp)
            if prev_odom is not None:
                T_guess = poses[-1] @ relative(prev_odom, odom_pose)

        pts_local = scan.points_in_lidar_frame()
        if len(pts_local) < 30:
            n_rejected += 1
            continue
        # only points that can possibly have a partner in the submap count as
        # matchable - otherwise the inlier fraction is meaningless
        pts_match = _crop(pts_local, np.zeros(2), config.SUBMAP_RADIUS_M)
        if len(pts_match) < 30:
            pts_match = pts_local

        # --- match against the recent submap --------------------------
        centre = translation(T_guess)
        submap = np.vstack([_crop(c, centre, config.SUBMAP_RADIUS_M)
                            for c in keys_clouds[-config.SUBMAP_NUM_KEYFRAMES:]])
        if len(submap) < 50:
            poses.append(T_guess)
            last_motion = inverse(poses[-2]) @ T_guess
            continue

        # start the alignment at the odometry prediction: ICP only has to
        # polish the small error that is left
        result = icp(pts_match, submap, T_init=T_guess)
        T_new = result.transform

        ok, why = _accept_match(result, T_guess, T_new)
        if not ok:
            n_rejected += 1
            if k % 10 == 0:
                report(f"  scan {k:4d}: match rejected ({why}, "
                       f"err={result.mean_error:.3f} m, "
                       f"inliers={result.inlier_fraction:.2f})")
            continue

        n_matched += 1
        last_motion = inverse(poses[-1]) @ T_new

        # --- is this scan worth keeping as a keyframe? ----------------
        if not _keyframe_wanted(poses[-1], T_new,
                                config.KEYFRAME_MIN_DIST_M,
                                config.KEYFRAME_MIN_ROT_RAD):
            continue

        poses.append(T_new)
        keys.append(scan)
        keys_clouds.append(transform_points(T_new @ lidar_offsets, pts_local))

        i = len(poses) - 2
        j = len(poses) - 1
        edges.append(graph.Edge(i, j, relative(poses[i], poses[j]),
                                config.ODOM_SIGMA_XY, config.ODOM_SIGMA_THETA,
                                "odom"))

        # ---------------- 2. loop closure ----------------
        if loop_closure and len(poses) > config.LOOP_MIN_NODE_GAP + 2:
            cand = _find_loop_candidate(poses, j)
            if cand is not None:
                c = cand
                loop_cloud = np.vstack([
                    _crop(keys_clouds[m], translation(poses[c]),
                          config.LOOP_SUBMAP_RADIUS_M)
                    for m in range(max(0, c - 8), min(len(keys_clouds), c + 9))])
                if len(loop_cloud) < 50:
                    loop_cloud = keys_clouds[c]
                # propose a small correction but stay in our own frame
                res2 = icp(pts_local, loop_cloud, T_init=T_new)
                T2 = res2.transform
                if (res2.mean_error < config.LOOP_ACCEPT_ERROR_M and
                        res2.inlier_fraction >= config.LOOP_ACCEPT_INLIER_FRACTION):
                    T_ij = relative(poses[c], T2)
                    edges.append(graph.Edge(
                        c, j, T_ij, config.LOOP_SIGMA_XY,
                        config.LOOP_SIGMA_THETA, "loop"))
                    loops.append((c, j, res2.mean_error))
                    report(f"  scan {k:4d}: LOOP CLOSURE node {c} <-> {j} "
                           f"(err={res2.mean_error:.3f} m)")

    if len(poses) < 2:
        raise SystemExit("SLAM produced fewer than 2 keyframes - check that the "
                         "lidar is spinning and MIN_RANGE/MAX_RANGE are sane.")

    # ---------------- 3. optimise the pose graph ----------------
    chi2_before = chi2_after = float("nan")
    backend_used = "none"
    if optimize_graph:
        report(f"  pose graph: {len(poses)} vertices, {len(edges)} edges "
               f"({len(loops)} loops) -> optimising with g2o")
        poses, chi2_before, chi2_after, backend_used, _opt = graph.optimize(
            poses, edges, backend=backend)
        report(f"  chi2 {chi2_before:.3f} -> {chi2_after:.3f} ({backend_used})")

    # ---------------- 4. render the occupancy map ----------------
    report("  rendering occupancy grid ...")
    grid = occupancy_map.build_map(poses, keys, lidar_offsets=lidar_offsets)

    stats = {
        "scans_read": len(scans),
        "keyframes": len(poses),
        "edges": len(edges),
        "loops": len(loops),
        "matched": n_matched,
        "rejected": n_rejected,
        "chi2_before": chi2_before,
        "chi2_after": chi2_after,
        "backend": backend_used,
        "path_length_m": float(np.sum(np.linalg.norm(
            np.diff(np.array([translation(p) for p in poses]), axis=0), axis=1)))
        if len(poses) > 1 else 0.0,
        "runtime_s": round(time.time() - t_start, 1),
    }
    return SlamResult(poses=poses, scans=keys, edges=edges, grid=grid, stats=stats)


def _accept_match(result, T_guess: np.ndarray, T_new: np.ndarray,
                  max_error: float | None = None,
                  min_inliers: float | None = None) -> tuple[bool, str]:
    """Three cheap sanity checks before a match is trusted."""
    max_error = config.ICP_ACCEPT_ERROR_M if max_error is None else max_error
    min_inliers = (config.ICP_ACCEPT_INLIER_FRACTION
                   if min_inliers is None else min_inliers)
    if not np.isfinite(result.mean_error) or result.inlier_fraction == 0.0:
        return False, "no correspondences"
    if result.mean_error > max_error:
        return False, f"mean error {result.mean_error:.3f} m"
    if result.inlier_fraction < min_inliers:
        return False, f"only {result.inlier_fraction:.0%} inliers"
    moved = float(np.linalg.norm(translation(T_new) - translation(T_guess)))
    turned = abs(angle_diff(heading(T_new), heading(T_guess)))
    if moved > config.ICP_MAX_CORRECTION_M:
        return False, f"jumped {moved:.2f} m"
    if turned > math.radians(config.ICP_MAX_CORRECTION_DEG):
        return False, f"turned {math.degrees(turned):.0f} deg"
    return True, "ok"


def _find_loop_candidate(poses, j: int) -> int | None:
    """Closest earlier keyframe (in current pose estimate) that we may loop to."""
    if j <= config.LOOP_MIN_NODE_GAP:
        return None
    here = translation(poses[j])
    best, best_d = None, 1e9
    for c in range(0, j - config.LOOP_MIN_NODE_GAP):
        d = float(np.linalg.norm(translation(poses[c]) - here))
        dh = abs(angle_diff(heading(poses[c]), heading(poses[j])))
        if d < config.LOOP_MAX_DISTANCE_M and \
                dh < math.radians(config.LOOP_MAX_HEADING_DEG) and d < best_d:
            best, best_d = c, d
    return best


# --------------------------------------------------------------------------
def save_results(result: SlamResult, out_dir: str | Path = "maps",
                 db_path: str | Path | None = None, name: str = "room") -> dict:
    """Write PNG + g2o file and store the optimised graph back into SQLite."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    png = out / f"{name}.png"
    occupancy_map.save_png(result.grid, str(png), poses=result.poses,
                           title=f"2D SLAM map - {len(result.poses)} keyframes")
    occupancy_map.save_npz(result.grid, str(out / f"{name}_grid.npz"))
    graph.write_g2o_file(str(out / f"{name}.g2o"), result.poses, result.edges)

    if db_path is not None:
        from .db import connect as _connect, save_graph
        conn = _connect(db_path)
        rows = [(i, s.id, s.stamp, T[0, 2], T[1, 2], heading(T))
                for i, (T, s) in enumerate(zip(result.poses, result.scans))]
        erows = [(e.i, e.j, e.T_ij[0, 2], e.T_ij[1, 2], heading(e.T_ij), e.kind, None)
                 for e in result.edges]
        save_graph(conn, rows, erows,
                   meta={"map_png": str(png), **{k: v for k, v in result.stats.items()}})
        conn.close()

    return {"png": str(png), "g2o": str(out / f"{name}.g2o"),
            "npz": str(out / f"{name}_grid.npz")}
