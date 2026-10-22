"""2D scan matching with point-to-point ICP.

Why this file exists
--------------------
Your robot has no GPS and no compass, and cheap wheel odometry drifts within
seconds.  So we do not trust odometry to place a scan - we *optimise* the
alignment between the new scan and the map we already built.  That alignment
gives us a pose correction, and the corrected pose is what makes the map look
crisp instead of smeared.

This is a textbook implementation (Besl & McKay 1992) with two practical
additions that matter a lot for lidar:

  * trimmed correspondences  - throw away the worst 20 % of point pairs so a
    moving chair or a person does not drag the whole scan;
  * a correspondence gate    - never pair two points more than
    ``ICP_MAX_CORRESPONDENCE_M`` apart.

Standard library + numpy only, so it runs anywhere.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:                                            # optional but recommended
    from scipy.spatial import cKDTree as _cKDTree
except Exception:                               # pragma: no cover
    _cKDTree = None

from . import config
from .geometry import inverse, se2, transform_points, xytheta


@dataclass
class IcpResult:
    transform: np.ndarray        # T_prev_new: moves points from "new" into "prev"
    mean_error: float            # average distance of the accepted pairs, metres
    inlier_fraction: float       # accepted pairs / candidate points
    iterations: int
    converged: bool

    def __bool__(self) -> bool:               # allows `if result:`
        return bool(self.converged)


def _best_rigid_transform(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Closed-form 2D Procrustes: the transform that best maps src onto dst."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    s, d = src - cs, dst - cd
    # 2x2 cross covariance
    H = s.T @ d
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                 # reflection guard
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = cd - R @ cs
    return T


class NearestNeighbour2D:
    """Nearest neighbour search over a fixed point cloud.

    Uses scipy's cKDTree when available (10-50x faster) and falls back to a
    uniform grid otherwise, so the project still runs on a bare numpy install.
    """

    def __init__(self, cloud: np.ndarray, cell: float | None = None):
        self.cloud = np.asarray(cloud, dtype=float).reshape(-1, 2)
        self.cell = cell or config.ICP_MAX_CORRESPONDENCE_M
        self.tree = None
        if _cKDTree is not None and len(self.cloud) > 0:
            self.tree = _cKDTree(self.cloud)
        self.grid: dict[tuple[int, int], list[int]] = {}
        if len(self.cloud):
            keys = np.floor(self.cloud / self.cell).astype(np.int64)
            for idx, (kx, ky) in enumerate(keys):
                self.grid.setdefault((int(kx), int(ky)), []).append(idx)

    def nearest(self, pts: np.ndarray, max_dist: float):
        """Return (distances, indices into self.cloud), -1 where nothing found."""
        n = len(pts)
        if self.tree is not None and n:
            d, i = self.tree.query(pts, k=1,
                                   distance_upper_bound=max_dist)
            found = np.isfinite(d)
            i = np.where(found, i, -1).astype(np.int64)
            return d, i

        best_d = np.full(n, np.inf)
        best_i = np.full(n, -1, dtype=np.int64)
        if not self.grid or n == 0:
            return best_d, best_i
        # search radius in cells; +1 because the query cell itself is included
        ring = int(np.ceil(max_dist / self.cell)) + 1
        keys = np.floor(pts / self.cell).astype(np.int64)
        for k, (qx, qy) in enumerate(keys):
            x, y = int(qx), int(qy)
            for dx in range(-ring, ring + 1):
                for dy in range(-ring, ring + 1):
                    bucket = self.grid.get((x + dx, y + dy))
                    if not bucket:
                        continue
                    idx = np.asarray(bucket)
                    d = np.hypot(self.cloud[idx, 0] - pts[k, 0],
                                 self.cloud[idx, 1] - pts[k, 1])
                    j = int(np.argmin(d))
                    if d[j] < best_d[k]:
                        best_d[k] = d[j]
                        best_i[k] = idx[j]
        return best_d, best_i


def icp(src: np.ndarray, dst_cloud: np.ndarray, T_init: np.ndarray | None = None,
        max_iter: int | None = None, gate: float | None = None,
        trim: float | None = None) -> IcpResult:
    """Align ``src`` points onto the target cloud ``dst_cloud``.

    ``T_init`` is your best guess (e.g. from odometry).  The returned transform
    is the *correction* that is applied on top of it, i.e. the composition
    ``T_correction @ T_init`` maps src points into dst coordinates.
    """
    max_iter = max_iter or config.ICP_MAX_ITER
    gate = gate or config.ICP_MAX_CORRESPONDENCE_M
    trim = trim if trim is not None else config.ICP_TRIM_FRACTION

    src = np.asarray(src, dtype=float).reshape(-1, 2)
    target = NearestNeighbour2D(dst_cloud)
    T = np.eye(3)
    if T_init is not None:
        T = np.asarray(T_init, dtype=float).copy()

    moved = transform_points(T, src)
    prev_err = np.inf
    mean_err = np.inf
    inlier_frac = 0.0
    it = 0
    for it in range(1, max_iter + 1):
        d, idx = target.nearest(moved, gate)
        ok = (idx >= 0) & (d < gate)
        inlier_frac = float(ok.sum()) / max(len(src), 1)
        if ok.sum() < 6:
            return IcpResult(T, float(mean_err if np.isfinite(mean_err) else 9.9),
                             inlier_frac, it, False)

        # trimmed: keep the closest `trim` fraction of the accepted pairs
        d_ok = d[ok]
        keep_n = max(6, int(trim * len(d_ok)))
        order = np.argsort(d_ok)[:keep_n]
        sel = np.flatnonzero(ok)[order]

        step = _best_rigid_transform(moved[sel], target.cloud[idx[sel]])
        moved = transform_points(step, moved)
        T = step @ T

        mean_err = float(np.mean(d[sel]))
        step_norm = np.hypot(step[0, 2], step[1, 2]) + abs(
            np.arctan2(step[1, 0], step[0, 0]))
        if step_norm < config.ICP_CONVERGED_M and abs(prev_err - mean_err) < 1e-6:
            return IcpResult(T, mean_err, inlier_frac, it, True)
        prev_err = mean_err

    # ran out of iterations: accept if the residual is small enough overall
    converged = mean_err < config.ICP_ACCEPT_ERROR_M and \
        inlier_frac > config.ICP_ACCEPT_INLIER_FRACTION * 0.8
    return IcpResult(T, mean_err, inlier_frac, it, converged)


def align_scan_to_map(scan_pts: np.ndarray, map_pts: np.ndarray,
                      T_guess: np.ndarray, **kw) -> tuple[np.ndarray, IcpResult]:
    """Convenience wrapper: returns the *absolute* pose and the ICP result.

    ``T_guess`` is the predicted scan pose in the map frame.  The returned pose
    is ``T_icp @ T_guess``.
    """
    res = icp(scan_pts, map_pts, T_init=np.eye(3), **kw)
    return res.transform @ T_guess, res


def pose_from_xytheta(x: float, y: float, t: float) -> np.ndarray:
    return se2(x, y, t)


__all__ = ["icp", "IcpResult", "NearestNeighbour2D", "align_scan_to_map",
           "pose_from_xytheta", "inverse", "xytheta"]
