"""Coarse-to-fine pose search for when there is no motion prior.

Why this file exists
--------------------
ICP (``slam2d/icp.py``) is a *local* method: you must start it within roughly
10-20 cm of the answer or it slides onto the wrong wall.  With wheel odometry
that is never a problem - odometry gives the start point.  But when the lidar is
plugged straight into a USB port there is no odometry at all, and "assume the
robot did not move" is a bad guess: at 5.5 Hz even a slow 0.15 m/s walk moves
8 cm between sweeps, and the error compounds every sweep until tracking dies.

So before running ICP we search for a good starting pose on a coarse grid:

  1. rasterise the recent map points into a *likelihood field* - every cell
     holds a score that decays with the distance to the nearest map point
     (a distance transform, computed once per keyframe);
  2. for every candidate (dx, dy, dtheta) in a window around the last pose,
     score the scan against that field;
  3. keep the best few candidates, refine each with ICP, keep the best result.

This is the same idea as Hector SLAM's multi-resolution correlation search.
Because the field is a lookup table, evaluating thousands of candidate poses is
cheap - no nearest-neighbour search per candidate.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:                                    # scipy is already a hard dependency
    from scipy.ndimage import distance_transform_edt, map_coordinates as _sample
except Exception:                       # pragma: no cover
    distance_transform_edt = None
    _sample = None

from . import config
from .geometry import se2, transform_points

__all__ = ["LikelihoodField", "search_poses"]


@dataclass
class LikelihoodField:
    """Distance-to-nearest-map-point on a regular grid."""

    dist: np.ndarray            # (H, W) metres to the nearest map point
    origin: np.ndarray          # (2,) world coordinate of cell (0, 0)
    cell: float                 # metres per cell
    sigma: float                # score = exp(-d^2 / (2 sigma^2))

    @classmethod
    def from_points(cls, points: np.ndarray, cell: float | None = None,
                    sigma: float | None = None,
                    padding: float = 1.0) -> "LikelihoodField | None":
        cell = cell or config.SEARCH_CELL_M
        sigma = sigma or config.SEARCH_SIGMA_M
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) < 20:
            return None
        lo = pts.min(axis=0) - padding
        hi = pts.max(axis=0) + padding
        w = int(np.ceil((hi[0] - lo[0]) / cell)) + 1
        h = int(np.ceil((hi[1] - lo[1]) / cell)) + 1
        if w * h > 16_000_000:              # keep memory sane on huge maps
            return None

        occupied = np.zeros((h, w), dtype=bool)
        cols = np.floor((pts[:, 0] - lo[0]) / cell).astype(np.int64)
        rows = np.floor((pts[:, 1] - lo[1]) / cell).astype(np.int64)
        ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        occupied[rows[ok], cols[ok]] = True

        if distance_transform_edt is not None:
            dist = distance_transform_edt(~occupied) * cell
        else:                               # pragma: no cover - numpy fallback
            dist = _brute_force_distance(occupied, cell)
        return cls(dist=dist, origin=np.asarray(lo, dtype=float), cell=cell,
                   sigma=sigma)

    # ------------------------------------------------------------------
    def score(self, points: np.ndarray) -> float:
        """Mean match score of world-frame points, 1.0 = perfectly on the map."""
        if _sample is None or len(points) == 0:      # pragma: no cover
            return 0.0
        d = self.distances(points)
        if d is None:
            return 0.0
        return float(np.mean(np.exp(-(d ** 2) / (2.0 * self.sigma ** 2))))

    def distances(self, points: np.ndarray) -> np.ndarray | None:
        """Bilinearly interpolated distance-to-map for world-frame points."""
        if _sample is None or len(points) == 0:      # pragma: no cover
            return None
        col = (points[:, 0] - self.origin[0]) / self.cell
        row = (points[:, 1] - self.origin[1]) / self.cell
        h, w = self.dist.shape
        if (col.min() < -1 or col.max() > w or row.min() < -1
                or row.max() > h):
            return None                              # scan outside the field
        return _sample(self.dist, np.vstack((row, col)), order=1, mode="nearest")


def _brute_force_distance(occupied: np.ndarray, cell: float) -> np.ndarray:
    """Fallback when scipy is missing: exact but slow, on a small grid."""
    from .icp import NearestNeighbour2D

    rows, cols = np.nonzero(occupied)
    pts = np.column_stack((cols, rows)).astype(float) * cell
    nn = NearestNeighbour2D(pts, cell=max(cell, 0.15))
    rr, cc = np.meshgrid(np.arange(occupied.shape[0]),
                         np.arange(occupied.shape[1]), indexing="ij")
    query = np.column_stack((cc.ravel(), rr.ravel())).astype(float) * cell
    d, _ = nn.nearest(query, max_dist=1e3)
    return d.reshape(occupied.shape)


# ----------------------------------------------------------------------
def search_poses(scan_pts: np.ndarray, field: LikelihoodField,
                 T_pred: np.ndarray, window_xy: float, window_deg: float,
                 step_xy: float = None, step_deg: float = None,
                 top_k: int = 3) -> list[tuple[float, np.ndarray]]:
    """Score a grid of candidate poses around ``T_pred``.

    Returns up to ``top_k`` (score, pose) pairs, best first.  The scan points are
    rotated once per candidate heading instead of once per candidate pose, which
    is what makes the search fast enough to run online.
    """
    step_xy = step_xy or config.SEARCH_STEP_XY_M
    step_deg = step_deg or config.SEARCH_STEP_DEG

    n_xy = max(1, int(round(window_xy / step_xy)))
    n_th = max(1, int(round(window_deg / step_deg)))
    offsets = np.arange(-n_xy, n_xy + 1) * step_xy
    angles = np.radians(np.arange(-n_th, n_th + 1) * step_deg)

    base_xy = T_pred[:2, 2]
    base_th = np.arctan2(T_pred[1, 0], T_pred[0, 0])
    results: list[tuple[float, np.ndarray]] = []

    for dth in angles:
        c, s = np.cos(base_th + dth), np.sin(base_th + dth)
        R = np.array([[c, -s], [s, c]])
        rotated = scan_pts @ R.T                    # rotate once
        # all translations on the grid, evaluated in one flat array
        gx, gy = np.meshgrid(offsets, offsets, indexing="ij")
        tx = (base_xy[0] + gx).ravel()
        ty = (base_xy[1] + gy).ravel()
        # (n_candidates, n_points, 2) is heavy; loop over x slabs instead
        best_local = None
        for k in range(len(tx)):
            pts = rotated + np.array([tx[k], ty[k]])
            sc = field.score(pts)
            if best_local is None or sc > best_local[0]:
                best_local = (sc, tx[k], ty[k])
        if best_local is not None:
            sc, x, y = best_local
            results.append((sc, se2(x, y, base_th + dth)))

    results.sort(key=lambda r: r[0], reverse=True)
    return results[:top_k]
