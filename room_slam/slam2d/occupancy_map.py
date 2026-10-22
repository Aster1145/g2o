"""Occupancy grid (the actual "2D SLAM map" you can look at).

Each cell stores a log-odds value:

    > 0  -> the wall is probably here     (dark in the PNG)
    < 0  -> the lidar beam passed through (light)
    == 0 -> never observed                (white / unknown)

Rays are traced from the sensor to the hit point; cells the ray crosses get a
"miss" vote and the cell at the end gets a "hit" vote.  This is the same idea
used by every occupancy-grid SLAM stack, just written out explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import config
from .geometry import transform_points


@dataclass
class OccMap:
    resolution: float
    xmin: float
    ymin: float
    width: int
    height: int
    log_odds: np.ndarray

    # ---------------- construction ----------------
    @classmethod
    def around(cls, points_xy: np.ndarray, resolution: float | None = None,
               padding: float | None = None) -> "OccMap":
        resolution = resolution or config.MAP_RESOLUTION_M
        padding = padding if padding is not None else config.MAP_PADDING_M
        if len(points_xy) == 0:
            xmin, ymin, xmax, ymax = -5.0, -5.0, 5.0, 5.0
        else:
            xmin, ymin = points_xy.min(axis=0) - padding
            xmax, ymax = points_xy.max(axis=0) + padding
        w = int(np.ceil((xmax - xmin) / resolution)) + 1
        h = int(np.ceil((ymax - ymin) / resolution)) + 1
        return cls(resolution, float(xmin), float(ymin), w, h,
                   np.zeros((h, w), dtype=np.float32))

    # ---------------- indexing ----------------
    def world_to_cell(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        col = np.floor((xy[:, 0] - self.xmin) / self.resolution).astype(np.int64)
        row = np.floor((xy[:, 1] - self.ymin) / self.resolution).astype(np.int64)
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.xmin + (col + 0.5) * self.resolution
        y = self.ymin + (row + 0.5) * self.resolution
        return x, y

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(xmin, xmax, ymin, ymax) for plotting."""
        return (self.xmin, self.xmin + self.width * self.resolution,
                self.ymin, self.ymin + self.height * self.resolution)

    # ---------------- the interesting part ----------------
    def integrate(self, lidar_origin_world: np.ndarray, points_world: np.ndarray,
                  max_range: float | None = None,
                  hit: float | None = None, miss: float | None = None) -> None:
        """Add one scan's evidence to the grid.

        lidar_origin_world : (2,) where the lidar was when this scan was taken
        points_world       : (N, 2) world coordinates of the ray end points
        """
        max_range = max_range or config.MAX_RANGE_M
        hit = config.MAP_LOG_ODDS_HIT if hit is None else hit
        miss = config.MAP_LOG_ODDS_MISS if miss is None else miss
        if len(points_world) == 0:
            return

        origin = np.asarray(lidar_origin_world, dtype=float)
        deltas = points_world - origin
        dists = np.hypot(deltas[:, 0], deltas[:, 1])
        ok = (dists > 1e-3) & (dists < max_range)
        deltas, dists = deltas[ok], dists[ok]
        if len(dists) == 0:
            return

        # sample every ray at half-cell spacing, then de-duplicate cells
        step = 0.5 * self.resolution
        n_max = int(np.ceil(dists.max() / step)) + 1
        n_max = min(n_max, 2000)                      # safety valve
        frac = np.linspace(0.0, 1.0, n_max)
        # (N, n_max, 2): point along each ray
        samples = origin[None, None, :] + deltas[:, None, :] * frac[None, :, None]

        cells = np.floor((samples - np.array([self.xmin, self.ymin])) /
                         self.resolution).astype(np.int64)
        inside = ((cells[..., 0] >= 0) & (cells[..., 0] < self.width) &
                  (cells[..., 1] >= 0) & (cells[..., 1] < self.height))
        rows = cells[..., 1][inside]
        cols = cells[..., 0][inside]

        # misses: every sampled cell except the last one of each ray
        is_last = np.zeros_like(inside)
        is_last[np.arange(len(dists)), n_max - 1] = True
        miss_mask = inside & ~is_last
        m_rows = cells[..., 1][miss_mask]
        m_cols = cells[..., 0][miss_mask]
        np.add.at(self.log_odds, (m_rows, m_cols), miss)

        # hits: the end point of every ray
        end_cells = np.floor((points_world[ok] - np.array([self.xmin, self.ymin])) /
                            self.resolution).astype(np.int64)
        inb = ((end_cells[:, 0] >= 0) & (end_cells[:, 0] < self.width) &
               (end_cells[:, 1] >= 0) & (end_cells[:, 1] < self.height))
        np.add.at(self.log_odds, (end_cells[inb, 1], end_cells[inb, 0]), hit)

        np.clip(self.log_odds, config.MAP_LOG_ODDS_MIN, config.MAP_LOG_ODDS_MAX,
                out=self.log_odds)

    # ---------------- output ----------------
    def probability(self) -> np.ndarray:
        """Occupancy probability in [0, 1]."""
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def to_image(self) -> np.ndarray:
        """RGB uint8 image, row 0 = smallest y (matplotlib ``origin='lower'``)."""
        p = self.probability()
        known = self.log_odds != 0.0
        shade = np.clip((p - 0.30) / 0.40, 0.0, 1.0)        # 0 = free, 1 = wall
        grey = 255.0 * (1.0 - shade)
        img = np.full(p.shape, 240.0)                        # unknown = light grey
        img[known] = grey[known]
        rgb = np.repeat(img[:, :, None].astype(np.uint8), 3, axis=2)
        return rgb

    def world_cloud(self, threshold: float = 0.6) -> np.ndarray:
        rows, cols = np.nonzero(self.probability() > threshold)
        return np.column_stack((self.xmin + (cols + 0.5) * self.resolution,
                                self.ymin + (rows + 0.5) * self.resolution))


def build_map(poses, scans, resolution: float | None = None,
              lidar_offsets=None) -> OccMap:
    """Render all scans at their optimised poses into one occupancy grid."""
    clouds = []
    for pose, scan in zip(poses, scans):
        pts = scan.points_in_lidar_frame()
        T_sensor = pose if lidar_offsets is None else pose @ lidar_offsets
        if len(pts):
            clouds.append(transform_points(T_sensor, pts))
    all_pts = np.vstack(clouds) if clouds else np.zeros((0, 2))
    grid = OccMap.around(all_pts, resolution=resolution)
    for pose, scan in zip(poses, scans):
        pts_local = scan.points_in_lidar_frame()
        T_sensor = pose if lidar_offsets is None else pose @ lidar_offsets
        if len(pts_local) == 0:
            continue
        grid.integrate(T_sensor[:2, 2], transform_points(T_sensor, pts_local))
    return grid


def save_png(grid: OccMap, path: str, poses=None, title: str = "2D SLAM map"):
    """Write the map as a PNG.  matplotlib is only needed for this step."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xmin, xmax, ymin, ymax = grid.extent
    fig, ax = plt.subplots(figsize=(9, 9), dpi=130)
    ax.imshow(grid.to_image(), origin="lower", extent=(xmin, xmax, ymin, ymax),
              interpolation="nearest")
    if poses is not None and len(poses):
        px = [p[0, 2] for p in poses]
        py = [p[1, 2] for p in poses]
        ax.plot(px, py, "-", color="#d62728", linewidth=1.2, label="robot path")
        ax.plot(px[0], py[0], "o", color="#2ca02c", markersize=7, label="start")
        ax.plot(px[-1], py[-1], "s", color="#d62728", markersize=6, label="end")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_npz(grid: OccMap, path: str) -> None:
    np.savez_compressed(path, log_odds=grid.log_odds,
                        resolution=grid.resolution,
                        origin=np.array([grid.xmin, grid.ymin]))


def show_map(grid: OccMap, poses=None, title: str = "2D SLAM map") -> None:
    """Interactive matplotlib window (needs a GUI backend)."""
    import matplotlib.pyplot as plt

    xmin, xmax, ymin, ymax = grid.extent
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(grid.to_image(), origin="lower", extent=(xmin, xmax, ymin, ymax),
              interpolation="nearest")
    if poses is not None and len(poses):
        px = [p[0, 2] for p in poses]
        py = [p[1, 2] for p in poses]
        ax.plot(px, py, "-", color="#d62728", linewidth=1.2, label="robot path")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.set_aspect("equal")
    plt.show()
