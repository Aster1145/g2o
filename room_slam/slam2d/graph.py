"""Pose-graph optimisation with g2o (with a numpy fallback).

The SLAM loop produces this graph:

    vertices : robot poses  T_0 ... T_N-1   (one per keyframe scan)
    edges    : relative measurements  T_ij  = inv(T_i) * T_j

    * consecutive poses are linked by "odometry" edges - cheap, but they drift
    * whenever the robot revisits a place, scan matching creates a "loop" edge
      that pulls the drifted chain back together

g2o solves the non-linear least squares problem

    argmin  sum_ij  e_ij(T_i, T_j)^T Omega_ij e_ij(T_i, T_j)

where e_ij = log( inv(T_ij) * inv(T_i) * T_j ).  That is exactly the
``EDGE_SE2`` residual implemented in g2o/types/slam2d.

If the g2o Python bindings are not installed we fall back to a plain
Gauss-Newton solver with numeric Jacobians so the pipeline still finishes.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass

import numpy as np

from . import config
from .geometry import inverse, normalize_angle, se2

try:                                            # pragma: no cover
    import g2o as _g2o
except Exception:                               # pragma: no cover
    _g2o = None


@dataclass
class Edge:
    i: int
    j: int
    T_ij: np.ndarray            # relative pose Ti -> Tj
    sigma_xy: float = config.ODOM_SIGMA_XY
    sigma_theta: float = config.ODOM_SIGMA_THETA
    kind: str = "odom"
    information: np.ndarray | None = None

    def omega(self) -> np.ndarray:
        if self.information is not None:
            return self.information
        return np.diag([1.0 / self.sigma_xy ** 2,
                        1.0 / self.sigma_xy ** 2,
                        1.0 / self.sigma_theta ** 2])


# --------------------------------------------------------------------------
def _compose_chain(edges: list[Edge], n: int) -> list[np.ndarray]:
    """Initial guess: integrate the odometry chain from the fixed first pose."""
    poses = [np.eye(3)] * n
    for e in edges:
        if e.kind != "odom":
            continue
        poses[e.j] = poses[e.i] @ e.T_ij
    return poses


def _chi2(poses, edges) -> float:
    total = 0.0
    for e in edges:
        err = _error(poses[e.i], poses[e.j], e.T_ij)
        total += float(err @ e.omega() @ err)
    return total


def _error(Ti: np.ndarray, Tj: np.ndarray, Tij: np.ndarray) -> np.ndarray:
    """Residual (dx, dy, dtheta) in the frame of the measurement."""
    E = inverse(Tij) @ inverse(Ti) @ Tj
    return np.array([E[0, 2], E[1, 2], math.atan2(E[1, 0], E[0, 0])])


# --------------------------------------------------------------------------
# g2o backend
# --------------------------------------------------------------------------
def _optimize_g2o(poses: list[np.ndarray], edges: list[Edge], max_iter: int):
    optimizer = _g2o.SparseOptimizer()
    solver = _g2o.BlockSolverSE2(_g2o.LinearSolverEigenSE2())
    algorithm = _g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(algorithm)

    vertices = []
    for i, T in enumerate(poses):
        v = _g2o.VertexSE2()
        v.set_id(i)
        x, y, theta = T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])
        v.set_estimate(_g2o.SE2(float(x), float(y), float(theta)))
        optimizer.add_vertex(v)
        vertices.append(v)
    vertices[0].set_fixed(True)

    for e in edges:
        edge = _g2o.EdgeSE2()
        edge.set_vertex(0, vertices[e.i])
        edge.set_vertex(1, vertices[e.j])
        x, y, theta = e.T_ij[0, 2], e.T_ij[1, 2], math.atan2(e.T_ij[1, 0], e.T_ij[0, 0])
        edge.set_measurement(_g2o.SE2(float(x), float(y), float(theta)))
        edge.set_information(e.omega())
        optimizer.add_edge(edge)

    optimizer.initialize_optimization()
    optimizer.set_verbose(False)
    optimizer.optimize(max_iter)

    out = []
    for v in vertices:
        est = v.estimate()
        t = est.translation()
        out.append(se2(float(t[0]), float(t[1]), float(est.rotation().angle())))
    return out, optimizer


# --------------------------------------------------------------------------
# fallback backend (numeric Gauss-Newton, works with scipy or plain numpy)
# --------------------------------------------------------------------------
def _optimize_gn(poses, edges, max_iter: int, tol: float = 1e-9):
    n = len(poses)
    if n < 2:
        return [p.copy() for p in poses]

    def residuals(vec):
        Ts = [se2(vec[3 * k], vec[3 * k + 1], vec[3 * k + 2]) for k in range(n)]
        r = []
        for e in edges:
            r.extend(_error(Ts[e.i], Ts[e.j], e.T_ij))
        return np.asarray(r), Ts

    def angle_wrap_residual(r, Ts):
        # keep the theta component of each residual in [-pi, pi]
        r = r.copy()
        r[2::3] = np.arctan2(np.sin(r[2::3]), np.cos(r[2::3]))
        return r

    x = np.concatenate([[p[0, 2], p[1, 2], math.atan2(p[1, 0], p[0, 0])] for p in poses])
    m = 3 * len(edges)
    free = np.arange(3, 3 * n)          # vertex 0 is fixed
    lam = 1e-6

    for _ in range(max_iter):
        r, _ = residuals(x)
        r = angle_wrap_residual(r, None)
        J = np.zeros((m, 3 * n))
        eps = 1e-6
        for k in free:
            xp = x.copy()
            xp[k] += eps
            rp, _ = residuals(xp)
            rp = angle_wrap_residual(rp, None)
            J[:, k] = (rp - r) / eps
        Jf = J[:, free]
        H = Jf.T @ Jf
        g = Jf.T @ r
        H += lam * np.eye(H.shape[0])
        try:
            delta = -np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            warnings.warn("pose graph solve failed; keeping odometry poses")
            break
        x[free] += delta
        if np.linalg.norm(delta) < tol:
            break

    return [se2(x[3 * k], x[3 * k + 1], x[3 * k + 2]) for k in range(n)], None


# --------------------------------------------------------------------------
def optimize(poses: list[np.ndarray], edges: list[Edge],
             max_iter: int | None = None, backend: str = "auto"):
    """Optimise the graph.  Returns (poses, chi2_before, chi2_after, backend)."""
    max_iter = max_iter or config.G2O_MAX_ITERATIONS
    before = _chi2(poses, edges)
    use = None

    if backend in ("auto", "g2o") and _g2o is not None:
        use = "g2o"
    elif backend in ("auto", "gn"):
        use = "gn"
    if backend == "g2o" and _g2o is None:
        raise RuntimeError("g2o python bindings not installed "
                           "(pip install g2o-python)")

    optimizer = None
    if use == "g2o":
        try:
            out, optimizer = _optimize_g2o(poses, edges, max_iter)
        except Exception as exc:                     # pragma: no cover
            warnings.warn(f"g2o failed ({exc}); using numpy fallback")
            out, optimizer = _optimize_gn(poses, edges, max_iter)
            use = "gn"
    else:
        out, optimizer = _optimize_gn(poses, edges, max_iter)

    # re-anchor at the original first pose so the result lands in the same
    # world frame as the input (the map then does not jump between runs)
    shift = poses[0] @ inverse(out[0])
    out = [shift @ T for T in out]
    return out, before, _chi2(out, edges), use, optimizer


def write_g2o_file(path: str, poses, edges, fixed_first: bool = True) -> None:
    """Write a standard .g2o file you can open with g2o_viewer.

        g2o_viewer map.g2o
        g2o -i 50 -o out.g2o map.g2o
    """
    lines = []
    for i, T in enumerate(poses):
        lines.append(f"VERTEX_SE2 {i} {T[0, 2]:.6f} {T[1, 2]:.6f} "
                     f"{math.atan2(T[1, 0], T[0, 0]):.6f}")
    if fixed_first:
        lines.append("FIX 0")
    for e in edges:
        info = e.omega()
        x, y = e.T_ij[0, 2], e.T_ij[1, 2]
        t = math.atan2(e.T_ij[1, 0], e.T_ij[0, 0])
        i11, i12, i13 = info[0, 0], info[0, 1], info[0, 2]
        i22, i23, i33 = info[1, 1], info[1, 2], info[2, 2]
        lines.append(f"EDGE_SE2 {e.i} {e.j} {x:.6f} {y:.6f} {t:.6f} "
                     f"{i11:.6f} {i12:.6f} {i13:.6f} {i22:.6f} {i23:.6f} {i33:.6f}")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


__all__ = ["Edge", "optimize", "write_g2o_file"]
