# How it works (the maths, gently)

Read this if you want to modify the pipeline. Every claim here maps to a short
function in `slam2d/`.

---

## 1. A pose is a 3×3 matrix

```python
T = se2(x, y, theta)      # slam2d/geometry.py
```

Composition (`A @ B`) = "do B, then A". Inverse (`inverse(T)`) = "the pose of the
world seen from T". Relative motion between two poses is

```python
T_ij = inverse(T_i) @ T_j
```

which is the only way poses are ever compared in this project.

Why a matrix and not `(x, y, theta)` tuples? Because relative poses compose by
multiplication, and because g2o's `SE2` type uses the same convention.

## 2. One sweep, in the lidar frame

The RPLIDAR firmware reports one distance per angle. `Scan.points_in_lidar_frame()`
turns them into points:

```
p_i = ( r_i cos a_i , r_i sin a_i )
```

`a_i = 2π i / bins` (counter-clockwise, x forward, y left — the ROS convention).
Points in the world are then `T_robot @ T_lidar_offset @ p_i`.

## 3. Scan matching (ICP) — `slam2d/icp.py`

Given the new scan points `src` and a target cloud `dst` (the previous keyframes'
points, in world coordinates) we look for the rigid transform `T` minimising

```
E(T) = Σ_k  ρ( || T·src_k − NN_dst(T·src_k) || )
```

where `NN` is the nearest neighbour. The algorithm alternates:

1. **match** — for every transformed source point, find the closest target point
   (`NearestNeighbour2D`, a scipy cKDTree when available);
2. **reject** — drop pairs further apart than `ICP_MAX_CORRESPONDENCE_M`, and
   drop the worst 20 % of what is left (`ICP_TRIM_FRACTION = 0.80`). This is
   what makes the matcher robust to a moving chair or a person;
3. **solve** — compute the optimal rigid transform for the surviving pairs with
   the closed-form 2-D Procrustes/SVD solution (`_best_rigid_transform`):

   ```
   μ_src, μ_dst      centroids
   H  = Σ (src−μ_src)(dst−μ_dst)ᵀ
   R  = V Uᵀ   from SVD(H) = U Σ Vᵀ      (det(R) = −1 forbidden)
   t  = μ_dst − R μ_src
   ```

4. **test** — stop when the step is smaller than `ICP_CONVERGED_M`.

**Crucial detail:** it is called with `T_init = T_guess`, the pose predicted from
the previous motion. ICP is a local method; starting it 1 m away from the answer
makes it converge onto the wrong wall. (That was a real bug in this project —
see the comment in `slam.py`.)

Afterwards the match is accepted only if three independent checks pass
(`_accept_match`): mean error, inlier fraction, and "the correction is not
absurd" (max 0.6 m / 25°). Belt and braces, because a bad match that gets
accepted corrupts everything downstream.

## 4. Keyframes and submaps — `slam.py`

Matching every sweep against the *whole* map would be slow and would let old,
slightly-wrong geometry drag the scan around. So:

* a sweep becomes a **keyframe** only after 10 cm / 6° of motion
  (`KEYFRAME_MIN_DIST_M`, `KEYFRAME_MIN_ROT_RAD`);
* the target cloud is only the **last 15 keyframes**, cropped to a 6 m radius
  around the prediction (`SUBMAP_NUM_KEYFRAMES`, `SUBMAP_RADIUS_M`).

That keeps the matching local, fast, and accurate.

## 5. The pose graph — `slam2d/graph.py`

Vertices are keyframe poses `T_0 … T_{N-1}`. Edges are relative measurements:

```
odometry edge   (i, i+1) :  T_{i,i+1} from scan matching
loop edge       (c, j)   :  T_{c,j}   when keyframe j revisits keyframe c
```

The residual of an edge is its pose error expressed in the measurement frame:

```
E = inverse(T_ij) · inverse(T_i) · T_j
e = ( E.translation , atan2(E.R) )      ∈ R³
```

and g2o minimises

```
χ² = Σ_edges  eᵀ Ω e ,        Ω = diag(1/σ_xy², 1/σ_xy², 1/σ_θ²)
```

with Levenberg–Marquardt on the `SE2` manifold (`EDGE_SE2` / `VERTEX_SE2` from
`g2o/types/slam2d`). Vertex 0 is fixed (`set_fixed(True)`) so the solution does
not drift in the gauge direction; afterwards the result is re-anchored to the
original first pose so the map stays in the same frame between runs.

`slam2d/graph.py` also ships a numeric Gauss-Newton fallback (`_optimize_gn`) so
the project still works if `g2o-python` is not installed. On the demo dataset the
two agree to a few millimetres.

Loop closure detection is deliberately boring: with the *current* estimate, find
the closest keyframe at least 25 nodes old (`LOOP_MIN_NODE_GAP`) within 0.8 m and
45° of heading, then try ICP against its neighbourhood. If the match is better
than `LOOP_ACCEPT_ERROR_M` with 60 % inliers, it becomes a loop edge.

## 6. The occupancy grid — `slam2d/occupancy_map.py`

Every cell stores log-odds `L = log(p / (1−p))`. For each beam:

* all cells the ray passes through get `L += MAP_LOG_ODDS_MISS` (−0.4);
* the cell where the beam ends gets `L += MAP_LOG_ODDS_HIT` (+0.85).

`L` is clamped to ±2…3.5 so one weird scan cannot paint over a wall forever, and
the probability is recovered as `p = 1/(1+e^{−L})`. Cells never touched stay at
`L = 0` (unknown, drawn light grey). The PNG is a grey ramp of that probability,
5 cm per pixel.

Ray casting is vectorised: all rays are sampled at half-cell spacing in one
numpy array, converted to cell indices, and voted with `np.add.at` — about 10 ms
per sweep.

## 7. End-to-end cost

On the demo dataset (160 sweeps, 97 keyframes, 14.4 m path):

| stage | time |
| --- | --- |
| parse + SQL read | 0.05 s |
| scan matching (97 keyframes, ICP to 15-sweep submap) | ~2.8 s |
| g2o (97 vertices, 100 edges) | ~0.1 s |
| occupancy grid + PNG | ~0.5 s |

So a 300-sweep room is a 3–10 s job on a laptop. You can afford to try ten
parameter sets.

## 8. Where the accuracy comes from (and where it leaks)

Measured on the simulated room, aligned to ground truth:

| | value |
| --- | --- |
| pose error, mean | 3.7 cm |
| pose error, max | 6.2 cm |
| heading error, mean | 0.7° |
| final closure error | 0.1 cm |

Sources of error, in order of importance:

1. **Angular resolution.** 1° at 4 m = 7 cm of tangential error. More bins
   (720) or a slower motor helps, at the cost of data rate.
2. **Prediction error without encoders.** If the guess is off by more than
   `ICP_MAX_CORRESPONDENCE_M`, the match fails or jumps.
3. **Environmental change.** A moved chair creates a loop edge that disagrees
   with reality; the trimming and thresholds keep it from dominating.
4. **Lidar mounting.** Any wobble is instantly visible as doubled walls.

## 9. Reading order for the code

```
slam2d/config.py          every number, nothing else
slam2d/geometry.py        3x3 pose maths
slam2d/db.py              SQLite <-> Scan
slam2d/icp.py             scan matching
slam2d/slam.py            the pipeline (start here after the above)
slam2d/graph.py           g2o + fallback
slam2d/occupancy_map.py   the picture
slam2d/record.py          TCP -> SQLite
slam2d/run_slam.py        command line
```

About 1200 lines of Python in total, no framework, no magic.
