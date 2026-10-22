# Tuning the map

All numbers live in `slam2d/config.py`. Change one at a time, re-run
`run_slam.py` on the same recording, and compare the PNGs. Nothing here needs a
new walk.

## The three dials that matter most

### 1. How much the robot is allowed to move between scans

```python
KEYFRAME_MIN_DIST_M = 0.10      # new keyframe after 10 cm
KEYFRAME_MIN_ROT_RAD = 0.10     # or 6 degrees
```

Smaller = more keyframes = better accuracy, slower, more memory. If your robot
moves ~1 cm per sweep, `0.10` keeps roughly every 10th sweep.

### 2. How far ICP looks for a matching point

```python
ICP_MAX_CORRESPONDENCE_M = 0.5
```

This must be **larger** than the real error of your prediction, or ICP gives up:

| Your robot | Value |
| --- | --- |
| encoders + slow driving | 0.3 |
| no encoders, slow | 0.5 (default) |
| no encoders, fast jerky driving | 0.8, but expect more false matches |

### 3. How strict the matcher is

```python
ICP_ACCEPT_ERROR_M = 0.08          # mean point-to-point distance
ICP_ACCEPT_INLIER_FRACTION = 0.50  # share of points that found a partner
ICP_MAX_CORRECTION_M = 0.60        # the match may not teleport the robot
ICP_MAX_CORRECTION_DEG = 25.0
```

Too strict → most scans rejected → the map freezes mid-lap. Too loose → a bad
match gets accepted → a wall is duplicated at an angle. Watch the `rejected`
count in the summary: 0–5 % is healthy, > 20 % means something is wrong with the
sensor or the prediction.

## Symptom → setting

| What the map looks like | Cause | What to change |
| --- | --- | --- |
| walls are two parallel lines / ghost walls | drift not corrected | close the lap; `LOOP_MAX_DISTANCE_M = 1.2`; `LOOP_MIN_NODE_GAP = 15` |
| everything is a thick blur | motion per scan too large | drive slower; `ICP_MAX_CORRESPONDENCE_M = 0.3`; `KEYFRAME_MIN_DIST_M = 0.06` |
| map bends then straightens at the end | normal without a loop closure | it *is* a loop closure; look for `loops: 0` in the summary |
| one wall curves away | heading drift | add furniture/features; drive closer to walls; `SUBMAP_NUM_KEYFRAMES = 25` |
| robot path cuts through a wall | false match accepted | `ICP_ACCEPT_ERROR_M = 0.05`; `ICP_MAX_CORRECTION_M = 0.35` |
| map is mirrored | lidar upside down | `LIDAR_ANGLE_SIGN = -1.0` (then record again — angles are stored per scan) |
| map is rotated by a constant angle | index-0 mounting offset | `LIDAR_ANGLE_OFFSET_DEG = 90` (record again) |
| map has a hole around where the robot drove | rays too short | `MAX_RANGE_M = 6` is fine for a room; higher = more data, slower |
| `loops: 0` but you did close the lap | no encoder => poses far apart | `LOOP_MAX_DISTANCE_M = 1.5`; `LOOP_ACCEPT_ERROR_M = 0.12` |
| map too small / few keyframes | keyframes too far apart | `KEYFRAME_MIN_DIST_M = 0.05` |
| very slow (> 60 s) | too many keyframes | `KEYFRAME_MIN_DIST_M = 0.2` |

## A worked example

Suppose the room walls come out doubled on the right side and the summary shows
`loops: 0, rejected: 41` of 300 scans.

1. `rejected: 41` (14 %) says the matcher is struggling. The likely cause
   without encoders is a prediction that is too far off after fast turns.
   Set `ICP_MAX_CORRESPONDENCE_M = 0.8` and re-run. If `rejected` drops to ~5,
   that was it.
2. `loops: 0` says the end of the lap never re-visited something. With 14 % of
   scans rejected, the trajectory is already wrong by the time it closes.
   After fixing (1), loops usually appear on their own.
3. If loops appear but the map still looks doubled, the loop threshold is too
   tight: `LOOP_ACCEPT_ERROR_M = 0.12`.

## Occupancy grid appearance

```python
MAP_RESOLUTION_M = 0.05     # 5 cm; 0.02 for a small room, 0.10 for a big one
MAP_PADDING_M = 0.60
MAP_LOG_ODDS_HIT = 0.85
MAP_LOG_ODDS_MISS = -0.40
```

* Walls too faint → raise `MAP_LOG_ODDS_HIT` to 1.2.
* Walls too thick / smeared → increase the miss weight magnitude
  (`-0.7`) so traversed space is cleaned more aggressively.
* Everything black → your poses are bad, not the grid. Fix matching first.

## Pose graph weights

```python
ODOM_SIGMA_XY = 0.05      # how much you trust consecutive-pose edges (metres)
ODOM_SIGMA_THETA = 0.03
LOOP_SIGMA_XY = 0.08      # how much you trust a loop closure
```

If you have real wheel encoders that you trust, lower `ODOM_SIGMA_*` (e.g. 0.02).
If the matches are noisy, raise them. A loop closure should generally be
slightly *less* trusted than a good scan-to-submap match, because it was made
against an older, less accurate submap.

## Using g2o's own viewer

```bash
python -m slam2d.run_slam --db data/room.db     # writes maps/room.g2o
./build/bin/g2o_viewer maps/room.g2o            # from the g2o build you made
```

Drag the trajectory, press *Optimize*, and watch the loop errors collapse. This
is the fastest way to understand what the pose graph is doing.

Also useful:

```bash
python -m slam2d.run_slam --db data/room.db --no-loops     # see the drift
python -m slam2d.run_slam --db data/room.db --no-optimize  # no g2o at all
python -m slam2d.run_slam --db data/room.db --backend gn   # fallback solver
```
