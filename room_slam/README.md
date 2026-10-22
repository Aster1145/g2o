# room_slam — 2D SLAM with an RPLIDAR, an ESP32 and g2o

A complete, readable 2D SLAM stack for a small differential-drive robot: an
RPLIDAR sweeps the room, an ESP32 forwards the scans over WiFi, a laptop stores
them in SQLite and turns them into an occupancy-grid map using scan matching and
g2o pose-graph optimisation.

```
RPLIDAR A1 ──UART──▶ ESP32 ──WiFi/TCP──▶ record.py ──▶ room.db (SQLite)
                                                          │
                                          run_slam.py ◀───┘
                                                │
                                    maps/room.png  (your map)
                                    maps/room.g2o  (open in g2o_viewer)
```

**New here? → [`docs/STEP_BY_STEP_GUIDE.md`](docs/STEP_BY_STEP_GUIDE.md)**
It walks from unboxing to a finished map, with a "you should now see" checkpoint
at every step.

## Try it right now, without hardware

```bash
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python -m slam2d.simulate_room --out data/demo.db    # fake a 6x5 m room
python -m slam2d.run_slam --db data/demo.db --name demo
```

`maps/demo.png` appears in a few seconds:

```
   keyframes: 97     edges: 100     loops: 4
   chi2: 0.553 -> 0.019 (g2o)      runtime: 3.4 s
```

That is the same output your robot will produce.

## With the robot

```bash
# 1. flash firmware/esp32_rplidar_bridge (set WiFi + your laptop IP first)
# 2. on the laptop, start the recorder
python -m slam2d.record --db data/room.db
# 3. drive one slow closed lap, then Ctrl-C
# 4. build the map
python -m slam2d.run_slam --db data/room.db --name room --show
```

Check the sensor before recording:

```bash
python -m slam2d.check_stream --seconds 15
# 118 sweeps in 15.0 s -> 7.9 Hz, beams 352/360, nearest 0.34 m
# plus an ASCII picture of the room
```

## What is in here

| Path | What it is |
| --- | --- |
| `slam2d/` | the Python SLAM stack (~1200 lines, numpy + g2o) |
| `slam2d/icp.py` | 2D scan matching (trimmed, gated, SVD closed form) |
| `slam2d/graph.py` | pose graph: g2o `EDGE_SE2` + a numpy fallback solver |
| `slam2d/occupancy_map.py` | log-odds grid, PNG/GUI drawing |
| `slam2d/slam.py` | the pipeline: keyframes → submaps → loops → optimise → render |
| `slam2d/record.py` | TCP/serial receiver that writes SQLite |
| `slam2d/replay.py` | fakes the ESP32 so you can test the network offline |
| `slam2d/check_stream.py` | live stream stats + ASCII view |
| `slam2d/simulate_room.py` | simulated room with drifting odometry |
| `firmware/esp32_rplidar_bridge/` | Arduino sketch + `rplidar_parser.h` |
| `sql/schema.sql` | the database layout |
| `unit_test/` | python tests + a host-compiled test of the C++ parser |
| `docs/` | the guide and the deep dives |

## Database

Four tables, all ordinary SQL (`sql/schema.sql`):

| table | written by | contents |
| --- | --- | --- |
| `scans` | `record.py` | one row per sweep: stamp, float32 ranges + angles (BLOBs) |
| `odom` | `record.py` | optional wheel odometry, `stamp, x, y, theta` |
| `poses` | `run_slam.py` | optimised keyframe poses |
| `graph_edges` | `run_slam.py` | odometry + loop-closure edges |
| `map_meta` | both | run statistics, map path |

No `sqlite3` command line tool? Use the bundled query helper (or the same SQL
in any SQLite viewer):

```bash
python -m slam2d.dbquery --db data/room.db --summary
python -m slam2d.dbquery --db data/room.db --tables
python -m slam2d.dbquery --db data/room.db "select kind, count(*) from graph_edges group by kind"
```

With the `sqlite3` CLI the same thing is:

```bash
sqlite3 data/room.db "select count(*) from scans;"
sqlite3 data/room.db "select node_id, round(x,2), round(y,2) from poses limit 5;"
```

## Commands

```bash
make install     # venv + dependencies
make demo        # simulate + map, no hardware
make record      # listen for the ESP32, write data/room.db
make live        # record and refresh maps/live/live.png every 60 sweeps
make map         # SLAM on data/room.db
make check       # live stream stats + ASCII room view
make test        # python unit tests
make test-cpp    # compile + run the lidar parser test on this PC
```

Useful flags for `run_slam.py`: `--no-odom`, `--no-loops`, `--no-optimize`,
`--backend {auto,g2o,gn}`, `--max-scans N`, `--name`, `--show`.

## Tuning

Everything is in `slam2d/config.py`, grouped and commented. The three that matter
are `KEYFRAME_MIN_DIST_M`, `ICP_MAX_CORRESPONDENCE_M` and
`ICP_ACCEPT_ERROR_M`. Symptom → setting tables are in
[`docs/04_tuning.md`](docs/04_tuning.md), failures in
[`docs/05_troubleshooting.md`](docs/05_troubleshooting.md).

## Requirements

Python 3.9+, `numpy`, `scipy` (fast nearest neighbour), `matplotlib` (drawing),
`g2o-python` (optional — a fallback solver is used if absent), `pyserial` (only
for USB recording).

> **numpy 2 caveat:** `g2o-python` wheels are built against numpy 1.x. If
> `import g2o` segfaults, `pip install "numpy<2"`.

## Notes

* This folder lives inside the [g2o](https://github.com/RainerKuemmerle/g2o)
  repository because the pose-graph back end *is* g2o; build g2o normally and
  `maps/*.g2o` files open in `g2o_viewer`.
* The ESP32 never writes to SQL: it has neither the memory nor a practical
  filesystem for it. It streams; the laptop records. That split also means one
  recording can be re-mapped endlessly.
* No wheel encoders are required. Odometry is only used as the scan matcher's
  initial guess — see [`docs/06_no_encoders.md`](docs/06_no_encoders.md).
