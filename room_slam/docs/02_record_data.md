# Recording data

Recording and mapping are deliberately separate steps:

```
   ESP32 ──TCP──▶ slam2d/record.py ──SQL──▶ data/room.db
                                                 │
                                    slam2d/run_slam.py (read only)
                                                 ▼
                                          maps/room.png
```

You can re-run the SLAM a hundred times on one recording, change parameters, and
compare. That is the whole point.

## Start recording

```bash
python -m slam2d.record --db data/room.db
```

```
[db] writing to /home/you/g2o/room_slam/data/room.db
[tcp] listening on 0.0.0.0:8888
[tcp] ESP32 connected from 192.168.1.23
     214 scans    5.42 Hz  beams=352/360  odom=    0
```

Options:

| Option | Meaning |
| --- | --- |
| `--db PATH` | which SQLite file to write (created if missing) |
| `--port N` | TCP port, default 8888 (must match the sketch) |
| `--host ADDR` | bind address, default `0.0.0.0` (all interfaces) |
| `--snapshot-every N` | rebuild `maps/live/live.png` every N sweeps |
| `--source serial --port /dev/ttyUSB0` | read a USB-TTL stream instead of WiFi |

Everything is committed to the database immediately, so a crash or Ctrl-C never
loses more than one sweep.

## What good data looks like

| Metric | Healthy value | If not |
| --- | --- | --- |
| sweeps per second | 4 – 8 (A1 spins ~5.5 Hz) | low: WiFi congestion, or the lidar is browning out |
| beams per sweep | 300 – 360 | much lower: something is blocking the view, or `MIN_RANGE` filters too little |
| nearest object | 0.15 – 0.5 m | larger: `MIN_RANGE_M` may be hiding a wall; smaller: robot's own body in view |
| duplicates / dropouts | occasional | many: ESP32 reconnect churn, check serial monitor |

## Driving the robot

Do this:

* one **closed lap**: finish where you started, pointing the same way;
* **slow**: 0.1–0.2 m/s, roughly walking pace at half speed;
* **smooth**: gentle arcs; avoid jerky starts and stops;
* keep at least ~0.3 m from walls so the lidar always sees something;
* 150–300 sweeps is enough (30–60 s).

Avoid:

* spinning on the spot faster than ~20°/s (without encoders the matcher has no
  prior and loses track);
* long featureless corridors (the matcher cannot tell 2 m forward from 3 m);
* people walking in front of the lidar;
* bright sun through a window directly into the lidar (it is an IR sensor).

## Two passes over the same room

If you want a nicer map, record **two laps** in the same session. More overlap
means more loop closures, and g2o has more constraints to work with. Just keep
driving — the recorder does not care.

## Stop

`Ctrl-C`. You get:

```
[done] 264 scans, 0 odometry samples -> /home/you/g2o/room_slam/data/room.db
       now run:  python -m slam2d.run_slam --db data/room.db --show
```

## Inspecting the recording

Open it with the bundled helper (no sqlite3 install needed), or any SQLite
viewer:

```bash
python -m slam2d.dbquery --db data/room.db --summary
python -m slam2d.dbquery --db data/room.db --tables
python -m slam2d.dbquery --db data/room.db \
    "select count(*) from scans where n_points != 360"
```

`--summary` shows the number of sweeps, the time span, the mean beams per sweep,
odometry samples, optimised poses and loop closures — the fastest way to know
whether a recording is usable before running SLAM on it.

Or from Python:

```python
from slam2d.db import connect, fetch_scans, stats
conn = connect("data/room.db", read_only=True)
print(stats(conn))                     # {'scans': 264, 'odom': 0}
scan = fetch_scans(conn)[0]
print(scan.ranges[:8])                 # metres, 0.0 = no return
```

## Replaying without the robot

`slam2d/replay.py` speaks exactly the wire protocol, so you can test the whole
network path on one machine:

```bash
# terminal 1
python -m slam2d.record --db data/test.db

# terminal 2
python -m slam2d.replay --db data/demo.db --with-odom
```

The resulting `data/test.db` should give the same map as `data/demo.db`.
That is a genuinely useful regression test when you change the parser.

## Recording over USB instead of WiFi

If WiFi refuses to cooperate, connect a USB-TTL adapter (3.3 V) to the lidar's
TX/RX/GND and run:

```bash
pip install pyserial
python -m slam2d.record --source serial --port /dev/ttyUSB0 --db data/room.db
```

On Windows the port is `COM5` (or similar). You then need a small sender on the
ESP32 side, or simply use the lidar's own USB adapter and the RPLIDAR SDK — but
at that point the ESP32 is out of the loop entirely, which is fine for a first
map.
