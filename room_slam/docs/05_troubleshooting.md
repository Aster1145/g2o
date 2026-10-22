# Troubleshooting

Work top-down: sensor → network → database → SLAM. Each section has a one-line
test that tells you which layer is broken.

---

## A. The lidar

**Test:** does it spin, and does the ESP32 count beams?

```
# serial monitor, 115200
# scans=12 beams=352 good=2112 bad=0 wifi=3 client=1 heap=230100
```

| Symptom | Cause | Fix |
| --- | --- | --- |
| motor does not spin | MOTOCTL not driven | check GPIO25 in the sketch, or tie MOTOCTL to 5 V |
| motor spins, `good=0` | no data arriving | TX/RX swapped; wrong pins; baud not 115200 |
| `beams` grows over the first seconds only | core browning out | better 5 V supply, 470 µF capacitor, common ground |
| `bad=0` but `beams=40` | something blocking the view | check `MIN_RANGE_M`; look for cables in the scan |
| `bad` climbing fast | electrical noise / UART dropping bytes | shorten wires, twist TX/GND, lower motor PWM slightly |
| lidar keeps restarting | supply cannot deliver the current | 2 A supply, separate rail from the ESP32 |
| scan has a weird "wall" 10 cm away | the robot's own body / mount | `MIN_RANGE_M = 0.25` |
| point cloud rotates with the robot but mirrored | lidar mounted upside down | `LIDAR_ANGLE_SIGN = -1.0` |

## B. The network

**Test:** `python -m slam2d.check_stream --seconds 15` and watch the ESP32.

| Symptom | Cause | Fix |
| --- | --- | --- |
| `wifi: connecting` forever | SSID/password; 5 GHz-only AP | fix credentials, use a 2.4 GHz SSID or hotspot |
| `tcp: connecting ... failed` | record.py not running / wrong IP / firewall | see `03_network.md` |
| connects then drops every ~2 s | AP blocking, or IP moved | DHCP reservation, disable AP isolation |
| connects but nothing arrives | port mismatch (8888 vs something else) | check `--port` and `PC_PORT` |
| works, then dies after 10 min | laptop slept / suspended | disable sleep, or use a powered USB hub |
| duplicate scans in the database | ESP32 rebooted and replayed old timestamps | the recorder now handles clock resets; check `min(stamp)` is sane |

Quick raw check with netcat:

```bash
nc -l 8888 | head -c 300
# SCAN 4321 360
# 0:1204 1:1198 2:0 3:1210 ...
```

## C. The database

**Test:** `python -m slam2d.dbquery --db data/room.db --summary`

| Symptom | Cause | Fix |
| --- | --- | --- |
| `unable to open database file` | directory missing with a custom path | the recorder creates parents; check the path is writable |
| `database is locked` | you opened it read-write in another program (e.g. DB browser) | write-access is single-process; close the other tool, or copy the file |
| 0 scans after a good stream | wrong `--db` path; you are looking at another file | the recorder prints the absolute path at start and exit |
| scans but wrong `n_points` | old firmware with a different `bins` | keep 360; the parser bins by index, so any value works if sent consistently |
| WAL files (`-wal`, `-shm`) growing | a reader keeps a long transaction open | normal during recording; they disappear when all connections close |

## D. The SLAM

**Test:** look at `keyframes`, `matched`, `rejected`, `loops` in the summary.

| Symptom | Cause | Fix |
| --- | --- | --- |
| `Only 2 scan(s) in ...` | the database is empty | record first |
| `rejected` > 30 % | prediction too poor, or sensor noise | see `04_tuning.md`, raise `ICP_MAX_CORRESPONDENCE_M` |
| `keyframes` < 10 for a walk | robot barely moved, or every match rejected | check `path_length_m`; drive more |
| `loops: 0` | lap not closed, or poses too far apart | close the lap; raise `LOOP_MAX_DISTANCE_M` |
| `chi2_after > chi2_before` | inconsistent loop edges | raise `LOOP_ACCEPT_INLIER_FRACTION`, lower `LOOP_ACCEPT_ERROR_M` |
| map is a single dark blob | all poses collapsed to one place | the very first scan must be static; start recording, wait 2 s, then drive |
| map is a long thin smear | the robot only rotated | ask for translation, or lower `ICP_MAX_CORRECTION_DEG` |
| PNG is empty/white | `MAX_RANGE_M` smaller than the room | raise it |

## E. Python / g2o issues

| Symptom | Cause | Fix |
| --- | --- | --- |
| segfault on `import g2o` | g2o-python wheel built for numpy 1.x, you have 2.x | `pip install "numpy<2"` |
| `ModuleNotFoundError: slam2d` | not run from the `room_slam` folder | `cd room_slam` (or `pip install -e .`) |
| `No module named g2o` | optional dependency missing | `pip install g2o-python`; the fallback solver is used otherwise |
| `ModuleNotFoundError: matplotlib` | only needed for PNGs | `pip install matplotlib` |
| very slow (> 2 min for 300 scans) | scipy missing (fallback grid search) | `pip install scipy` |
| `--show` opens nothing | headless / no GUI backend | just open `maps/room.png` |

## F. "It works but the map is ugly"

Look at the picture and ask one question: *are the walls in the right place?*

* **Walls in the right place, thick and dashed** → sensor geometry. Slightly
  rotate the lidar so you do not hit the same 1° bin twice; add
  `MAP_LOG_ODDS_HIT = 1.1`; check the mount is rigid (a wobbling lidar ruins
  everything).
* **Walls in the wrong place, duplicated** → pose error. Loop closure / tuning.
* **Walls in the right place but the robot path wanders** → expected: the path
  is drawn from the scans, not from truth. If you care, add encoders.

## G. Escalation checklist

If you are stuck, collect these four things before changing code:

1. `check_stream` output (Hz, beams, nearest/farthest, ASCII picture).
2. The `run_slam` summary block.
3. The serial monitor of the ESP32 for 30 s.
4. `python -m slam2d.dbquery --db data/room.db --summary`.

Nine times out of ten the answer is in (1).
