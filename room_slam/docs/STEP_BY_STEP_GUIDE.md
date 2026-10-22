# Step-by-step: build a 2D SLAM map of your room

This guide is written for someone who has **never done SLAM before**. Do the
steps in order. Every step ends with "**you should now see**" so you know
whether to continue or stop and fix something.

Total time: about one evening, of which 20 minutes is waiting for installs.

> **No ESP32?** If you plug the lidar's own USB adapter into the laptop you can
> skip sections 4-7 (wiring, flashing, network). Use
> [`VSCODE_GUIDE.md`](VSCODE_GUIDE.md) instead for the setup, then come back here
> for the driving rules (section 8) and map reading (section 9).

---

## 0. First, the picture

```
   ┌───────────┐  5 wires    ┌────────────┐   WiFi (TCP)   ┌──────────────────┐
   │ RPLIDAR   │────────────▶│   ESP32    │───────────────▶│ your laptop      │
   │ A1 (2D)   │  115200 baud│            │  text scans    │                  │
   └───────────┘             └────────────┘                │ 1 record.py ──▶  │
         ▲                                                 │   SQLite (.db)   │
         │ spins, must get                                │ 2 run_slam.py    │
         │ a solid 5 V                                    │   ├ ICP matching │
   ┌─────┴──────────────────────────────────────────────┐ │   ├ g2o pose graph
   │  4-wheel robot, you push/drive it around the room  │ │   └ occupancy map
   └────────────────────────────────────────────────────┘ │        ▼         │
                                                          │  maps/room.png   │
                                                          └──────────────────┘
```

Two things surprise most people, so let us get them out of the way:

1. **The ESP32 does not do the SLAM.** It has no memory for it (a map +
   pose graph wants megabytes) and no room to run the optimiser. The ESP32's
   only job is to read the lidar and forward the data. All the maths happens
   on your laptop, in Python.
2. **The SQL database is written by your laptop, not by the ESP32.** The ESP32
   streams text over WiFi; `record.py` receives it and writes SQLite rows. This
   is the normal way to build such a robot, and it means you can re-run the SLAM
   as many times as you like on the same recording.

---

## 1. What you need

Hardware:

| Part | Notes |
| --- | --- |
| RPLIDAR A1 (A1M8) | the classic cheap 360° lidar, 12 m, ~5.5 Hz |
| ESP32 dev board | any ESP32-WROOM board (30 or 38 pins) |
| 5 V power supply, **at least 2 A** | for the lidar. A phone charger often works, a power bank works |
| Jumper wires, breadboard | 5 wires |
| USB cable for the ESP32 | for flashing and for the serial monitor |
| Your robot | 4 wheels, a flat spot on top for the lidar |

You do **not** need wheel encoders for the first map. Without them you will
drive the robot slowly and in a smooth path, and the SLAM works it out from the
lidar alone (see `06_no_encoders.md`).

Software: Python 3.9+ on the laptop, the Arduino IDE (or PlatformIO),
and this `room_slam` folder.

---

## 2. Install the software on your laptop (do this before touching hardware)

### 2.1 Python packages

Open a terminal in the `room_slam` folder and run:

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install --upgrade pip
pip install -r requirements.txt
```

> **Known trap:** `g2o-python` is compiled against numpy 1.x. With numpy 2.x,
> `import g2o` can crash with a segfault. If that happens:
> `pip install "numpy<2"` and try again. Everything still works without g2o
> (a built-in fallback solver is used), it is just slower.

### 2.2 Prove the software works, with fake data

You can do this right now, with no hardware:

```bash
python -m slam2d.simulate_room --out data/demo.db
python -m slam2d.run_slam --db data/demo.db --name demo
```

**You should now see** a summary like this, and a new file `maps/demo.png`:

```
      scans_read: 160
       keyframes: 97
           edges: 100
           loops: 4
         matched: 159
        rejected: 0
       chi2_before: 0.553
        chi2_after: 0.019
   path_length_m: 14.414
       runtime_s: 3.400
```

Open `maps/demo.png`. You should see the outline of a 6 x 5 m room with two
boxes of furniture inside, and the red path of the robot around it.

That picture is exactly what you are going to produce with your own robot.
If you got this far, 80 % of the project is already working — everything left
is cables and pushing the robot around.

### 2.3 Run the unit tests (optional but reassuring)

```bash
python -m unittest discover -s unit_test      # python side
make test-cpp                                 # ESP32 parser, compiled on your PC
```

---

## 3. Understand the four ideas (5 minutes of theory)

You do not need the maths to make this work, but knowing these four words will
make the tuning file make sense.

1. **Scan matching (ICP).** Each 360° sweep is a cloud of points in the lidar's
   own frame. If you know roughly where the robot is, you can slide and rotate
   the new cloud until it lies on top of the previous clouds. That alignment
   *is* the robot's motion. It is far more accurate than wheels or a phone's
   compass.

2. **Odometry** = "where the robot thinks it is from wheel rotation". Cheap,
   smooth, and always drifts. Here it is only used as the *starting guess* for
   scan matching. No encoders? Then the previous scan-match result is the guess
   — that is why you drive slowly.

3. **Pose graph.** Over a lap the errors pile up, so the end of the path no
   longer meets its beginning. When the robot comes back to a place it has seen
   before, we add a **loop closure** edge saying "these two poses are actually
   the same place". **g2o** then redistributes the error over the whole
   trajectory and the map snaps back together. This is what makes a *map* and
   not just a smear.

4. **Occupancy grid.** Finally each sweep is drawn into a picture where every
   pixel holds the probability that it is a wall. Rays that pass through a pixel
   vote "free"; rays that stop there vote "occupied". That picture is your map:
   `maps/room.png`, 5 cm per pixel.

The code that does all this is small: `slam2d/icp.py` (~150 lines),
`slam2d/graph.py` (~200), `slam2d/occupancy_map.py` (~200), and
`slam2d/slam.py` glues them together. `07_how_it_works.md` walks through it.

---

## 4. Wire the lidar to the ESP32

Detailed version: `01_wiring.md`. Short version:

```
RPLIDAR A1                 ESP32
---------                  -----
5V   (VCC_5V)  ─────────▶  5 V  ← from the separate 2 A supply, NOT the 3V3 pin
GND            ──────────  GND
TX  (data out) ─────────▶  GPIO16   (RX2)
RX  (data in)  ◀─────────  GPIO17   (TX2)
MOTOCTL        ─────────▶  GPIO25   (or tie it to 5 V for full speed)
motor 5V       ─────────▶  5 V supply
motor GND      ──────────  GND (same ground as everything else)
```

Rules that save you an hour:

* **All grounds together** — lidar, ESP32 and the 5 V supply must share GND.
* **Never power the lidar from the ESP32's 3V3 pin.** It needs 5 V, 4.9 V
  minimum, and up to ~0.5 A in bursts. If the voltage sags the lidar silently
  reboots and you get garbage.
* The lidar's TX/RX are 3.3 V logic, so they connect straight to the ESP32.
  No level shifter needed. **TX goes to RX** — this is the most common wiring
  mistake.
* If the ESP32 browns out or the lidar stutters, add a 470 µF capacitor across
  the lidar's 5 V/GND, and power the ESP32 from USB while the lidar has its own
  supply.

**You should now see** the lidar spinning when you power the 5 V rail (MOTOCTL
high or PWM at 255).

---

## 5. Flash the ESP32

1. Open `firmware/esp32_rplidar_bridge/esp32_rplidar_bridge.ino` in the Arduino
   IDE (keep `rplidar_parser.h` in the same folder). With PlatformIO, open the
   whole `firmware/esp32_rplidar_bridge` folder instead.
2. At the top of the sketch, fill in **four values**:

   ```cpp
   const char* WIFI_SSID = "YOUR_WIFI_NAME";
   const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
   const char* PC_HOST   = "192.168.1.50";   // your laptop's IP - see step 6
   const uint16_t PC_PORT = 8888;
   ```

3. Select board **ESP32 Dev Module**, the right COM port, and upload.
4. Open the Serial Monitor at **115200 baud**.

**You should now see** something like:

```
# esp32 rplidar bridge starting
# lidar: scan mode started
# wifi: connected, ip=192.168.1.23  laptop=192.168.1.50:8888
# tcp: connecting to 192.168.1.50:8888 ... failed (is record.py running? firewall?)
# scans=0 beams=352 good=1760 bad=0 wifi=3 client=0 heap=234512
```

`beams=352` and `bad=0` mean the lidar is being decoded correctly. `client=0`
is expected until you start the recording script — which is the next step.

---

## 6. Find your laptop's IP address

The ESP32 needs the laptop's address on the WiFi network.

| System | Command |
| --- | --- |
| Windows | `ipconfig` → look for `IPv4 Address` on the WiFi adapter |
| macOS | `ipconfig getifaddr en0` |
| Linux | `ip addr show` or `hostname -I` |

Put that address in `PC_HOST` and re-upload. Tips:

* Use a phone hotspot if your home router isolates clients (many do). The
  laptop becomes `192.168.43.x` on Android hotspots, `172.20.10.x` on iPhone.
* Give the laptop a fixed/reserved IP, otherwise the ESP32 loses the laptop
  after a router reboot. Router admin page → DHCP reservation.

Firewall: allow inbound TCP on port 8888 (Windows Defender Firewall → Advanced
settings → Inbound Rules → New Rule → Port → TCP 8888 → Allow). Details in
`03_network.md`.

---

## 7. Check the stream before you record

Terminal 1 (your laptop — this is the program that will write to SQL):

```bash
python -m slam2d.check_stream --seconds 15
```

It waits for the ESP32. Power-cycle the ESP32 (or press its reset button).
Within a few seconds it prints:

```
connected: 192.168.1.23
118 sweeps in 15.0 s  ->  7.9 Hz
beams per sweep : 352 of 360
nearest object  : 0.34 m
farthest object : 5.85 m

view from above (o = robot, # = something the lidar saw):

                ######           ########
                #                       #
                #                       #
                           #####        #
                #      ####             #
                #             o         #
                #########################
```

**You should now see** the shape of your own room in that ASCII picture. Point
the robot at a wall and watch it move. If this looks right, everything
downstream will work.

If nothing arrives: `05_troubleshooting.md`, section "no data".

---

## 8. Record the walk

Why record first and compute later? Because then you can tune the SLAM without
walking around again, and you keep a permanent dataset of your room.

Terminal 1 — start the recorder:

```bash
source .venv/bin/activate
python -m slam2d.record --db data/room.db
```

It prints `listening on 0.0.0.0:8888`. Press the ESP32 reset button. Within a
second you should see a live line:

```
   214 scans    5.42 Hz  beams=352/360  odom=0
```

To watch the recording grow (second terminal), use the bundled helper — it
needs no sqlite3 installation:

```bash
python -m slam2d.dbquery --db data/room.db --summary
```

### How to move the robot (this matters more than the code)

* Put the lidar **high** enough to see over small clutter, and keep it level.
* Drive **slowly** — 0.1–0.2 m/s. If you have no encoders, the previous scan
  match is the only motion guess, so fast motion = lost tracking.
* **Turn gently.** Prefer wide arcs. If you must turn on the spot, do it slowly
  and in small steps (< 20° at a time).
* Make **one closed lap**, ending where you started, facing the same way. The
  closing loop is what lets g2o produce a crisp map.
* Avoid people walking between the robot and the walls, and avoid rooms that
  are just one big empty box (a couple of chairs or boxes help the matcher).
* 150–300 sweeps (30–60 s of driving) is plenty for a room.

When you are done, press **Ctrl-C** in terminal 1. It prints
`[done] N scans ... -> /path/data/room.db`.

**You should now see** a recording of a few hundred scans, e.g. `250 scans,
0 odometry samples`.

---

## 9. Build the map

```bash
python -m slam2d.run_slam --db data/room.db --name room
```

**You should now see** a summary and three files:

| File | What it is |
| --- | --- |
| `maps/room.png` | the map — this is the answer to your question |
| `maps/room.g2o` | the pose graph; open it with `g2o_viewer maps/room.g2o` |
| `maps/room_grid.npz` | raw occupancy grid, for your own scripts |

The optimised poses are also written back into the database, so you can query
them like any SQL table:

```bash
python -m slam2d.dbquery --db data/room.db --summary
python -m slam2d.dbquery --db data/room.db \
    "select node_id, round(x,2), round(y,2) from poses limit 10"
python -m slam2d.dbquery --db data/room.db \
    "select kind, count(*) from graph_edges group by kind"
```

(If you do have the `sqlite3` tool, `sqlite3 data/room.db "select ..."` works too.)

A healthy run for a 5 x 5 m room with one lap looks like:

```
       keyframes: 80        <- poses in the graph
           loops: 2..6      <- revisits found
        rejected: 0..5      <- scans the matcher refused to trust
       chi2_before: ~0.5 → chi2_after: ~0.02   <- error dropped a lot
    path_length_m: ~15
       runtime_s: ~4
```

Your number of `loops` should be **at least 1** if you closed the lap. If it is
0, the map will look slightly bent — see `04_tuning.md`.

---

## 10. If the map looks wrong

Match your symptom in this table, first fix listed first.

| Symptom | Most likely cause | Fix |
| --- | --- | --- |
| Two copies of the same wall, offset | loop not closed / no encoders | close the lap; drive slower; `LOOP_MAX_DISTANCE_M = 1.2` |
| Map is blurry everywhere, walls thick | robot moved too fast, or odometry lies | drive at half speed; `ICP_MAX_CORRESPONDENCE_M = 0.3` |
| Rooms are **mirrored** left/right | lidar mounted upside down (or angle sign) | `LIDAR_ANGLE_SIGN = -1.0` in `slam2d/config.py`, record again |
| One straight wall is curving | heading drift; too few features | drive closer to walls; add furniture; slow down |
| Path jumps through a wall | scan match failed and was accepted | lower `ICP_ACCEPT_ERROR_M` to 0.05 |
| `rejected` is huge (> 50 %) | ranges clipped, or `MIN_RANGE_M` too big | check `nearest object` in `check_stream`; set `MIN_RANGE_M = 0.1` |
| Map is tiny / one blob | robot never moved | check `path_length_m` and `keyframes` |
| Everything is empty | all points out of range | `MAX_RANGE_M` too small or lidar not spinning |

More detail, including how each config value works, in `04_tuning.md`.
For everything else (WiFi, SPI, crashes) see `05_troubleshooting.md`.

---

## 11. Optional next steps

* **Live map while driving** — `python -m slam2d.record --db data/room.db
  --snapshot-every 60`. Every 60 sweeps it rebuilds `maps/live/live.png`.
* **Open the graph in g2o's own tool**: `g2o_viewer maps/room.g2o`. You built
  this repo of g2o, so `build/bin/g2o_viewer` after compiling.
* **Add wheel encoders** — two €2 hall sensors make motion smoother and let you
  drive faster. Then send `ODOM <ms> <x> <y> <theta>` lines from the ESP32 and
  the recorder will store them automatically (see `06_no_encoders.md`).
* **Compare g2o with the fallback solver**:
  `python -m slam2d.run_slam --db data/room.db --backend gn`.
* **Scale it up**: the same scripts work on a bigger flat if you record longer
  walks; just raise `SUBMAP_NUM_KEYFRAMES` and lower `KEYFRAME_MIN_DIST_M`.

---

## Appendix: every command in one place

```bash
# setup
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# fake data (no hardware)
python -m slam2d.simulate_room --out data/demo.db
python -m slam2d.run_slam --db data/demo.db --name demo

# check the ESP32 stream
python -m slam2d.check_stream --seconds 15

# record for real
python -m slam2d.record --db data/room.db
#   ... drive one slow closed lap, Ctrl-C ...

# build the map
python -m slam2d.run_slam --db data/room.db --name room --show

# look at the database
python -m slam2d.dbquery --db data/room.db --summary

# tests
python -m unittest discover -s unit_test
make test-cpp
```
