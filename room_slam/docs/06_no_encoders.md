# What if the robot has no wheel encoders?

Short answer: it still works. The pipeline is designed so odometry is only a
*starting guess*, never the truth.

```
with encoders:      guess = previous pose + wheel motion     (smooth, accurate short-term)
without encoders:   guess = previous pose + last scan match  (smooth enough if slow)
```

`slam2d/slam.py` does exactly this: if the `odom` table is empty it predicts the
next pose by repeating the last measured motion (`last_motion`). That assumption
is excellent for a robot driving smoothly and terrible for a robot that jerks or
spins on the spot.

## How to drive without encoders

| Do | Don't |
| --- | --- |
| 0.1–0.2 m/s | full speed |
| wide arcs (turn radius > 0.5 m) | spin on the spot |
| pause 0.5 s before turning | start a turn while still rolling |
| one continuous lap | stop-and-go |

Rule of thumb: between two lidar sweeps the robot should move less than
**10 cm** and turn less than **5°**. At 5.5 Hz that is 0.55 m/s and 165°/s —
generous in theory, but leave headroom: aim for 0.15 m/s.

You can also record "ghost odometry": drive in short straight segments and, after
each segment, insert the nominal command into the database yourself. This is
rarely worth it — the scan matcher is better than the command.

## When you do want encoders

Two cheap options for a 4-wheel robot:

1. **Hall-effect / optical slot sensors** (one per driven wheel). ~2 € each,
   ~20 pulses per revolution → with a 65 mm wheel that is ~1 cm per pulse.
   Count with an ESP32 interrupt, compute `x, y, theta` by dead reckoning, and
   print `ODOM <millis> <x> <y> <theta>` once per sweep. The recorder stores it
   automatically and the SLAM starts using it.

   ```cpp
   // in the sketch, after sending END for a scan:
   client.printf("ODOM %lu %.4f %.4f %.4f\n",
                 millis(), odom_x, odom_y, odom_theta);
   ```

2. **Use the lidar itself as the encoder.** That is what this project does by
   default. The pose graph then smooths out the small errors.

Important: `x, y, theta` must be in a **consistent arbitrary frame** starting at
0,0,0, in metres and radians. The SLAM only uses relative motion, so the origin
does not matter and no calibration to the map frame is needed.

## Calibrating cheap encoders (if you add them)

* Measure the wheel diameter and count the real distance over 5 m. Scale error
  shows up as a map that is 5 % too long — harmless, but easy to fix.
* Do not rely on the encoder's heading. Wheel slip on carpet makes it drift
  fast; the scan matcher corrects it.
* Keep `ODOM_SIGMA_XY` moderate (0.03–0.05). If you pretend the encoders are
  perfect (0.001), a single slip event poisons the whole graph.
