"""room_slam - a small, readable 2D SLAM stack for a differential-drive robot
with an RPLIDAR and an ESP32.

Modules
-------
config            every tunable number in one file
db                SQLite storage (raw scans + odometry + results)
icp               2D scan matching
graph             pose graph optimisation with g2o
occupancy_map     the grid map + PNG output
slam              the pipeline that glues the above together
record            TCP / serial receiver for the ESP32 stream
run_slam          command line entry point
simulate_room     fake data so you can test without hardware
"""

__version__ = "0.1.0"
