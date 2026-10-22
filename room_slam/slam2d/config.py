"""All tunable numbers for the room-SLAM pipeline live here.

Every value has a sensible default.  The two you will most likely change on a
real robot are LIDAR_OFFSET_* (where the lidar sits relative to the rotation
centre of the robot) and the ICP_* tolerances if your lidar is noisier.
"""

# --------------------------------------------------------------------------
# Lidar / sensor
# --------------------------------------------------------------------------
# Ranges outside this window are thrown away.  RPLIDAR A1: 0.15 .. 12 m.
MIN_RANGE_M = 0.15
MAX_RANGE_M = 12.0

# Points with a low return quality are usually glass / dark surfaces / far
# walls.  RPLIDAR A1 reports 0..63; keep everything if you are unsure.
MIN_QUALITY = 0

# Where the lidar is mounted, relative to the point the robot rotates about
# (centre of the wheel axle, in the robot frame: +x forward, +y left).
# Leave at zero if you do not know it yet - the map will still be fine for a
# first run, it only matters once the robot turns tightly.
# If the finished map comes out MIRRORED (left/right swapped), set the sign to
# -1.0 and record again.  If it is rotated by a constant amount, use the offset.
# You should not need to touch these with the provided ESP32 firmware.
LIDAR_ANGLE_SIGN = 1.0
LIDAR_ANGLE_OFFSET_DEG = 0.0

LIDAR_OFFSET_X = 0.0
LIDAR_OFFSET_Y = 0.0
LIDAR_OFFSET_YAW = 0.0

# --------------------------------------------------------------------------
# Scan matching (ICP)
# --------------------------------------------------------------------------
ICP_MAX_ITER = 40
ICP_MAX_CORRESPONDENCE_M = 0.5   # reject point pairs further apart than this
ICP_TRIM_FRACTION = 0.80         # keep the best 80 % of the pairs
ICP_CONVERGED_M = 1e-5           # stop when the correction gets this small

# A match is accepted when the mean error is below this and enough points
# found a partner.  Raise ACCEPT_ERROR if your environment is cluttered.
ICP_ACCEPT_ERROR_M = 0.08
ICP_ACCEPT_INLIER_FRACTION = 0.50

# A scan match may only move the predicted pose by this much.  It is a cheap
# sanity check that catches "ICP snapped onto the wrong wall" failures.
ICP_MAX_CORRECTION_M = 0.60
ICP_MAX_CORRECTION_DEG = 25.0

# Loop closure detection
LOOP_MIN_NODE_GAP = 25           # do not close a loop with a scan this recent
LOOP_MAX_DISTANCE_M = 0.80       # only look for a loop this close (odometry)
LOOP_MAX_HEADING_DEG = 45.0
LOOP_ACCEPT_ERROR_M = 0.08
LOOP_ACCEPT_INLIER_FRACTION = 0.60

# --------------------------------------------------------------------------
# Pose graph optimisation (g2o)
# --------------------------------------------------------------------------
G2O_MAX_ITERATIONS = 30

ODOM_SIGMA_XY = 0.05             # metres, 1-sigma per odometry edge
ODOM_SIGMA_THETA = 0.03          # radians
LOOP_SIGMA_XY = 0.08
LOOP_SIGMA_THETA = 0.03

# --------------------------------------------------------------------------
# Occupancy map
# --------------------------------------------------------------------------
MAP_RESOLUTION_M = 0.05          # one pixel = 5 cm
MAP_PADDING_M = 0.60             # empty border around the map
MAP_LOG_ODDS_HIT = 0.85
MAP_LOG_ODDS_MISS = -0.40
MAP_LOG_ODDS_MIN = -2.0
MAP_LOG_ODDS_MAX = 3.5

# --------------------------------------------------------------------------
# Networking: the ESP32 connects to this TCP port on your laptop
# --------------------------------------------------------------------------
TCP_PORT = 8888

# --------------------------------------------------------------------------
# Files
# --------------------------------------------------------------------------
DEFAULT_DB = "data/room.db"
DEFAULT_MAP_DIR = "maps"

# --------------------------------------------------------------------------
# Keyframes and submaps
# --------------------------------------------------------------------------
KEYFRAME_MIN_DIST_M = 0.10       # new keyframe after moving this far ...
KEYFRAME_MIN_ROT_RAD = 0.10      # ... or turning this much (~6 deg)
SUBMAP_NUM_KEYFRAMES = 15        # recent keyframes used for scan matching
SUBMAP_RADIUS_M = 6.0            # crop the matching cloud to this radius
LOOP_SUBMAP_RADIUS_M = 4.0
