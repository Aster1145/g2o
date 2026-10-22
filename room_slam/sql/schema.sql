/* room_slam database schema (SQLite).

   Two kinds of data live here:

     1. RAW DATA, written by the ESP32 bridge:        scans, odom
     2. RESULTS, written back by the SLAM script:     poses, graph_edges, map_meta

   The raw tables are your recording - never throw them away, you can always
   re-run SLAM on them with different settings.  The result tables are just
   overwritten each time you run.
*/

PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;

-- One row per 360 degree lidar sweep.
CREATE TABLE IF NOT EXISTS scans (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    stamp     REAL    NOT NULL,          -- seconds (ESP32 millis / 1000)
    n_points  INTEGER NOT NULL,
    ranges    BLOB    NOT NULL,          -- float32 LE, metres, 0 = invalid
    angles    BLOB    NOT NULL,          -- float32 LE, radians
    quality   BLOB,                      -- uint8, optional
    node_id   INTEGER,                   -- filled in by SLAM
    UNIQUE (stamp)
);

CREATE INDEX IF NOT EXISTS idx_scans_stamp ON scans (stamp);

-- One row per wheel-odometry sample (or per manual waypoint if you have no
-- wheel encoders yet - see docs/06_no_encoders.md).
CREATE TABLE IF NOT EXISTS odom (
    id     INTEGER PRIMARY KEY AUTOINCREMENT,
    stamp  REAL NOT NULL,
    x      REAL NOT NULL,                -- metres, odom frame
    y      REAL NOT NULL,
    theta  REAL NOT NULL,                -- radians
    source TEXT DEFAULT 'wheel'
);

CREATE INDEX IF NOT EXISTS idx_odom_stamp ON odom (stamp);

-- SLAM result: optimised robot pose for every keyframe.
CREATE TABLE IF NOT EXISTS poses (
    node_id  INTEGER PRIMARY KEY,
    scan_id  INTEGER REFERENCES scans (id),
    stamp    REAL,
    x        REAL NOT NULL,
    y        REAL NOT NULL,
    theta    REAL NOT NULL,
    kind     TEXT DEFAULT 'odom'          -- 'odom' = raw, 'slam' = optimised
);

-- SLAM result: every edge of the pose graph, for inspection / re-plotting.
CREATE TABLE IF NOT EXISTS graph_edges (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    from_node INTEGER NOT NULL,
    to_node   INTEGER NOT NULL,
    dx        REAL NOT NULL,
    dy        REAL NOT NULL,
    dtheta    REAL NOT NULL,
    kind      TEXT NOT NULL,              -- 'odom' or 'loop'
    chi2      REAL
);

CREATE TABLE IF NOT EXISTS map_meta (
    key   TEXT PRIMARY KEY,
    value TEXT
);
