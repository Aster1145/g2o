"""Tests for the SLAM pipeline.  Run from room_slam/:

    python -m unittest discover -s unit_test -v

Only numpy is required for the fast tests; the demo-dataset test needs scipy
(or it just runs slower) and matplotlib is not needed at all.
"""

from __future__ import annotations

import math
import sqlite3
import tempfile
import unittest
from pathlib import Path

import numpy as np

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from slam2d import config, graph, icp, occupancy_map  # noqa: E402
from slam2d.db import (Scan, connect, fetch_scans, init_db, insert_odom,  # noqa: E402
                       insert_scan, save_graph, stats)
from slam2d.geometry import (angle_diff, inverse, normalize_angle, relative,  # noqa: E402
                             se2, transform_points, translation, xytheta)


class TestGeometry(unittest.TestCase):
    def test_inverse_roundtrip(self):
        T = se2(1.2, -0.7, 0.9)
        np.testing.assert_allclose(T @ inverse(T), np.eye(3), atol=1e-12)
        np.testing.assert_allclose(inverse(T) @ T, np.eye(3), atol=1e-12)

    def test_relative_composes(self):
        a, b = se2(0.5, 1.0, 0.3), se2(2.0, -1.0, -0.4)
        np.testing.assert_allclose(a @ relative(a, b), b, atol=1e-12)

    def test_xytheta(self):
        T = se2(3.0, 4.0, -1.5)
        x, y, t = xytheta(T)
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, 4.0)
        self.assertAlmostEqual(t, -1.5)

    def test_angle_wrap(self):
        self.assertLessEqual(abs(angle_diff(3.1, -3.1)), math.pi)
        self.assertAlmostEqual(angle_diff(0.1, 2 * math.pi - 0.1), 0.2, places=6)
        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi, places=9)


class TestIcp(unittest.TestCase):
    @staticmethod
    def synthetic_room(n=720, seed=0):
        """Points on the walls of a 6 x 5 m room."""
        rng = np.random.default_rng(seed)
        pts = []
        for _ in range(n // 4):
            pts.append((rng.uniform(0, 6), 0.0))
            pts.append((rng.uniform(0, 6), 5.0))
            pts.append((0.0, rng.uniform(0, 5)))
            pts.append((6.0, rng.uniform(0, 5)))
        return np.asarray(pts)

    def test_recovers_small_offset(self):
        world = self.synthetic_room()
        true_T = se2(0.35, -0.22, 0.08)                 # where the scan was taken
        scanned = transform_points(inverse(true_T), world)
        guess = se2(0.30, -0.20, 0.05)                  # imperfect prior
        res = icp.icp(scanned, world, T_init=guess)
        est = res.transform
        np.testing.assert_allclose(translation(est), translation(true_T), atol=0.01)
        self.assertAlmostEqual(xytheta(est)[2], 0.08, places=2)
        self.assertGreater(res.inlier_fraction, 0.9)
        self.assertLess(res.mean_error, 0.01)

    def test_rejects_impossible_alignment(self):
        world = self.synthetic_room()
        other = self.synthetic_room(seed=5) + np.array([40.0, 40.0])
        res = icp.icp(other, world, T_init=np.eye(3))
        self.assertFalse(res.converged or res.mean_error < 0.05)


class TestOccupancyMap(unittest.TestCase):
    def test_ray_marks_hit_and_free(self):
        grid = occupancy_map.OccMap.around(np.array([[2.0, 0.0]]), resolution=0.1,
                                           padding=1.0)
        grid.integrate(np.array([0.0, 0.0]), np.array([[2.0, 0.0]]))
        prob = grid.probability()
        # the hit is at x=2 m
        r, c = grid.world_to_cell(np.array([[2.0, 0.0]]))
        self.assertGreater(prob[r[0], c[0]], 0.6)
        # a cell the ray crossed must be free
        r, c = grid.world_to_cell(np.array([[1.0, 0.0]]))
        self.assertLess(prob[r[0], c[0]], 0.4)
        # a cell far off the ray stays unknown
        r, c = grid.world_to_cell(np.array([[1.0, 0.8]]))
        self.assertAlmostEqual(grid.log_odds[r[0], c[0]], 0.0)

    def test_extent_and_image(self):
        grid = occupancy_map.OccMap.around(np.array([[-1.0, -1.0], [1.0, 1.0]]),
                                           resolution=0.05, padding=0.5)
        xmin, xmax, ymin, ymax = grid.extent
        # the grid always covers the requested area, never by more than one cell
        self.assertLessEqual(xmin, -1.5 + 1e-9)
        self.assertGreaterEqual(xmax, 1.5 - 1e-9)
        self.assertLessEqual(xmax - 1.5, grid.resolution + 1e-9)
        self.assertLessEqual(-1.5 - xmin, grid.resolution + 1e-9)
        img = grid.to_image()
        self.assertEqual(img.shape, (grid.height, grid.width, 3))
        self.assertEqual(img.dtype, np.uint8)


class TestDatabase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "t.db"
        self.conn = connect(self.path)
        init_db(self.conn)

    def tearDown(self):
        self.conn.close()
        self.tmp.cleanup()

    def _scan(self, stamp, dist=2.0):
        n = 8
        angles = np.linspace(0, 2 * np.pi, n, endpoint=False, dtype=np.float32)
        return Scan(stamp=stamp, angles=angles,
                    ranges=np.full(n, dist, dtype=np.float32))

    def test_scan_roundtrip(self):
        s = self._scan(1.0)
        rowid = insert_scan(self.conn, s)
        self.assertIsNotNone(rowid)
        got = fetch_scans(self.conn)[0]
        np.testing.assert_allclose(got.ranges, s.ranges)
        np.testing.assert_allclose(got.angles, s.angles)
        self.assertEqual(got.stamp, 1.0)

    def test_duplicate_stamp_ignored(self):
        self.assertIsNotNone(insert_scan(self.conn, self._scan(1.0)))
        self.assertIsNone(insert_scan(self.conn, self._scan(1.0)))
        self.assertEqual(stats(self.conn)["scans"], 1)

    def test_odom_and_graph(self):
        insert_odom(self.conn, 0.0, 0.0, 0.0, 0.0)
        insert_odom(self.conn, 0.5, 1.0, 0.0, 0.0)
        self.assertEqual(stats(self.conn)["odom"], 2)
        sid = insert_scan(self.conn, self._scan(0.0))
        save_graph(self.conn,
                   [(0, sid, 0.0, 0.0, 0.0, 0.0),
                    (1, sid, 0.5, 1.0, 0.0, 0.0)],
                   [(0, 1, 1.0, 0.0, 0.0, "odom", None)],
                   meta={"note": "hi"})
        poses = {r["node_id"]: r for r in self.conn.execute("SELECT * FROM poses")}
        self.assertEqual(len(poses), 2)
        self.assertAlmostEqual(poses[1]["x"], 1.0)
        self.assertEqual(self.conn.execute(
            "SELECT COUNT(*) FROM graph_edges").fetchone()[0], 1)


class TestPoseGraph(unittest.TestCase):
    def test_loop_closure_pulls_trajectory_back(self):
        """A square loop: odometry drifts, one loop edge must fix it."""
        truth = [se2(0, 0, 0), se2(2, 0, 0), se2(2, 2, math.pi / 2),
                 se2(0, 2, math.pi), se2(0, 0, math.pi)]
        # measured odometry is wrong on the last leg, so the chain overshoots
        # its start position: the loop edge has to pull it back
        edges = []
        for i in range(len(truth) - 1):
            T = relative(truth[i], truth[i + 1]).copy()
            if i == 3:
                T[1, 2] += 0.45
                T[0, 2] -= 0.30
            edges.append(graph.Edge(i, i + 1, T, 0.05, 0.03, "odom"))
        edges.append(graph.Edge(4, 0, relative(truth[4], truth[0]),
                                0.05, 0.03, "loop"))

        poses = [np.eye(3)]
        for e in edges[:-1]:
            poses.append(poses[-1] @ e.T_ij)
        gap_before = np.linalg.norm(translation(poses[4]))

        out, before, after, backend, _ = graph.optimize(poses, edges, max_iter=40)
        gap_after = np.linalg.norm(translation(out[4]))
        self.assertLess(gap_after, gap_before)
        self.assertLess(after, before)

    def test_g2o_file_format(self):
        edges = [graph.Edge(0, 1, se2(1, 0, 0))]
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x.g2o"
            graph.write_g2o_file(str(p), [np.eye(3), se2(1, 0, 0)], edges)
            text = p.read_text().splitlines()
        self.assertTrue(text[0].startswith("VERTEX_SE2 0"))
        self.assertIn("FIX 0", text)
        self.assertTrue(any(line.startswith("EDGE_SE2 0 1") for line in text))


class TestUsbRecorder(unittest.TestCase):
    """The COM-port path must produce exactly the same kind of Scan."""

    def test_measurments_to_scan_bins_and_keeps_nearest(self):
        from slam2d.record_lidar_usb import measurments_to_scan

        meas = [(47, 0.0, 2000), (47, 0.0, 1200),   # same bearing: keep nearest
                (20, 90.0, 500), (20, 180.0, 0),    # zero distance = no return
                (20, 270.0, 30000)]                 # 30 m > MAX_RANGE: dropped
        scan = measurments_to_scan(meas, bins=360)
        self.assertEqual(len(scan.ranges), 360)
        self.assertEqual(len(scan.angles), 360)
        self.assertAlmostEqual(float(scan.ranges[0]), 1.2)      # nearest wins
        self.assertAlmostEqual(float(scan.ranges[90]), 0.5)
        self.assertEqual(scan.ranges[180], 0.0)                 # no return
        self.assertEqual(scan.ranges[270], 0.0)                 # out of range
        self.assertAlmostEqual(float(scan.angles[90]), math.pi / 2, places=6)

    def test_record_writes_the_database(self):
        from slam2d.record_lidar_usb import record

        class FakeLidar:
            """Stands in for rplidar.RPLidar: yields SDK-style measure tuples."""

            def __init__(self, n=6):
                self.n = n

            def iter_scans(self, min_len=5):
                for k in range(self.n):
                    yield [(47, a * 1.0, int((2000 + 10 * a) * 1.0))
                           for a in range(360)]

        with tempfile.TemporaryDirectory() as d:
            db = Path(d) / "usb.db"
            n, _ = record(FakeLidar(6), db, bins=360)
            self.assertEqual(n, 6)
            conn = connect(db, read_only=True)
            self.assertEqual(stats(conn)["scans"], 6)
            got = fetch_scans(conn)[0]
            self.assertEqual(len(got.ranges), 360)
            self.assertTrue(np.all(got.ranges > 0))
            conn.close()


class TestLikelihoodSearch(unittest.TestCase):
    """The no-odometry search must find a pose without a motion prior."""

    @staticmethod
    def room_cloud(n=600, seed=1):
        rng = np.random.default_rng(seed)
        pts = []
        for _ in range(n // 4):
            pts.append((rng.uniform(0, 6), 0.0))
            pts.append((rng.uniform(0, 6), 5.0))
            pts.append((0.0, rng.uniform(0, 5)))
            pts.append((6.0, rng.uniform(0, 5)))
        return np.asarray(pts)

    def test_field_scores_map_points_highly(self):
        from slam2d.likelihood import LikelihoodField

        cloud = self.room_cloud()
        field = LikelihoodField.from_points(cloud)
        self.assertIsNotNone(field)
        self.assertGreater(field.score(cloud), 0.9)
        far = cloud + np.array([40.0, 40.0])
        self.assertLess(field.score(far), 0.2)

    def test_search_recovers_a_large_offset(self):
        """A 30 cm / 12 deg offset is far beyond ICP's capture range."""
        from slam2d.likelihood import LikelihoodField, search_poses
        from slam2d.geometry import se2, transform_points

        world = self.room_cloud()
        truth = se2(0.31, -0.27, math.radians(12.0))
        # the scan as seen from `truth`, but we only know "roughly here"
        scan = transform_points(inverse(truth), world)
        field = LikelihoodField.from_points(world)
        guess = se2(0.0, 0.0, 0.0)

        cands = search_poses(scan, field, guess, window_xy=0.5, window_deg=25.0,
                             step_xy=0.10, step_deg=3.0, top_k=3)
        self.assertTrue(cands)
        best = cands[0][1]
        err = np.linalg.norm(translation(best) - translation(truth))
        self.assertLess(err, 0.15)              # coarse grid is 10 cm


class TestNoOdomPipeline(unittest.TestCase):
    def test_map_built_without_any_odometry(self):
        from slam2d.simulate_room import write_demo_scan
        from slam2d.slam import run_slam

        with tempfile.TemporaryDirectory() as d:
            db = write_demo_scan(Path(d) / "demo.db", n_scans=110, n_beams=240)
            res = run_slam(db, use_odom=False, verbose=False)
            self.assertGreater(res.stats["keyframes"], 25)
            self.assertGreaterEqual(res.stats["loops"], 1)
            self.assertLess(res.stats["rejected"], 10)
            # the estimated path must be close to the true loop length (~14 m)
            self.assertGreater(res.stats["path_length_m"], 10.0)


class TestEndToEnd(unittest.TestCase):
    """Generate a small simulated recording and map it."""

    def test_demo_dataset_builds_a_closed_map(self):
        from slam2d.simulate_room import write_demo_scan
        from slam2d.slam import run_slam

        with tempfile.TemporaryDirectory() as d:
            db = write_demo_scan(Path(d) / "demo.db", n_scans=120, n_beams=240)
            res = run_slam(db, verbose=False)
            self.assertGreater(res.stats["keyframes"], 20)
            self.assertGreater(res.stats["matched"], 100)
            self.assertGreaterEqual(res.stats["loops"], 1)
            self.assertLess(res.stats["chi2_after"], res.stats["chi2_before"] + 1e-9)
            traj = res.trajectory()
            # a closed lap must end near where it started (within 40 cm)
            self.assertLess(np.linalg.norm(traj[-1] - traj[0]), 0.4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
