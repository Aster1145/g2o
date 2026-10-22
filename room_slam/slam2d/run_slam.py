"""Build the 2D SLAM map from a recording.

    python -m slam2d.run_slam --db data/room.db
    python -m slam2d.run_slam --db data/room.db --show
    python -m slam2d.run_slam --db data/demo.db --name demo

Outputs (in ``maps/`` by default):

    room.png       the map image, 5 cm per pixel
    room.g2o       the pose graph - open it with g2o_viewer
    room_grid.npz  raw log-odds grid, for your own scripts

The optimised poses are also written back into the database (``poses`` and
``graph_edges`` tables) so you can inspect them with plain SQL.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import config
from .slam import run_slam, save_results


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", default=config.DEFAULT_DB, help="recording to process")
    ap.add_argument("--out", default=config.DEFAULT_MAP_DIR, help="output folder")
    ap.add_argument("--name", default="room", help="base name of the output files")
    ap.add_argument("--max-scans", type=int, default=None,
                    help="only use the first N scans (quick experiments)")
    ap.add_argument("--no-odom", action="store_true",
                    help="ignore wheel odometry, use scan matching only")
    ap.add_argument("--no-loops", action="store_true", help="disable loop closure")
    ap.add_argument("--no-optimize", action="store_true",
                    help="skip g2o, just use the scan-matching chain")
    ap.add_argument("--backend", default="auto", choices=["auto", "g2o", "gn"],
                    help="pose graph solver (default: g2o if installed)")
    ap.add_argument("--show", action="store_true", help="open a matplotlib window")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    db = Path(args.db)
    if not db.exists():
        print(f"error: {db} not found.\n"
              f"  record data first:  python -m slam2d.record --db {db}\n"
              f"  or make a demo:     python -m slam2d.simulate_room --out data/demo.db",
              file=sys.stderr)
        return 2

    print(f"SLAM on {db}")
    result = run_slam(db, use_odom=not args.no_odom,
                      max_scans=args.max_scans,
                      loop_closure=not args.no_loops,
                      optimize_graph=not args.no_optimize,
                      backend=args.backend,
                      verbose=not args.quiet)
    files = save_results(result, out_dir=args.out, db_path=db, name=args.name)

    print("\n--- summary ---")
    for key, value in result.stats.items():
        if isinstance(value, float):
            print(f"{key:>16}: {value:.3f}")
        else:
            print(f"{key:>16}: {value}")
    print(f"{'map image':>16}: {files['png']}")
    print(f"{'pose graph':>16}: {files['g2o']}   (g2o_viewer {files['g2o']})")
    print(f"{'grid data':>16}: {files['npz']}")
    print(f"{'sqlite rows':>16}: poses / graph_edges updated in {db}")

    if args.show:
        try:
            from .occupancy_map import show_map
            show_map(result.grid, result.poses, title=Path(files["png"]).name)
        except Exception as exc:                       # headless machine
            print(f"\n(could not open a window: {exc})")
            print(f"open the image yourself: {files['png']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
