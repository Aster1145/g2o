"""Run SQL against a recording without needing the sqlite3 command line tool.

    python -m slam2d.dbquery --db data/room.db "select count(*) from scans"
    python -m slam2d.dbquery --db data/room.db "select node_id, x, y from poses limit 5"
    python -m slam2d.dbquery --db data/room.db --tables
    python -m slam2d.dbquery --db data/room.db --summary
"""

from __future__ import annotations

import argparse
import sqlite3
import sys

from . import config
from .db import connect


def print_rows(rows, headers) -> None:
    if not rows:
        print("(no rows)")
        return
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))
    line = "  ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    print(line)
    print("  ".join("-" * w for w in widths))
    for row in rows:
        print("  ".join(str(c).ljust(widths[i]) for i, c in enumerate(row)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sql", nargs="*", help="SQL statement to run")
    ap.add_argument("--db", default=config.DEFAULT_DB)
    ap.add_argument("--tables", action="store_true", help="list tables and row counts")
    ap.add_argument("--summary", action="store_true",
                    help="quick overview of the recording")
    args = ap.parse_args(argv)

    try:
        conn = connect(args.db, read_only=True)
    except sqlite3.OperationalError as exc:
        print(f"cannot open {args.db}: {exc}", file=sys.stderr)
        return 2
    conn.row_factory = sqlite3.Row

    if args.tables:
        names = [r[0] for r in conn.execute(
            "select name from sqlite_master where type='table' order by name")]
        rows = []
        for name in names:
            count = conn.execute(f"select count(*) from {name}").fetchone()[0]
            rows.append((name, count))
        print_rows(rows, ["table", "rows"])
        return 0

    if args.summary or not args.sql:
        try:
            n_scans, first, last = conn.execute(
                "select count(*), min(stamp), max(stamp) from scans").fetchone()
            beams = conn.execute(
                "select avg(n_points) from scans").fetchone()[0]
            n_odom = conn.execute("select count(*) from odom").fetchone()[0]
            n_poses = conn.execute("select count(*) from poses").fetchone()[0]
            n_loops = conn.execute(
                "select count(*) from graph_edges where kind='loop'").fetchone()[0]
        except sqlite3.OperationalError as exc:
            print(f"{args.db} has no room_slam tables yet ({exc})")
            return 1
        print_rows([("scans", n_scans),
                    ("time span [s]", f"{first or 0:.2f} .. {last or 0:.2f}"),
                    ("mean beams/sweep", f"{beams or 0:.0f}"),
                    ("odometry samples", n_odom),
                    ("optimised poses", n_poses),
                    ("loop closures", n_loops)],
                   ["key", "value"])
        return 0

    sql = " ".join(args.sql)
    try:
        cur = conn.execute(sql)
    except sqlite3.Error as exc:
        print(f"SQL error: {exc}", file=sys.stderr)
        return 1
    if cur.description:
        rows = cur.fetchall()
        print_rows([tuple(r) for r in rows], [d[0] for d in cur.description])
    else:
        print("ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
