"""Loads autolabel_cli's event TSV output into a SQLite database. The C++
side never links SQLite directly (section 2 of BUILDER.md's repository
hygiene rule against unjustified heavy vendored dependencies: the SQLite
amalgamation is an 8MB+ single file, and this repo's whole budget is 65 to
500KB); the storage format the project claims is this: mine to a plain TSV,
then load it, which is one `sqlite3.connect()` call away from queryable.

Usage: python3 events_to_sqlite.py <events.tsv> <out.sqlite3>
"""
import sqlite3
import sys


def load(events_tsv_path: str, db_path: str) -> int:
    conn = sqlite3.connect(db_path)
    conn.execute("DROP TABLE IF EXISTS events")
    conn.execute(
        """
        CREATE TABLE events (
            segment_id INTEGER NOT NULL,
            agent_a INTEGER NOT NULL,
            agent_b INTEGER NOT NULL,
            event_class TEXT NOT NULL,
            start_step INTEGER NOT NULL,
            end_step INTEGER NOT NULL
        )
        """
    )
    rows = []
    with open(events_tsv_path) as f:
        next(f)  # header
        for line in f:
            seg_id, agent_a, agent_b, event_class, start_step, end_step = line.rstrip("\n").split("\t")
            rows.append((int(seg_id), int(agent_a), int(agent_b), event_class, int(start_step), int(end_step)))
    conn.executemany("INSERT INTO events VALUES (?, ?, ?, ?, ?, ?)", rows)
    conn.execute("CREATE INDEX idx_events_segment ON events(segment_id)")
    conn.execute("CREATE INDEX idx_events_class ON events(event_class)")
    conn.commit()
    conn.close()
    return len(rows)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: events_to_sqlite.py <events.tsv> <out.sqlite3>", file=sys.stderr)
        sys.exit(2)
    n = load(sys.argv[1], sys.argv[2])
    print(f"loaded {n} events into {sys.argv[2]}")
