import sqlite3
import tempfile
from pathlib import Path

from events_to_sqlite import load

SAMPLE_TSV = """segment_id\tagent_a\tagent_b\tevent_class\tstart_step\tend_step
1\t5\t5\tlane_change\t200\t245
1\t6\t6\thard_brake\t300\t340
2\t9\t10\tcrossing_paths\t595\t605
"""


def test_round_trip_count_and_values():
    with tempfile.TemporaryDirectory() as tmp:
        tsv_path = Path(tmp) / "events.tsv"
        db_path = Path(tmp) / "events.sqlite3"
        tsv_path.write_text(SAMPLE_TSV)

        n = load(str(tsv_path), str(db_path))
        assert n == 3

        conn = sqlite3.connect(str(db_path))
        rows = conn.execute("SELECT segment_id, agent_a, agent_b, event_class FROM events ORDER BY start_step").fetchall()
        assert rows == [(1, 5, 5, "lane_change"), (1, 6, 6, "hard_brake"), (2, 9, 10, "crossing_paths")]

        by_class = conn.execute(
            "SELECT event_class, COUNT(*) FROM events GROUP BY event_class ORDER BY event_class"
        ).fetchall()
        assert dict(by_class) == {"crossing_paths": 1, "hard_brake": 1, "lane_change": 1}
        conn.close()


def test_reload_overwrites_previous_table():
    with tempfile.TemporaryDirectory() as tmp:
        tsv_path = Path(tmp) / "events.tsv"
        db_path = Path(tmp) / "events.sqlite3"
        tsv_path.write_text(SAMPLE_TSV)
        load(str(tsv_path), str(db_path))
        n_second = load(str(tsv_path), str(db_path))
        assert n_second == 3
        conn = sqlite3.connect(str(db_path))
        (count,) = conn.execute("SELECT COUNT(*) FROM events").fetchone()
        assert count == 3
        conn.close()
