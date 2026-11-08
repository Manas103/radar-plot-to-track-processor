import shutil
import tempfile
from pathlib import Path

import pytest

from maneuver_search.corpus_io import load_maneuver_segments, run_mine_corpus

CLI_CANDIDATES = [
    Path(__file__).resolve().parents[2] / "build" / "autolabel_cli",
    Path(__file__).resolve().parents[2] / "build" / "autolabel_cli.exe",
]


def _find_cli() -> str | None:
    for c in CLI_CANDIDATES:
        if c.exists():
            return str(c)
    return None


@pytest.fixture(scope="module")
def small_segments():
    cli = _find_cli()
    if cli is None:
        pytest.skip("autolabel_cli binary not found; build the C++ side first (see README)")
    with tempfile.TemporaryDirectory() as tmp:
        corpus_path = str(Path(tmp) / "corpus.tsv")
        events_path = str(Path(tmp) / "events.tsv")
        try:
            run_mine_corpus(cli, corpus_path, events_path, num_segments=500, seed=7)
        except OSError as e:
            # the binary at CLI_CANDIDATES is an ELF built under WSL2; this
            # interpreter's OS cannot execute it (see README "Honest
            # framing"). Run this test file under the WSL2 venv instead.
            pytest.skip(f"autolabel_cli is not executable on this platform: {e}")
        return load_maneuver_segments(corpus_path, events_path)


def test_every_segment_gets_one_of_the_three_labels(small_segments):
    assert len(small_segments) > 0
    assert {s.label for s in small_segments} <= {"lane_change", "pullover", "pickup_dropoff_approach"}


def test_segment_count_does_not_exceed_requested_corpus_size(small_segments):
    assert len(small_segments) <= 500


def test_pullover_and_pdo_segments_have_a_near_zero_end_speed(small_segments):
    for s in small_segments:
        if s.has_stop:
            assert s.end_speed <= 2.0 + 1e-6


def test_lane_change_only_segments_do_not_report_has_stop(small_segments):
    for s in small_segments:
        if s.label == "lane_change":
            assert not s.has_stop


def test_pdo_segments_detection_point_falls_inside_declared_zone(small_segments):
    pdo = [s for s in small_segments if s.label == "pickup_dropoff_approach"]
    for s in pdo:
        assert 1200.0 <= s.x_at_detection <= 1400.0
