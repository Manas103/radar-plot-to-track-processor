from maneuver_search.corpus_io import ManeuverSegment
from maneuver_search.labeling import is_relevant


def _seg(label, lane_shift_m, duration_s):
    return ManeuverSegment(
        segment_id=1, label=label, base_speed=14.0, lane_shift_m=lane_shift_m,
        duration_s=duration_s, has_stop=False, end_speed=14.0, x_at_detection=-500.0,
    )


def test_identical_segments_are_relevant_to_themselves():
    s = _seg("lane_change", 3.5, 4.5)
    assert is_relevant(s, s)


def test_different_class_is_never_relevant():
    a = _seg("lane_change", 3.5, 4.5)
    b = _seg("pullover", 3.5, 4.5)
    assert not is_relevant(a, b)


def test_same_class_within_tolerance_is_relevant():
    a = _seg("lane_change", 3.5, 4.5)
    b = _seg("lane_change", 3.8, 4.8)
    assert is_relevant(a, b)


def test_same_class_far_outside_tolerance_is_not_relevant():
    a = _seg("lane_change", 3.5, 4.5)
    b = _seg("lane_change", 14.0, 4.5)  # 4x the lane shift magnitude
    assert not is_relevant(a, b)


def test_relevance_is_symmetric():
    a = _seg("pullover", 7.0, 8.0)
    b = _seg("pullover", 7.5, 8.5)
    assert is_relevant(a, b) == is_relevant(b, a)
