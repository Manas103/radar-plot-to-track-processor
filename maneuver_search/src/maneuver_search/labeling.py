"""The "hand-labeled" relevance proxy used to measure precision@10.

This project has no human labeler; a 600-pair relevance label set is
produced here by a fixed, disclosed scripted rule, not by a person, in
the same spirit as the ground-truth-labeling-autolabeler-eval repo's
"scripted annotator proxy, not a human". The rule: a candidate is
relevant to a query if both are the same maneuver class AND their
lane-shift magnitude and maneuver duration are each within a stated
relative tolerance of each other. This is a reasonable, bright-line
proxy for "would a human reviewer call these two maneuvers the same kind
of event", but it is a rule, not a judgment call, and it is the whole
labeling budget this project has.
"""
from __future__ import annotations

from maneuver_search.corpus_io import ManeuverSegment

LANE_SHIFT_RELATIVE_TOLERANCE = 0.25
DURATION_RELATIVE_TOLERANCE = 0.35


def _within_relative_tolerance(a: float, b: float, tol: float) -> bool:
    denom = max(abs(a), abs(b), 1e-6)
    return abs(a - b) / denom <= tol


def is_relevant(query: ManeuverSegment, candidate: ManeuverSegment) -> bool:
    if query.label != candidate.label:
        return False
    if not _within_relative_tolerance(query.lane_shift_m, candidate.lane_shift_m, LANE_SHIFT_RELATIVE_TOLERANCE):
        return False
    if not _within_relative_tolerance(query.duration_s, candidate.duration_s, DURATION_RELATIVE_TOLERANCE):
        return False
    return True
