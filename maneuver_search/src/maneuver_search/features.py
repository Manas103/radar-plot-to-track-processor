"""Turns each ManeuverSegment's kinematics into a fixed-length numeric
embedding: base speed, lane-shift magnitude, maneuver duration, whether it
ends in a stop, the ending speed, and whether the detection point falls
inside the declared PUDO zone. Six features, standardized (zero mean,
unit variance) with scikit-learn's StandardScaler before indexing, since
hnswlib's L2 distance is not scale-invariant and these six features live
on very different native scales (speed in m/s, duration in seconds, a
boolean).
"""
from __future__ import annotations

import numpy as np
from sklearn.preprocessing import StandardScaler

from maneuver_search.corpus_io import ManeuverSegment

PUDO_ZONE_X_START_M = 1200.0
PUDO_ZONE_X_END_M = 1400.0

FEATURE_NAMES = [
    "base_speed",
    "lane_shift_m",
    "duration_s",
    "has_stop",
    "end_speed",
    "in_pudo_zone",
]


def segment_to_raw_feature_vector(seg: ManeuverSegment, rng: np.random.Generator | None = None) -> np.ndarray:
    """The continuous fields carry simulated sensor/measurement noise
    (a real similarity search never indexes the exact ground-truth
    kinematics, only what a perception stack measured); the two boolean
    fields (has_stop, in_pudo_zone) are reported exactly, since those are
    discrete decisions the same autolabel miner that detected the segment
    already made, not re-measured here. `labeling.is_relevant` is checked
    against the UNNOISED ManeuverSegment fields, so retrieval quality is
    genuinely bounded by how much the noise can scramble the embedding,
    not a property this project can tune away.
    """
    in_zone = 1.0 if PUDO_ZONE_X_START_M <= seg.x_at_detection <= PUDO_ZONE_X_END_M else 0.0
    base_speed = seg.base_speed
    lane_shift_m = seg.lane_shift_m
    duration_s = seg.duration_s
    end_speed = seg.end_speed
    if rng is not None:
        base_speed += rng.normal(0.0, 2.0)
        lane_shift_m += rng.normal(0.0, 2.4)
        duration_s += rng.normal(0.0, 1.2)
        end_speed += rng.normal(0.0, 0.65)
    return np.array(
        [
            base_speed,
            lane_shift_m,
            duration_s,
            1.0 if seg.has_stop else 0.0,
            end_speed,
            in_zone,
        ],
        dtype=np.float32,
    )


def build_feature_matrix(
    segments: list[ManeuverSegment], rng: np.random.Generator | None = None
) -> tuple[np.ndarray, StandardScaler]:
    """Returns (standardized feature matrix, the fitted scaler), so the
    identical scaler can be reused to transform a held-out query or an
    injected novel-scene vector at retrieval time."""
    raw = np.stack([segment_to_raw_feature_vector(s, rng) for s in segments])
    scaler = StandardScaler()
    standardized = scaler.fit_transform(raw).astype(np.float32)
    return standardized, scaler
