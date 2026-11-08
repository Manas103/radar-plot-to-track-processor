"""Inverts the same hnswlib index used for retrieval into a novelty
detector: a segment's novelty score is its distance to its single
nearest neighbor in the index, and a segment is flagged novel if that
distance exceeds a threshold calibrated from the in-distribution corpus
itself (a percentile of in-distribution 1-NN distances, excluding each
point's own self-match).

"Injected unseen scene types" are synthetic feature vectors built from
combinations of raw kinematic values well outside anything the three
planted maneuver classes ever produce (see
`generate_injected_novel_raw_vectors`): a much larger lane shift than any
real lane change plants, a much longer maneuver duration, and a highway
cruising speed none of the mined segments use. These stand in for "a
maneuver type the corpus never planted", the honest substitute available
here for a genuinely novel real-world scene.
"""
from __future__ import annotations

import numpy as np

from maneuver_search.index import ManeuverIndex


def calibrate_novelty_threshold(
    index: ManeuverIndex, in_distribution_vectors: np.ndarray, percentile: float = 99.0
) -> float:
    """1-NN distance (excluding self) for every in-distribution vector,
    then the given percentile of that distribution: the threshold above
    which a 1-NN distance is treated as novel.
    """
    labels, distances = index.index.knn_query(in_distribution_vectors, k=2)
    # column 0 is almost always the point itself (distance ~0); take
    # column 1, the true nearest DIFFERENT neighbor. Guard the rare case
    # where the self-match is not first (duplicate points) by taking the
    # max of the two, which is always the non-self distance in that case.
    nn_dist = np.where(distances[:, 0] < 1e-9, distances[:, 1], np.maximum(distances[:, 0], distances[:, 1]))
    return float(np.percentile(nn_dist, percentile))


def novelty_scores(index: ManeuverIndex, query_vectors: np.ndarray) -> np.ndarray:
    _labels, distances = index.index.knn_query(query_vectors, k=1)
    return distances[:, 0]


def generate_injected_novel_raw_vectors(n: int, rng: np.random.Generator) -> np.ndarray:
    """Raw (pre-scaling) feature vectors for a scene type the mining
    corpus never plants: a somewhat larger lane shift and somewhat
    longer maneuver duration than anything planted (versus the planted
    1-4 lanes / ~4.5s), and a cruising speed just outside the planted
    10-18 m/s range. These are deliberately near, not far past, the edge
    of the planted distribution: a scene type that is genuinely new but
    only moderately so is the harder, more honest version of this test,
    and is why the catch rate below is not a trivial 20/20 (see README
    "What broke" for the two more extreme designs that were tried and
    rejected as too easy). has_stop and end_speed are kept internally
    consistent (a stopped vehicle has end_speed near 0).
    """
    out = np.empty((n, 6), dtype=np.float32)
    for i in range(n):
        base_speed = rng.uniform(19.0, 24.0)
        lane_shift_m = rng.uniform(16.0, 21.0)
        duration_s = rng.uniform(7.0, 9.5)
        has_stop = 1.0 if rng.random() < 0.5 else 0.0
        end_speed = rng.uniform(0.0, 1.0) if has_stop else base_speed
        in_pudo_zone = 1.0 if rng.random() < 0.5 else 0.0
        out[i] = [base_speed, lane_shift_m, duration_s, has_stop, end_speed, in_pudo_zone]
    return out
