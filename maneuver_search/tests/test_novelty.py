import numpy as np

from maneuver_search.index import build_index
from maneuver_search.novelty import (
    calibrate_novelty_threshold,
    generate_injected_novel_raw_vectors,
    novelty_scores,
)


def _in_distribution_vectors(rng, n=2000):
    # mimics real planted-maneuver feature ranges: base_speed 10-18,
    # lane_shift_m 3.5-14, duration_s ~4.5, has_stop/in_pudo_zone in {0,1}
    base_speed = rng.uniform(10.0, 18.0, size=n)
    lane_shift_m = rng.uniform(3.5, 14.0, size=n)
    duration_s = rng.normal(4.5, 0.3, size=n)
    has_stop = rng.integers(0, 2, size=n).astype(np.float32)
    end_speed = np.where(has_stop == 1.0, rng.uniform(0.0, 2.0, size=n), base_speed)
    in_zone = rng.integers(0, 2, size=n).astype(np.float32)
    return np.stack([base_speed, lane_shift_m, duration_s, has_stop, end_speed, in_zone], axis=1).astype(np.float32)


def test_injected_vectors_have_higher_novelty_score_than_in_distribution_points():
    rng = np.random.default_rng(10)
    in_dist = _in_distribution_vectors(rng)
    idx = build_index(in_dist, ef_search=128)

    threshold = calibrate_novelty_threshold(idx, in_dist, percentile=99.0)

    in_dist_scores = novelty_scores(idx, in_dist[:200])
    injected = generate_injected_novel_raw_vectors(20, rng)
    injected_scores = novelty_scores(idx, injected.astype(np.float32))

    assert injected_scores.mean() > in_dist_scores.mean()
    assert injected_scores.mean() > threshold


def test_in_distribution_points_mostly_fall_under_their_own_calibrated_threshold():
    rng = np.random.default_rng(11)
    in_dist = _in_distribution_vectors(rng)
    idx = build_index(in_dist, ef_search=128)
    threshold = calibrate_novelty_threshold(idx, in_dist, percentile=99.0)

    held_out_scores = novelty_scores(idx, in_dist[200:400])
    frac_over = np.mean(held_out_scores > threshold)
    # a 99th-percentile threshold calibrated on the same distribution
    # should flag only a small minority of a held-out in-distribution sample
    assert frac_over < 0.15


def test_generate_injected_vectors_shape_and_internal_consistency():
    rng = np.random.default_rng(12)
    injected = generate_injected_novel_raw_vectors(20, rng)
    assert injected.shape == (20, 6)
    # has_stop==1 rows must have a near-zero end_speed (column index 4)
    stopped = injected[injected[:, 3] == 1.0]
    assert np.all(stopped[:, 4] <= 1.0)
