import numpy as np

from maneuver_search.corpus_io import ManeuverSegment
from maneuver_search.features import FEATURE_NAMES, build_feature_matrix, segment_to_raw_feature_vector


def _seg(label="lane_change", base_speed=14.0, lane_shift_m=3.5, duration_s=4.5, has_stop=False,
         end_speed=14.0, x_at_detection=-500.0):
    return ManeuverSegment(
        segment_id=1, label=label, base_speed=base_speed, lane_shift_m=lane_shift_m,
        duration_s=duration_s, has_stop=has_stop, end_speed=end_speed, x_at_detection=x_at_detection,
    )


def test_raw_vector_has_six_named_features():
    v = segment_to_raw_feature_vector(_seg())
    assert v.shape == (6,)
    assert len(FEATURE_NAMES) == 6


def test_in_pudo_zone_flag_set_correctly():
    inside = segment_to_raw_feature_vector(_seg(x_at_detection=1300.0))
    outside = segment_to_raw_feature_vector(_seg(x_at_detection=50.0))
    assert inside[FEATURE_NAMES.index("in_pudo_zone")] == 1.0
    assert outside[FEATURE_NAMES.index("in_pudo_zone")] == 0.0


def test_has_stop_flag_matches_segment():
    stopped = segment_to_raw_feature_vector(_seg(has_stop=True))
    moving = segment_to_raw_feature_vector(_seg(has_stop=False))
    assert stopped[FEATURE_NAMES.index("has_stop")] == 1.0
    assert moving[FEATURE_NAMES.index("has_stop")] == 0.0


def test_build_feature_matrix_is_standardized():
    segments = [
        _seg(base_speed=b, lane_shift_m=s, duration_s=d)
        for b, s, d in zip(np.linspace(10, 18, 50), np.linspace(3.5, 14.0, 50), np.linspace(1.0, 10.0, 50))
    ]
    matrix, scaler = build_feature_matrix(segments)
    assert matrix.shape == (50, 6)
    # standardized columns (where there is variance) have ~zero mean, ~unit std
    stds = matrix.std(axis=0)
    varying_cols = stds > 1e-6
    assert np.allclose(matrix[:, varying_cols].mean(axis=0), 0.0, atol=1e-5)
    assert np.allclose(stds[varying_cols], 1.0, atol=1e-5)
    assert scaler.n_features_in_ == 6
