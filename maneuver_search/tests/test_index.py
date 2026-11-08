import numpy as np

from maneuver_search.index import brute_force_top_k, build_index, query_top_k


def test_hnswlib_matches_brute_force_reference_oracle_on_small_data():
    """Correctness oracle: hnswlib's approximate top-k must agree with an
    independent, from-scratch exact nearest-neighbor computation when the
    index is given a generous ef_search on a small dataset, where
    "approximate" and "exact" should coincide almost everywhere."""
    rng = np.random.default_rng(3)
    vectors = rng.normal(size=(500, 6)).astype(np.float32)
    idx = build_index(vectors, ef_construction=400, m=32, ef_search=400)

    queries = vectors[:50]
    hnsw_labels, _ = query_top_k(idx, queries, k=10)
    exact_labels = brute_force_top_k(vectors, queries, k=10, exclude_self=False)

    # compare as sets per-query (hnswlib does not guarantee tie order)
    agreement = np.mean(
        [len(set(hnsw_labels[i]) & set(exact_labels[i])) / 10.0 for i in range(len(queries))]
    )
    assert agreement > 0.95


def test_query_returns_requested_k_and_valid_labels():
    rng = np.random.default_rng(4)
    vectors = rng.normal(size=(200, 6)).astype(np.float32)
    idx = build_index(vectors)
    labels, distances = query_top_k(idx, vectors[:5], k=10)
    assert labels.shape == (5, 10)
    assert distances.shape == (5, 10)
    assert labels.min() >= 0 and labels.max() < 200


def test_brute_force_exclude_self_never_returns_the_query_itself():
    rng = np.random.default_rng(5)
    vectors = rng.normal(size=(100, 6)).astype(np.float32)
    top_k = brute_force_top_k(vectors, vectors, k=10, exclude_self=True)
    for i in range(100):
        assert i not in top_k[i]
