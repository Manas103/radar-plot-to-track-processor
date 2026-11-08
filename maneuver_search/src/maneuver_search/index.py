"""The approximate-nearest-neighbor index over standardized maneuver
feature vectors (hnswlib, L2 space), plus an independent brute-force exact
kNN function used only as a reference oracle in tests, never as the
production retrieval path.
"""
from __future__ import annotations

from dataclasses import dataclass

import hnswlib
import numpy as np


@dataclass
class ManeuverIndex:
    index: hnswlib.Index
    n_items: int
    dim: int


def build_index(
    vectors: np.ndarray, ef_construction: int = 200, m: int = 16, ef_search: int = 64
) -> ManeuverIndex:
    n_items, dim = vectors.shape
    idx = hnswlib.Index(space="l2", dim=dim)
    idx.init_index(max_elements=n_items, ef_construction=ef_construction, M=m)
    idx.add_items(vectors, np.arange(n_items))
    idx.set_ef(ef_search)
    return ManeuverIndex(index=idx, n_items=n_items, dim=dim)


def query_top_k(index: ManeuverIndex, query_vectors: np.ndarray, k: int = 10) -> tuple[np.ndarray, np.ndarray]:
    """Returns (labels, distances), each shape (n_queries, k)."""
    labels, distances = index.index.knn_query(query_vectors, k=k)
    return labels, distances


def brute_force_top_k(vectors: np.ndarray, query_vectors: np.ndarray, k: int, exclude_self: bool = False) -> np.ndarray:
    """Independent, from-scratch exact nearest-neighbor computation
    (plain NumPy, no approximate structure), used as a reference oracle
    for hnswlib's approximate results. O(n_queries * n_items), fine for
    the small correctness-check samples this is run on, never for the
    full 220K-item corpus.
    """
    out = np.empty((query_vectors.shape[0], k), dtype=np.int64)
    for i, q in enumerate(query_vectors):
        d2 = np.sum((vectors - q) ** 2, axis=1)
        if exclude_self:
            # the query is item i itself; exclude distance-0 self-match
            order = np.argsort(d2)
            order = order[order != i][:k]
        else:
            order = np.argsort(d2)[:k]
        out[i] = order
    return out
