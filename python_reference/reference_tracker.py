#!/usr/bin/env python3
"""Independent Python reference tracker.

This is a separate implementation of the same specification the C++ side
implements (chi-square-gated nearest-neighbor association, a constant-
velocity Kalman filter, M-of-N confirm/delete), written from scratch in
pure Python with no shared code and no code generation from the C++ side.
Where it must match the C++ side, it matches by specification (the same
chi-square threshold is a published statistical constant, not shared
code; the same M-of-N window sizes are a chosen tracker parameter, not an
algorithmic shortcut) -- not by translation.

The assignment step deliberately does NOT use the Hungarian algorithm the
C++ side uses. For the small scenario sizes this oracle is run over (a
handful of targets), it instead brute-forces every possible one-to-one
matching within a gated group and keeps the minimum-cost one. That is a
different algorithm arriving at the same exact optimum, which is a
stronger independence guarantee than re-implementing Hungarian a second
time would be.
"""

from __future__ import annotations

import itertools
import math
import sys
from dataclasses import dataclass, field

CHI_SQUARE_GATE_99_2DOF = 9.21034
CONFIRM_WINDOW_N = 5
CONFIRM_HITS_M = 3
DELETE_CONSECUTIVE_MISSES = 3
PROCESS_NOISE_DENSITY = 4.0
MEASUREMENT_SIGMA_M = 50.0
DT_SECONDS = 1.0


def mat4_identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def mat_mul(a, b):
    rows_a, cols_a = len(a), len(a[0])
    cols_b = len(b[0])
    out = [[0.0] * cols_b for _ in range(rows_a)]
    for i in range(rows_a):
        for k in range(cols_a):
            if a[i][k] == 0.0:
                continue
            for j in range(cols_b):
                out[i][j] += a[i][k] * b[k][j]
    return out


def mat_transpose(a):
    return [[a[i][j] for i in range(len(a))] for j in range(len(a[0]))]


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(len(v))) for i in range(len(a))]


def mat2_inv(s):
    det = s[0][0] * s[1][1] - s[0][1] * s[1][0]
    inv_det = 0.0 if det == 0.0 else 1.0 / det
    return [[s[1][1] * inv_det, -s[0][1] * inv_det], [-s[1][0] * inv_det, s[0][0] * inv_det]]


class KalmanCV:
    """Constant-velocity Kalman filter, state = [x, y, vx, vy].

    Same discrete white-noise-acceleration process model as the C++ side
    (it is the standard nearly-constant-velocity model, not an
    implementation detail copied across languages).
    """

    def __init__(self, x, y, vx, vy):
        self.x = [x, y, vx, vy]
        pos_var = MEASUREMENT_SIGMA_M ** 2
        vel_var = 400.0 ** 2
        self.p = [[0.0] * 4 for _ in range(4)]
        self.p[0][0] = pos_var
        self.p[1][1] = pos_var
        self.p[2][2] = vel_var
        self.p[3][3] = vel_var

    def predict(self, dt):
        f = mat4_identity()
        f[0][2] = dt
        f[1][3] = dt
        self.x = mat_vec(f, self.x)

        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q = [[0.0] * 4 for _ in range(4)]
        q[0][0] = PROCESS_NOISE_DENSITY * dt4 / 4.0
        q[0][2] = q[2][0] = PROCESS_NOISE_DENSITY * dt3 / 2.0
        q[2][2] = PROCESS_NOISE_DENSITY * dt2
        q[1][1] = PROCESS_NOISE_DENSITY * dt4 / 4.0
        q[1][3] = q[3][1] = PROCESS_NOISE_DENSITY * dt3 / 2.0
        q[3][3] = PROCESS_NOISE_DENSITY * dt2

        fp = mat_mul(f, self.p)
        fpft = mat_mul(fp, mat_transpose(f))
        self.p = mat_add(fpft, q)

    def innovation(self, det):
        h = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        hx = [self.x[0], self.x[1]]
        y = [det[0] - hx[0], det[1] - hx[1]]
        hp = mat_mul(h, self.p)
        ht = mat_transpose(h)
        hpht = mat_mul(hp, ht)
        r = [[MEASUREMENT_SIGMA_M ** 2, 0.0], [0.0, MEASUREMENT_SIGMA_M ** 2]]
        s = mat_add(hpht, r)
        return y, s

    def update(self, y, s):
        h = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        ht = mat_transpose(h)
        pht = mat_mul(self.p, ht)
        s_inv = mat2_inv(s)
        k = mat_mul(pht, s_inv)
        correction = mat_vec(k, y)
        self.x = [self.x[i] + correction[i] for i in range(4)]
        kh = mat_mul(k, h)
        i_minus_kh = mat4_identity()
        for i in range(4):
            for j in range(4):
                i_minus_kh[i][j] -= kh[i][j]
        self.p = mat_mul(i_minus_kh, self.p)


def mahalanobis_sq(y, s):
    s_inv = mat2_inv(s)
    sy = mat_vec(s_inv, y)
    return y[0] * sy[0] + y[1] * sy[1]


@dataclass
class Track:
    id: int
    status: str = "tentative"  # tentative | confirmed | deleted
    kf: KalmanCV = None
    hit_history: list = field(default_factory=list)
    consecutive_misses: int = 0


def solve_component_brute_force(track_ids, det_ids, cost):
    """Optimal one-to-one matching within one gated group, found by trying
    every permutation of the smaller side against the larger side. This is
    exponential in group size, which is fine here: gated groups in these
    small scenarios are a handful of targets at most.
    """
    if not track_ids or not det_ids:
        return []
    if len(track_ids) <= len(det_ids):
        rows, cols = track_ids, det_ids
        row_is_track = True
    else:
        rows, cols = det_ids, track_ids
        row_is_track = False

    best_cost = math.inf
    best_perm = None
    for perm in itertools.permutations(cols, len(rows)):
        total = 0.0
        ok = True
        for r, c in zip(rows, perm):
            cst = cost.get((r, c) if row_is_track else (c, r))
            if cst is None:
                ok = False
                break
            total += cst
        if ok and total < best_cost:
            best_cost = total
            best_perm = perm
    if best_perm is None:
        return []
    result = []
    for r, c in zip(rows, best_perm):
        if row_is_track:
            result.append((r, c))
        else:
            result.append((c, r))
    return result


def connected_components(edges, track_ids, det_ids):
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for t, d, _ in edges:
        union(("t", t), ("d", d))

    groups = {}
    for t, d, cost in edges:
        root = find(("t", t))
        groups.setdefault(root, {"tracks": set(), "dets": set(), "cost": {}})
        groups[root]["tracks"].add(t)
        groups[root]["dets"].add(d)
        groups[root]["cost"][(t, d)] = cost
    return list(groups.values())


class TrackManager:
    def __init__(self):
        self.tracks: list[Track] = []
        self.next_id = 1

    def process_scan(self, scan_id, detections):
        for t in self.tracks:
            if t.status != "deleted":
                t.kf.predict(DT_SECONDS)

        edges = []
        for ti, t in enumerate(self.tracks):
            if t.status == "deleted":
                continue
            for di, det in enumerate(detections):
                y, s = t.kf.innovation(det)
                m2 = mahalanobis_sq(y, s)
                if m2 < CHI_SQUARE_GATE_99_2DOF:
                    edges.append((ti, di, m2))

        groups = connected_components(edges, list(range(len(self.tracks))), list(range(len(detections))))
        assignments = []
        for g in groups:
            assignments.extend(solve_component_brute_force(sorted(g["tracks"]), sorted(g["dets"]), g["cost"]))

        track_hit = [False] * len(self.tracks)
        det_used = [False] * len(detections)
        for ti, di in assignments:
            t = self.tracks[ti]
            y, s = t.kf.innovation(detections[di])
            t.kf.update(y, s)
            track_hit[ti] = True
            det_used[di] = True

        for ti, t in enumerate(self.tracks):
            if t.status == "deleted":
                continue
            hit = track_hit[ti]
            t.hit_history.append(hit)
            if len(t.hit_history) > CONFIRM_WINDOW_N:
                t.hit_history.pop(0)
            t.consecutive_misses = 0 if hit else t.consecutive_misses + 1

            if t.status == "tentative":
                hits = sum(t.hit_history)
                if hits >= CONFIRM_HITS_M:
                    t.status = "confirmed"
                elif len(t.hit_history) >= CONFIRM_WINDOW_N and hits < CONFIRM_HITS_M:
                    t.status = "deleted"
            elif t.status == "confirmed":
                if t.consecutive_misses >= DELETE_CONSECUTIVE_MISSES:
                    t.status = "deleted"

        for di, det in enumerate(detections):
            if det_used[di]:
                continue
            nt = Track(id=self.next_id, status="tentative", kf=KalmanCV(det[0], det[1], 0.0, 0.0))
            nt.hit_history.append(True)
            self.next_id += 1
            self.tracks.append(nt)

        self.tracks = [t for t in self.tracks if t.status != "deleted"]

        confirmed_ids = sorted(t.id for t in self.tracks if t.status == "confirmed")
        num_tentative = sum(1 for t in self.tracks if t.status == "tentative")
        return scan_id, confirmed_ids, num_tentative


def read_scan_log(path):
    """Independent parser for the same whitespace-delimited RADAR_SCAN_LOG
    text format the C++ side reads and writes (see replay.hpp)."""
    scans = []
    with open(path, "r") as f:
        tokens = f.read().split()
    i = 0
    if i < len(tokens) and tokens[i] == "RADAR_SCAN_LOG":
        i += 2  # skip the header and its version token
    while i < len(tokens):
        assert tokens[i] == "SCAN", f"expected SCAN, got {tokens[i]!r}"
        scan_id = int(tokens[i + 1])
        i += 3  # SCAN, scan_id, timestamp
        count = int(tokens[i])
        i += 1
        dets = []
        for _ in range(count):
            x, y = float(tokens[i]), float(tokens[i + 1])
            dets.append((x, y))
            i += 2
        scans.append((scan_id, dets))
    return scans


def canonical_line(scan_id, confirmed_ids, num_tentative):
    return f"S {scan_id} C{len(confirmed_ids)} T{num_tentative} IDS:{','.join(str(x) for x in confirmed_ids)}\n"


def run_over_log(path):
    tm = TrackManager()
    lines = []
    for scan_id, dets in read_scan_log(path):
        sid, confirmed_ids, num_tentative = tm.process_scan(scan_id, dets)
        lines.append(canonical_line(sid, confirmed_ids, num_tentative))
    return lines


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: reference_tracker.py <in_scan_log> <out_track_log>", file=sys.stderr)
        sys.exit(2)
    out_lines = run_over_log(sys.argv[1])
    with open(sys.argv[2], "w") as f:
        f.writelines(out_lines)
    print(f"processed {len(out_lines)} scans from {sys.argv[1]}")
