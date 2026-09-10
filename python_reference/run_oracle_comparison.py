#!/usr/bin/env python3
"""Generates seeded scenarios, runs each through the independent Python
reference tracker and through the real C++ pipeline (`radar_cli track`),
and diffs the two on track count and track identity per scan.

Usage:
    python3 run_oracle_comparison.py --cli /path/to/radar_cli \
        --num-scenarios 120 --scenario-dir /tmp/scenarios --report docs/oracle_comparison_output.txt
"""

from __future__ import annotations

import argparse
import os
import random
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
from reference_tracker import run_over_log, canonical_line  # noqa: E402


def write_scenario(path, seed, num_targets, num_scans):
    rng = random.Random(seed)
    area = 20000.0
    targets = [
        {
            "x": rng.uniform(0, area),
            "y": rng.uniform(0, area),
            "vx": rng.uniform(-15, 15),
            "vy": rng.uniform(-15, 15),
        }
        for _ in range(num_targets)
    ]
    with open(path, "w") as f:
        f.write("RADAR_SCAN_LOG v1\n")
        for scan_id in range(1, num_scans + 1):
            dets = []
            for t in targets:
                t["vx"] += rng.gauss(0, 0.3)
                t["vy"] += rng.gauss(0, 0.3)
                t["x"] = min(max(t["x"] + t["vx"], 0.0), area)
                t["y"] = min(max(t["y"] + t["vy"], 0.0), area)
                if rng.random() < 0.97:  # Pd
                    dets.append((t["x"] + rng.gauss(0, 50.0), t["y"] + rng.gauss(0, 50.0)))
            # occasional clutter false alarm
            if rng.random() < 0.3:
                dets.append((rng.uniform(0, area), rng.uniform(0, area)))
            f.write(f"SCAN {scan_id} {float(scan_id)} {len(dets)}\n")
            for x, y in dets:
                f.write(f"{x:.6f} {y:.6f}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", required=True)
    ap.add_argument("--num-scenarios", type=int, default=120)
    ap.add_argument("--scenario-dir", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args()

    os.makedirs(args.scenario_dir, exist_ok=True)
    agree = 0
    disagreements = []

    for i in range(args.num_scenarios):
        seed = 1000 + i
        rng = random.Random(seed)
        num_targets = rng.randint(3, 8)
        num_scans = rng.randint(15, 25)
        scenario_path = os.path.join(args.scenario_dir, f"scenario_{i:03d}.log")
        write_scenario(scenario_path, seed, num_targets, num_scans)

        py_lines = run_over_log(scenario_path)

        cpp_out_path = os.path.join(args.scenario_dir, f"scenario_{i:03d}.cpp_tracks.log")
        result = subprocess.run(
            [args.cli, "track", scenario_path, cpp_out_path], capture_output=True, text=True
        )
        if result.returncode != 0:
            disagreements.append((i, "radar_cli track failed", result.stderr.strip()))
            continue
        with open(cpp_out_path) as f:
            cpp_lines = f.readlines()

        if py_lines == cpp_lines:
            agree += 1
        else:
            first_diff = next(
                (k for k in range(min(len(py_lines), len(cpp_lines))) if py_lines[k] != cpp_lines[k]),
                min(len(py_lines), len(cpp_lines)),
            )
            disagreements.append(
                (
                    i,
                    f"first differing scan line {first_diff}",
                    f"py={py_lines[first_diff] if first_diff < len(py_lines) else '<eof>'!r} "
                    f"cpp={cpp_lines[first_diff] if first_diff < len(cpp_lines) else '<eof>'!r}",
                )
            )

    total = args.num_scenarios
    with open(args.report, "w") as f:
        f.write(f"ORACLE_COMPARISON: {agree} of {total} scenarios agreed with the independent "
                f"Python reference tracker on track count and identity per scan\n")
        f.write(f"scenario target counts: 3-8, scan lengths: 15-25, seeds: 1000..{1000+total-1}\n")
        for idx, reason, detail in disagreements:
            f.write(f"DISAGREEMENT scenario_{idx:03d}: {reason} :: {detail}\n")

    print(f"{agree}/{total} scenarios agreed")
    return 0 if agree == total else 1


if __name__ == "__main__":
    sys.exit(main())
