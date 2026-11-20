#!/usr/bin/env python3
"""Generates 120 seeded three-feed fused scenarios (via `fusion_cli
export-fused-scenario`, which genuinely runs the real radar + ADS-B
encode/decode + AIS simulators and merges them), runs each through the
existing, completely unmodified independent Python reference tracker and
the existing, completely unmodified `radar_cli track`, and diffs the two
on track count and track identity per scan, exactly the way
run_oracle_comparison.py already does for the single-source tracker. No
change to reference_tracker.py or to radar_cli was needed: a fused scan is
just a RADAR_SCAN_LOG scan like any other, by the time it reaches either
side.

Usage:
    python3 run_fusion_oracle_comparison.py --fusion-cli /path/to/fusion_cli \
        --radar-cli /path/to/radar_cli --num-scenarios 120 \
        --scenario-dir /tmp/fusion_scenarios --report docs/fusion_oracle_comparison_output.txt
"""
from __future__ import annotations

import argparse
import os
import random
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
from reference_tracker import run_over_log  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fusion-cli", required=True)
    ap.add_argument("--radar-cli", required=True)
    ap.add_argument("--num-scenarios", type=int, default=120)
    ap.add_argument("--scenario-dir", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args()

    os.makedirs(args.scenario_dir, exist_ok=True)
    agree = 0
    disagreements = []
    source_mix = []

    for i in range(args.num_scenarios):
        seed = 2000 + i
        rng = random.Random(seed)
        num_total = rng.randint(3, 8)
        num_scans = rng.randint(15, 25)
        counts = [0, 0, 0]
        for _ in range(num_total):
            counts[rng.randint(0, 2)] += 1
        num_radar, num_adsb, num_ais = counts
        source_mix.append((num_radar, num_adsb, num_ais))

        scenario_path = os.path.join(args.scenario_dir, f"fusion_scenario_{i:03d}.log")
        export = subprocess.run(
            [
                args.fusion_cli,
                "export-fused-scenario",
                scenario_path,
                str(num_scans),
                str(num_radar),
                str(num_adsb),
                str(num_ais),
                str(seed),
            ],
            capture_output=True,
            text=True,
        )
        if export.returncode != 0:
            disagreements.append((i, "fusion_cli export-fused-scenario failed", export.stderr.strip()))
            continue

        py_lines = run_over_log(scenario_path)

        cpp_out_path = os.path.join(args.scenario_dir, f"fusion_scenario_{i:03d}.cpp_tracks.log")
        result = subprocess.run(
            [args.radar_cli, "track", scenario_path, cpp_out_path], capture_output=True, text=True
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
    radar_only = sum(1 for r, a, s in source_mix if a == 0 and s == 0)
    all_three = sum(1 for r, a, s in source_mix if r > 0 and a > 0 and s > 0)
    with open(args.report, "w") as f:
        f.write(
            f"FUSION_ORACLE_COMPARISON: {agree} of {total} fused scenarios agreed with the independent "
            f"Python reference tracker on track count and identity per scan\n"
        )
        f.write(f"scenario total target counts: 3-8, scan lengths: 15-25, seeds: 2000..{2000+total-1}\n")
        f.write(f"scenarios using all three sources: {all_three}/{total}\n")
        f.write(f"scenarios using radar only: {radar_only}/{total}\n")
        for idx, reason, detail in disagreements:
            f.write(f"DISAGREEMENT fusion_scenario_{idx:03d}: {reason} :: {detail}\n")

    print(f"{agree}/{total} fused scenarios agreed")
    return 0 if agree == total else 1


if __name__ == "__main__":
    sys.exit(main())
