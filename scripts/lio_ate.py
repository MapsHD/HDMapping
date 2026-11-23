#!/usr/bin/env python3
"""Absolute trajectory error (ATE) of a TUM trajectory against a golden one.

Poses are paired in order and must have the same timestamps (within
--max-dt-s). They are compared without any alignment: both trajectories come
from lidar_odometry_step_1 on the same data and start in the same frame, so an
alignment would only hide drift.

Exit codes: 0 when the position RMSE is within the limit, 1 when it is not,
2 when the two files do not have the same poses or timestamps.

Usage:
    lio_ate.py <golden.tum> <candidate.tum> [--max-ate-m 0.001]
"""

import argparse
import math
import sys


def load(path):
    """Return ([timestamp strings], [(x, y, z)]) from a TUM file."""
    times, positions = [], []
    with open(path) as f:
        for line in f:
            v = line.split()
            if not v or v[0].startswith("#"):
                continue
            times.append(v[0])
            positions.append((float(v[1]), float(v[2]), float(v[3])))
    return times, positions


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("golden", help="reference TUM trajectory")
    parser.add_argument("candidate", help="TUM trajectory to check")
    parser.add_argument("--max-ate-m", type=float, default=0.001, help="largest accepted position RMSE in metres (default 0.001)")
    parser.add_argument(
        "--max-dt-s", type=float, default=1e-5, help="largest accepted timestamp difference between paired poses (default 1e-5)"
    )
    args = parser.parse_args()

    golden_times, golden = load(args.golden)
    candidate_times, candidate = load(args.candidate)
    if len(golden_times) != len(candidate_times):
        print(f"Pose counts differ: golden has {len(golden_times)} poses, candidate {len(candidate_times)}")
        return 2
    # Timestamps are rounded from doubles, so platforms can differ in the last digits (~0.3 us at 1.8e9 s).
    worst = max((abs(float(g) - float(c)) for g, c in zip(golden_times, candidate_times)), default=0.0)
    if worst > args.max_dt_s:
        print(f"Timestamps differ by up to {worst:.9f} s (limit {args.max_dt_s} s)")
        return 2
    if not golden:
        print("No poses to compare")
        return 2

    errors = [math.dist(g, c) for g, c in zip(golden, candidate)]
    ate = math.sqrt(sum(e * e for e in errors) / len(errors))
    print(
        f"poses {len(errors)}  ATE RMSE {ate * 1000:.2f} mm  max {max(errors) * 1000:.2f} mm  "
        f"final {errors[-1] * 1000:.2f} mm  (limit {args.max_ate_m * 1000:.1f} mm)"
    )
    if ate > args.max_ate_m:
        print("FAIL: trajectory differs from the golden one")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
