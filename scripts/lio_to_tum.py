#!/usr/bin/env python3
"""Convert a lidar_odometry_step_1 result folder to a TUM trajectory.

Each pose is the chunk pose from 'session_ini_poses.mri' times the local pose
from 'trajectory_lio_<N>.csv'. The .mri is read rather than 'session_poses.mrp'
because steps 2 and 3 overwrite the .mrp; the .mri is what step 1 wrote.

Output lines are 'timestamp tx ty tz qx qy qz qw', sorted by time, with the
timestamp in seconds taken from the CSV's 'timestamp_nanoseconds' column.

Usage:
    lio_to_tum.py <lio_result_folder> <output.tum>
"""

import argparse
import math
import re
import sys
from pathlib import Path


def read_poses(path):
    """Return {chunk index: 4x4 pose} from a .mri/.mrp file."""
    tokens = Path(path).read_text().split()
    count = int(tokens[0])
    poses = {}
    pos = 1
    for _ in range(count):
        index = int(re.search(r"(\d+)", Path(tokens[pos]).stem).group(1))
        values = [float(v) for v in tokens[pos + 1 : pos + 17]]
        poses[index] = [values[r * 4 : r * 4 + 4] for r in range(4)]
        pos += 17
    return poses


def multiply(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def quaternion(m):
    """Rotation part of a 4x4 matrix as (qx, qy, qz, qw)."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda d: m[d][d])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i][i] - m[j][j] - m[k][k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j][i] + m[i][j]) / s
    q[k] = (m[k][i] + m[i][k]) / s
    q[3] = (m[k][j] - m[j][k]) / s
    return tuple(q)


def convert(folder):
    """Return [(timestamp_ns, 4x4 global pose)] sorted by time."""
    folder = Path(folder)
    chunks = read_poses(folder / "session_ini_poses.mri")
    rows = []
    for index in sorted(chunks):
        csv_path = folder / f"trajectory_lio_{index}.csv"
        with open(csv_path) as f:
            next(f)  # header
            for line in f:
                v = line.split()
                if len(v) < 13:
                    continue
                local = [[float(x) for x in v[1 + r * 4 : 5 + r * 4]] for r in range(3)] + [[0.0, 0.0, 0.0, 1.0]]
                rows.append((int(v[0]), multiply(chunks[index], local)))
    rows.sort(key=lambda row: row[0])
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", help="lio_result_N folder written by lidar_odometry_step_1")
    parser.add_argument("output", help="TUM file to write")
    args = parser.parse_args()

    rows = convert(args.folder)
    if not rows:
        print(f"No poses found in '{args.folder}'", file=sys.stderr)
        return 1
    with open(args.output, "w", newline="\n") as out:  # same bytes on every platform
        for ns, m in rows:
            qx, qy, qz, qw = quaternion(m)
            out.write(
                f"{ns // 10**9}.{ns % 10**9:09d} {m[0][3]:.6f} {m[1][3]:.6f} {m[2][3]:.6f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
    print(f"{len(rows)} poses written to '{args.output}'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
