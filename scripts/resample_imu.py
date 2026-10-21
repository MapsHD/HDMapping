#!/usr/bin/env python3
"""Resample Mandeye IMU CSV files to a lower rate by averaging.

Every sample inside a 1/rate window is averaged (a box filter followed by
decimation), so vibration above the new Nyquist rate is attenuated instead of
aliased. Each output row is stamped with the mean time of its samples.

Handles the Mandeye 'imu****.csv' files (columns: timestamp gyroX gyroY gyroZ
accX accY accZ imuId timestampUnix) and the external IMU files in
'MURATA_IMU/' (same columns without imuId). Files of one series are treated
as a single stream, so windows are not cut at file boundaries.

Usage:
    resample_imu.py <scan_folder> [-o <output_folder>] [--rate 200]
    resample_imu.py <file.csv> [<file.csv> ...] -o <output_folder> [--rate 200]

For a scan folder the output is a copy of the raw scan (top-level files and IMU
subfolders; other files hard-linked when possible, copied otherwise) with the
IMU files resampled; the input is never modified.
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

TIME_COLUMNS = ("timestamp", "timestampUnix")
ID_COLUMN = "imuId"


def read_csv(path):
    with open(path) as f:
        header = f.readline().split()
        rows = [line.split() for line in f if line.strip()]
    return header, rows


def resample_series(files, rate_hz):
    """Resample a list of CSV files forming one stream.

    Returns {path: (header, rows)} with each output row assigned to the file
    that held the first sample of its window."""
    period_ns = int(round(1e9 / rate_hz))
    header = None
    samples = []  # (timestamp_ns, file_index, row)
    for i, path in enumerate(files):
        h, rows = read_csv(path)
        if header is None:
            header = h
        elif h != header:
            sys.exit(f"error: '{path}' has columns {h}, expected {header}")
        samples.extend((int(r[0]), i, r) for r in rows)

    out = {path: (header or [], []) for path in files}
    if not samples:
        return out

    ts_cols = [header.index(c) for c in TIME_COLUMNS if c in header]
    id_col = header.index(ID_COLUMN) if ID_COLUMN in header else None
    value_cols = [c for c in range(len(header)) if c not in ts_cols and c != id_col]

    # Group by IMU id (several IMUs may share one file), then by time window.
    groups = {}
    for s in samples:
        imu_id = s[2][id_col] if id_col is not None else None
        groups.setdefault(imu_id, []).append(s)

    for group in groups.values():
        group.sort(key=lambda s: s[0])
        start = 0
        while start < len(group):
            window = group[start][0] // period_ns
            end = start
            while end < len(group) and group[end][0] // period_ns == window:
                end += 1
            chunk = group[start:end]
            n = len(chunk)
            row = list(chunk[0][2])
            for c in ts_cols:
                row[c] = str(sum(int(s[2][c]) for s in chunk) // n)
            for c in value_cols:
                row[c] = f"{sum(float(s[2][c]) for s in chunk) / n:.9g}"
            out[files[chunk[0][1]]][1].append(row)
            start = end

    for path in files:
        out[path][1].sort(key=lambda r: int(r[0]))
    return out


def input_rate(files):
    """Sample rate per IMU, in Hz."""
    stamps = []
    ids = set()
    for path in files:
        header, rows = read_csv(path)
        id_col = header.index(ID_COLUMN) if ID_COLUMN in header else None
        stamps.extend(int(r[0]) for r in rows)
        ids.update(r[id_col] for r in rows if id_col is not None)
    if len(stamps) < 2 or stamps[0] == stamps[-1]:
        return None
    stamps.sort()
    return (len(stamps) - 1) / max(len(ids), 1) / ((stamps[-1] - stamps[0]) / 1e9)


def write_csv(path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(" ".join(header) + "\n")
        for r in rows:
            f.write(" ".join(r) + "\n")


def link_or_copy(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def find_series(folder):
    """IMU series in a scan folder: the 'imu****.csv' files and each
    subfolder's CSV files (e.g. MURATA_IMU)."""
    series = []
    mandeye = sorted(p for p in folder.glob("imu*.csv") if p.is_file())
    if mandeye:
        series.append(mandeye)
    for sub in sorted(p for p in folder.iterdir() if p.is_dir()):
        csvs = sorted(p for p in sub.glob("*.csv") if p.is_file())
        if csvs and read_csv(csvs[0])[0][:1] == ["timestamp"]:
            series.append(csvs)
    return series


def process(series, rate_hz, out_path_for):
    for files in series:
        rate_in = input_rate(files)
        name = files[0].parent.name if len(files) == 1 else f"{files[0].parent.name}/{files[0].name}.."
        if rate_in is None:
            print(f"{name}: no data, copied as is")
            for f in files:
                link_or_copy(f, out_path_for(f))
            continue
        if rate_in <= rate_hz * 1.05:
            print(f"{name}: {rate_in:.0f} Hz is not above {rate_hz} Hz, copied as is")
            for f in files:
                link_or_copy(f, out_path_for(f))
            continue
        result = resample_series(files, rate_hz)
        n_in = n_out = 0
        for f in files:
            header, rows = result[f]
            n_in += len(read_csv(f)[1])
            n_out += len(rows)
            write_csv(out_path_for(f), header, rows)
        print(f"{name}: {len(files)} file(s), {rate_in:.0f} Hz -> {rate_hz} Hz, {n_in} -> {n_out} rows")


def main():
    parser = argparse.ArgumentParser(description="Resample Mandeye IMU CSV files to a lower rate by averaging.")
    parser.add_argument("inputs", nargs="+", type=Path, help="scan folder, or IMU CSV files")
    parser.add_argument("-o", "--output", type=Path, help="output folder (default for a scan folder: <folder>_imu<rate>hz)")
    parser.add_argument("--rate", type=float, default=200.0, help="output rate in Hz (default: 200)")
    args = parser.parse_args()

    if args.rate <= 0:
        sys.exit("error: --rate must be positive")
    rate_hz = int(args.rate) if args.rate.is_integer() else args.rate

    if len(args.inputs) == 1 and args.inputs[0].is_dir():
        src = args.inputs[0].resolve()
        dst = (args.output or src.with_name(f"{src.name}_imu{rate_hz}hz")).resolve()
        if dst == src or src in dst.parents:
            sys.exit("error: output folder must not be the scan folder or inside it")
        if dst.exists() and any(dst.iterdir()):
            sys.exit(f"error: output folder '{dst}' exists and is not empty")
        series = find_series(src)
        if not series:
            sys.exit(f"error: no IMU CSV files found in '{src}'")
        # Copy the raw scan only: top-level files and the IMU subfolders. Other
        # subfolders (lio_result_*, preview, cache, ...) are results of earlier runs.
        imu_files = {f for s in series for f in s}
        imu_dirs = {f.parent for f in imu_files} - {src}
        for f in sorted(src.iterdir()):
            if f.is_file() and f not in imu_files:
                link_or_copy(f, dst / f.name)
            elif f.is_dir() and f not in imu_dirs:
                print(f"skipped folder: {f.name}")
        for d in imu_dirs:
            for f in d.iterdir():
                if f.is_file() and f not in imu_files:
                    link_or_copy(f, dst / f.relative_to(src))
        process(series, rate_hz, lambda f: dst / f.relative_to(src))
        print(f"output: {dst}")
    else:
        if not args.output:
            sys.exit("error: -o/--output is required when resampling individual files")
        files = sorted(p.resolve() for p in args.inputs)
        for f in files:
            if not f.is_file():
                sys.exit(f"error: '{f}' is not a file")
        dst = args.output.resolve()
        if any(f.parent == dst for f in files):
            sys.exit("error: output folder must differ from the input files' folder")
        process([files], rate_hz, lambda f: dst / f.name)
        print(f"output: {dst}")


if __name__ == "__main__":
    main()
