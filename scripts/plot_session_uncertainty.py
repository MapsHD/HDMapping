#!/usr/bin/env python3
"""Plot the trajectory and pose uncertainty written by session_uncertainty_calculation.

Reads the CSV of any --method (point_to_point, point_to_plane, monte_carlo) and shows:
  1. top view (x, y) of the trajectory coloured by the position sigma, with covariance ellipses;
  2. sigma_px, sigma_py, sigma_pz [mm] and sigma_om, sigma_fi, sigma_ka [deg] over time, plus the method's extra
     columns (sigma0 and correspondences, or converged Monte Carlo runs).

px, py, pz are the node's global position, so the x-y block of the covariance is drawn as is in the top view.
Nodes whose uncertainty was not computed (inf, nan) are drawn as grey crosses.

Needs numpy and matplotlib (pip install numpy matplotlib).

  python3 scripts/plot_session_uncertainty.py uncertainty.csv
  python3 scripts/plot_session_uncertainty.py uncertainty.csv --ellipse-scale 100 --output uncertainty.png
"""

import argparse
import csv
import math
import os
import sys

PARAMETERS = ["px", "py", "pz", "om", "fi", "ka"]


def read_csv(path):
    """Returns a dict column name -> list of floats."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        columns = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in reader.fieldnames:
                try:
                    columns[name].append(float(row[name]))
                except (TypeError, ValueError):
                    columns[name].append(math.nan)
    return columns


def covariance_matrices(columns, np):
    """6x6 covariance per node from the cov_<row>_<col> upper triangle; None when the CSV has no covariance."""
    if "cov_px_px" not in columns:
        return None
    n = len(columns["x"])
    covariance = np.full((n, 6, 6), np.nan)
    for i, a in enumerate(PARAMETERS):
        for j in range(i, 6):
            values = np.asarray(columns[f"cov_{a}_{PARAMETERS[j]}"])
            covariance[:, i, j] = values
            covariance[:, j, i] = values
    return covariance


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", help="output of session_uncertainty_calculation")
    parser.add_argument(
        "--ellipse-scale",
        type=float,
        default=None,
        help="ellipse half-axes in sigmas (default: chosen so the median ellipse is ~1%% of the trajectory extent)",
    )
    parser.add_argument("--ellipse-step", type=int, default=0, help="draw every n-th ellipse (default: ~200 ellipses)")
    parser.add_argument("--log", action="store_true", help="logarithmic colour scale and sigma axes")
    parser.add_argument("--output", help="save the figures to this file (a _time suffix for the second) instead of showing them")
    args = parser.parse_args()

    try:
        import numpy as np
        import matplotlib

        if args.output:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import EllipseCollection
        from matplotlib.colors import LogNorm, Normalize
    except ImportError as error:
        sys.exit(f"{error}: install with 'pip install numpy matplotlib'")

    columns = read_csv(args.csv)
    if not columns.get("x"):
        sys.exit(f"no nodes in {args.csv}")

    x = np.asarray(columns["x"])
    y = np.asarray(columns["y"])
    t = np.asarray(columns["timestamp_lidar"])
    t = t - t[0]
    if t[-1] > 1e6:  # nanoseconds
        t = t * 1e-9
    sigma = {p: np.asarray(columns[f"sigma_{p}"]) for p in PARAMETERS}
    sigma_mm = {p: sigma[p] * 1000.0 for p in PARAMETERS[:3]}
    sigma_deg = {p: np.degrees(sigma[p]) for p in PARAMETERS[3:]}
    position_sigma_mm = np.sqrt(sigma_mm["px"] ** 2 + sigma_mm["py"] ** 2 + sigma_mm["pz"] ** 2)
    valid = np.isfinite(position_sigma_mm)
    covariance = covariance_matrices(columns, np)
    is_monte_carlo = "number_of_converged_runs" in columns

    print(f"{len(x)} nodes, {valid.sum()} with uncertainty")
    if valid.any():
        for p in PARAMETERS[:3]:
            v = sigma_mm[p][valid]
            print(f"  sigma_{p} [mm]: median {np.median(v):.4f}, 95% {np.percentile(v, 95):.4f}, max {v.max():.4f}")
        for p in PARAMETERS[3:]:
            v = sigma_deg[p][np.isfinite(sigma_deg[p])]
            if v.size:
                print(f"  sigma_{p} [deg]: median {np.median(v):.5f}, 95% {np.percentile(v, 95):.5f}, max {v.max():.5f}")

    # --- top view
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(x, y, color="lightgray", linewidth=1, zorder=1)
    if (~valid).any():
        ax.scatter(x[~valid], y[~valid], marker="x", color="gray", s=20, label="not computed", zorder=2)
    if valid.any():
        values = position_sigma_mm[valid]
        positive = values[values > 0]
        norm = (
            LogNorm(vmin=positive.min(), vmax=positive.max())
            if args.log and positive.size
            else Normalize(vmin=values.min(), vmax=np.percentile(values, 99))
        )
        points = ax.scatter(x[valid], y[valid], c=values, cmap="viridis", norm=norm, s=12, zorder=3)
        fig.colorbar(points, ax=ax, label="position sigma sqrt(px² + py² + pz²) [mm]")

    if covariance is not None:
        xy = covariance[:, :2, :2]
        ok = np.isfinite(xy).all(axis=(1, 2))
        indices = np.flatnonzero(ok)
        step = args.ellipse_step if args.ellipse_step > 0 else max(1, len(indices) // 200)
        indices = indices[::step]
        if indices.size:
            eigenvalues, eigenvectors = np.linalg.eigh(xy[indices])
            half_axes = np.sqrt(np.clip(eigenvalues, 0.0, None))  # ascending: minor, major
            scale = args.ellipse_scale
            if scale is None:
                extent = max(np.ptp(x), np.ptp(y), 1e-9)
                median_major = np.median(half_axes[:, 1])
                scale = 0.01 * extent / median_major if median_major > 0 else 1.0
            angles = np.degrees(np.arctan2(eigenvectors[:, 1, 1], eigenvectors[:, 0, 1]))
            ax.add_collection(
                EllipseCollection(
                    widths=2 * scale * half_axes[:, 1],
                    heights=2 * scale * half_axes[:, 0],
                    angles=angles,
                    units="xy",
                    offsets=np.column_stack([x[indices], y[indices]]),
                    offset_transform=ax.transData,
                    facecolors="none",
                    edgecolors="crimson",
                    linewidths=0.8,
                    zorder=4,
                )
            )
            ax.plot([], [], color="crimson", label=f"x-y covariance, ±{scale:.3g} sigma (every {step}. node)")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    method = "Monte Carlo ICP" if is_monte_carlo else "ICP normal matrix"
    ax.set_title(f"Trajectory and position uncertainty ({method}): {os.path.basename(args.csv)}")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="best")
    fig.tight_layout()

    # --- sigmas over time
    fig_time, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for p in PARAMETERS[:3]:
        axes[0].plot(t, sigma_mm[p], label=f"sigma_{p}", linewidth=1)
    axes[0].set_ylabel("[mm]")
    for p in PARAMETERS[3:]:
        axes[1].plot(t, sigma_deg[p], label=f"sigma_{p}", linewidth=1)
    axes[1].set_ylabel("[deg]")
    for a in axes[:2]:
        a.legend(loc="upper right")
        a.grid(True, alpha=0.3)
        if args.log:
            a.set_yscale("log")
    if is_monte_carlo:
        axes[2].plot(t, columns["number_of_converged_runs"], label="converged runs", color="tab:green")
        axes[2].plot(t, columns["number_of_runs"], label="runs", color="gray", linestyle="--")
        axes[2].set_ylabel("ICP runs")
        axes[2].legend(loc="upper right")
    elif "sigma0(RMS)" in columns:
        axes[2].plot(t, np.asarray(columns["sigma0(RMS)"]) * 1000.0, label="sigma0 (RMS residual)", color="tab:red")
        axes[2].set_ylabel("sigma0 [mm]", color="tab:red")
        twin = axes[2].twinx()
        twin.plot(t, columns["number_of_correspondences"], color="tab:blue", linewidth=1)
        twin.set_ylabel("correspondences", color="tab:blue")
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlabel("time [s]")
    axes[0].set_title(f"Pose sigmas over time ({method})")
    fig_time.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
        stem, dot, extension = args.output.rpartition(".")
        time_output = f"{stem}_time.{extension}" if dot else f"{args.output}_time"
        fig_time.savefig(time_output, dpi=150)
        print(f"saved {args.output} and {time_output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
