import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import os
import numpy as np
from pathlib import Path


def calculate_weighted_stats(centers, counts):
    """Calculate weighted mean and standard deviation."""
    total_count = np.sum(counts)
    if total_count == 0:
        return 0.0, 0.0

    weighted_mean = np.sum(centers * counts) / total_count
    variance = np.sum(counts * (centers - weighted_mean) ** 2) / total_count
    weighted_std = np.sqrt(variance)

    return weighted_mean, weighted_std


def gaussian_distribution(x, mean, std):
    """Gaussian distribution function."""
    if std == 0:
        return np.zeros_like(x)
    return (1 / (std * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mean) / std) ** 2)


def main():
    parser = argparse.ArgumentParser(description="Draw a histogram from a CSV file.")
    parser.add_argument("input_files", nargs="+", help="Path to the input CSV files")
    parser.add_argument(
        "-o",
        "--output",
        dest="output_file",
        default="histogram.png",
        help="Path to save the output image (e.g., histogram.png)",
    )
    parser.add_argument("--xlabel", help="Label for X axis", default="Distances (m)")
    parser.add_argument(
        "--xlabel_cm", help="Label for X axis in cm", default="Distances (cm)"
    )
    parser.add_argument("--ylabel", help="Label for Y axis", default="Count")

    args = parser.parse_args()

    data_list = []
    global_min_x = float("inf")
    global_max_x = float("-inf")

    # First pass: read all files and determine global x limits
    for file_path in args.input_files:
        if not os.path.exists(file_path):
            print(f"Error: The file '{file_path}' does not exist. Skipping.")
            continue

        try:
            df = pd.read_csv(file_path, sep=";")
            df.columns = df.columns.str.strip()
            df = df.loc[:, ~df.columns.str.contains("^Unnamed")]

            required_columns = ["Value", "Class start", "Class end"]
            missing = [col for col in required_columns if col not in df.columns]
            if missing:
                print(f"Error: Columns {missing} missing from '{file_path}'. Skipping.")
                continue

            df["center"] = (df["Class start"] + df["Class end"]) / 2
            df["width"] = df["Class end"] - df["Class start"]

            min_x = df["Class start"].min()
            max_x = df["Class end"].max()

            if min_x < global_min_x:
                global_min_x = min_x
            if max_x > global_max_x:
                global_max_x = max_x

            data_list.append({"file": file_path, "df": df})

        except Exception as e:
            print(f"Error processing '{file_path}': {e}")

    if not data_list:
        print("No valid data found to plot.")
        return

    # Plotting
    num_plots = len(data_list)
    fig, axes = plt.subplots(
        nrows=num_plots, ncols=1, figsize=(8, 4 * num_plots), sharex=True
    )

    if num_plots == 1:
        axes = [axes]

    # Add a bit of padding to limits
    x_range = global_max_x - global_min_x
    if x_range == 0:
        x_range = 1.0
    xlim_min = global_min_x - x_range * 0.05
    xlim_max = global_max_x + x_range * 0.05

    x_grid = np.linspace(xlim_min, xlim_max, 1000)

    # Define a color cycle
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for i, item in enumerate(data_list):
        ax = axes[i]
        df = item["df"]
        file_name = item["file"]

        # Select color for this plot
        color = colors[i % len(colors)]

        # Series name from filename without extension
        series_name = os.path.splitext(os.path.basename(file_name))[0]

        centers = df["center"].values
        counts = df["Value"].values
        widths = df["width"].values

        # Calculate stats
        mean_val, std_val = calculate_weighted_stats(centers, counts)
        mean_cm = mean_val * 100
        std_cm = std_val * 100
        stats_label = f"Mean: {mean_val:.4f} m ({mean_cm:.2f} cm) | Std: {std_val:.4f} m ({std_cm:.2f} cm)"
        bar_label = f"{series_name}\n{stats_label}"

        # Bar plot
        ax.bar(
            centers,
            counts,
            width=widths,
            align="center",
            alpha=0.5,
            edgecolor="black",
            label=bar_label,
            color=color,
        )

        # Gaussian Distribution Calculation
        gauss_pdf = gaussian_distribution(x_grid, mean_val, std_val)

        # Scale Gaussian to match histogram area
        hist_area = np.sum(counts * widths)
        gauss_scaled = gauss_pdf * hist_area

        # Make the Gaussian curve color slightly darker
        rgb = mcolors.to_rgb(color)
        darker_color = tuple(max(0, x * 0.7) for x in rgb)

        ax.plot(x_grid, gauss_scaled, color=darker_color, linewidth=2)

        # Annotations (Mean/Std in m and cm)
        ax.set_ylabel(args.ylabel)
        ax.grid(axis="y", linestyle="--", alpha=0.7)
        ax.legend()

        # Set global xlim
        ax.set_xlim(xlim_min, xlim_max)

        # Add Secondary X-axis for cm
    ax2 = axes[0].twiny()
    ax2.set_xlim(xlim_min * 100, xlim_max * 100)
    ax2.set_xlabel(args.xlabel_cm)

    axes[-1].set_xlabel(args.xlabel)
    # Adjust layout to accommodate titles and dual axes
    # Increase hspace to avoid overlap between x-axis of one plot and title/x-axis of the next
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    # Additional adjustment for multiple plots to avoid overlap
    # if num_plots > 1:
    #     plt.subplots_adjust(hspace=0.6)

    plt.savefig(args.output_file)
    print(f"Histograms saved successfully to '{args.output_file}'")
    eps_file = Path(args.output_file).with_suffix(".eps")
    plt.savefig(str(eps_file))
    print(f"Histograms saved successfully to '{eps_file}'")


if __name__ == "__main__":
    main()
