import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# --------------------------------------------------
# Paths
# --------------------------------------------------

CSV_PATH = os.path.join(
    "results",
    "path_evaluation.csv",
)

OUTPUT_DIR = os.path.join(
    "results",
    "plots",
)


# --------------------------------------------------
# Load evaluation results
# --------------------------------------------------

def load_results():

    dataframe = pd.read_csv(
        CSV_PATH
    )

    return dataframe


# --------------------------------------------------
# Plot 1
# Mean EKF localization error
# --------------------------------------------------

def plot_mean_error_comparison(
    dataframe,
):

    labels = [
        "Straight",
        "Curved",
        "FIM",
    ]

    columns = [
        "straight_mean_error",
        "curved_mean_error",
        "fim_mean_error",
    ]

    means = [
        dataframe[column].mean()
        for column in columns
    ]

    stds = [
        dataframe[column].std(
            ddof=1
        )
        for column in columns
    ]

    plt.figure(
        figsize=(8, 5)
    )

    bars = plt.bar(
        labels,
        means,
        yerr=stds,
        capsize=6,
    )

    plt.ylabel(
        "Mean EKF Localization Error [m]"
    )

    plt.title(
        "Monte Carlo Mean Localization Error"
    )

    plt.grid(
        axis="y",
        alpha=0.3,
    )

    for bar, value in zip(
        bars,
        means,
    ):

        plt.text(
            bar.get_x()
            + bar.get_width() / 2.0,
            bar.get_height() + 0.08,
            f"{value:.3f} m",
            ha="center",
            va="bottom",
        )

    plt.tight_layout()

    output_path = os.path.join(
        OUTPUT_DIR,
        "mean_error_comparison.png",
    )

    plt.savefig(
        output_path,
        dpi=200,
        bbox_inches="tight",
    )

    plt.close()

    print(
        f"Saved: {output_path}"
    )


# --------------------------------------------------
# Plot 2
# Monte Carlo error distribution
# --------------------------------------------------

def plot_error_distribution(
    dataframe,
):

    data = [
        dataframe[
            "straight_mean_error"
        ].to_numpy(),

        dataframe[
            "curved_mean_error"
        ].to_numpy(),

        dataframe[
            "fim_mean_error"
        ].to_numpy(),
    ]

    labels = [
        "Straight",
        "Curved",
        "FIM",
    ]

    plt.figure(
        figsize=(8, 5)
    )

    plt.boxplot(
        data,
        tick_labels=labels,
        showmeans=True,
    )

    plt.ylabel(
        "Mean EKF Localization Error [m]"
    )

    plt.title(
        "Monte Carlo Localization Error Distribution"
    )

    plt.grid(
        axis="y",
        alpha=0.3,
    )

    plt.tight_layout()

    output_path = os.path.join(
        OUTPUT_DIR,
        "mean_error_distribution.png",
    )

    plt.savefig(
        output_path,
        dpi=200,
        bbox_inches="tight",
    )

    plt.close()

    print(
        f"Saved: {output_path}"
    )


# --------------------------------------------------
# Plot 3
# Final localization error comparison
# --------------------------------------------------

def plot_final_error_comparison(
    dataframe,
):

    labels = [
        "Straight",
        "Curved",
        "FIM",
    ]

    columns = [
        "straight_final_error",
        "curved_final_error",
        "fim_final_error",
    ]

    means = [
        dataframe[column].mean()
        for column in columns
    ]

    stds = [
        dataframe[column].std(
            ddof=1
        )
        for column in columns
    ]

    plt.figure(
        figsize=(8, 5)
    )

    bars = plt.bar(
        labels,
        means,
        yerr=stds,
        capsize=6,
    )

    plt.ylabel(
        "Final Localization Error [m]"
    )

    plt.title(
        "Monte Carlo Final Localization Error"
    )

    plt.grid(
        axis="y",
        alpha=0.3,
    )

    for bar, value in zip(
        bars,
        means,
    ):

        plt.text(
            bar.get_x()
            + bar.get_width() / 2.0,
            bar.get_height() + 0.02,
            f"{value:.3f} m",
            ha="center",
            va="bottom",
        )

    plt.tight_layout()

    output_path = os.path.join(
        OUTPUT_DIR,
        "final_error_comparison.png",
    )

    plt.savefig(
        output_path,
        dpi=200,
        bbox_inches="tight",
    )

    plt.close()

    print(
        f"Saved: {output_path}"
    )


# --------------------------------------------------
# Print summary
# --------------------------------------------------

def print_summary(
    dataframe,
):

    straight_mean = dataframe[
        "straight_mean_error"
    ].mean()

    curved_mean = dataframe[
        "curved_mean_error"
    ].mean()

    fim_mean = dataframe[
        "fim_mean_error"
    ].mean()

    straight_final = dataframe[
        "straight_final_error"
    ].mean()

    curved_final = dataframe[
        "curved_final_error"
    ].mean()

    fim_final = dataframe[
        "fim_final_error"
    ].mean()

    print()
    print(
        "===== PLOT SUMMARY ====="
    )

    print(
        f"Straight mean error: "
        f"{straight_mean:.4f} m"
    )

    print(
        f"Curved mean error: "
        f"{curved_mean:.4f} m"
    )

    print(
        f"FIM mean error: "
        f"{fim_mean:.4f} m"
    )

    print()

    print(
        f"Straight final error: "
        f"{straight_final:.4f} m"
    )

    print(
        f"Curved final error: "
        f"{curved_final:.4f} m"
    )

    print(
        f"FIM final error: "
        f"{fim_final:.4f} m"
    )


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():

    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True,
    )

    dataframe = load_results()

    print(
        f"Loaded {len(dataframe)} "
        f"Monte Carlo runs."
    )

    plot_mean_error_comparison(
        dataframe
    )

    plot_error_distribution(
        dataframe
    )

    plot_final_error_comparison(
        dataframe
    )

    print_summary(
        dataframe
    )

    print()
    print(
        "Plot generation complete."
    )


if __name__ == "__main__":
    main()