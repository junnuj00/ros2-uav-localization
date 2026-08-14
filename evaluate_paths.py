import csv
import os

import numpy as np

from sim.path import generate_uav_path
from sim.aoa import calc_aoa, add_noise
from loc.aoa_loc import estimate_emitter
from filter.ekf import ExtendedKalmanFilter2D
from plan.planner import select_best_waypoint


# --------------------------------------------------
# Evaluation settings
# --------------------------------------------------

NUM_RUNS = 30

EMITTER_POSITION = np.array(
    [30.0, 30.0],
    dtype=float,
)

AOA_NOISE_STD_DEG = 2.0

MEASUREMENT_BUDGET = 25

INITIAL_COVARIANCE = 100.0
PROCESS_NOISE = 0.01

BOOTSTRAP_SIZE = 5


# --------------------------------------------------
# Straight path
# --------------------------------------------------

def generate_straight_path():

    x = np.linspace(
        0.0,
        40.0,
        MEASUREMENT_BUDGET,
    )

    y = np.full(
        MEASUREMENT_BUDGET,
        5.0,
    )

    return np.column_stack(
        (x, y)
    )


# --------------------------------------------------
# Curved path
#
# Use the same curved trajectory as main.py,
# then reduce it to the same measurement budget.
# --------------------------------------------------

def generate_curved_path():

    _, full_path = generate_uav_path(
        start=(0.0, 5.0),
        velocity_x=2.0,
        curve_rate=0.04,
        duration=20.0,
        dt=0.1,
    )

    indices = np.linspace(
        0,
        len(full_path) - 1,
        MEASUREMENT_BUDGET,
        dtype=int,
    )

    return full_path[
        indices
    ]


# --------------------------------------------------
# Generate noisy AOA measurements
# --------------------------------------------------

def generate_measurements(
    path,
):

    measurements = []

    for position in path:

        true_angle = calc_aoa(
            position,
            EMITTER_POSITION,
        )

        noisy_angle = add_noise(
            true_angle,
            noise_std_deg=AOA_NOISE_STD_DEG,
        )

        measurements.append(
            noisy_angle
        )

    return np.array(
        measurements
    )


# --------------------------------------------------
# Run EKF on a predefined path
# --------------------------------------------------

def run_fixed_path_ekf(
    path,
):

    measurements = generate_measurements(
        path
    )

    initial_state = estimate_emitter(
        path[:BOOTSTRAP_SIZE],
        measurements[:BOOTSTRAP_SIZE],
    )

    ekf = ExtendedKalmanFilter2D(
        initial_state=initial_state,
        initial_covariance=INITIAL_COVARIANCE,
        process_noise=PROCESS_NOISE,
        aoa_noise_std_deg=AOA_NOISE_STD_DEG,
    )

    estimate_history = []

    for position, measurement in zip(
        path,
        measurements,
    ):

        ekf.predict()

        state = ekf.update(
            aoa_measurement=measurement,
            uav_position=position,
        )

        estimate_history.append(
            state
        )

    estimate_history = np.array(
        estimate_history
    )

    error_history = np.linalg.norm(
        estimate_history
        - EMITTER_POSITION,
        axis=1,
    )

    mean_error = np.mean(
        error_history
    )

    final_error = error_history[
        -1
    ]

    return (
        mean_error,
        final_error,
    )


# --------------------------------------------------
# Run FIM adaptive path
#
# This follows the FIM structure used in main.py:
# 5 bootstrap measurements
# + 20 adaptive measurements
# = 25 total measurements.
# --------------------------------------------------

def run_fim_path():

    bootstrap_positions = np.array(
        [
            [0.0, 5.0],
            [5.0, 5.0],
            [10.0, 7.0],
            [15.0, 10.0],
            [20.0, 14.0],
        ],
        dtype=float,
    )

    bootstrap_measurements = (
        generate_measurements(
            bootstrap_positions
        )
    )

    initial_state = estimate_emitter(
        bootstrap_positions,
        bootstrap_measurements,
    )

    ekf = ExtendedKalmanFilter2D(
        initial_state=initial_state,
        initial_covariance=INITIAL_COVARIANCE,
        process_noise=PROCESS_NOISE,
        aoa_noise_std_deg=AOA_NOISE_STD_DEG,
    )

    path = list(
        bootstrap_positions.copy()
    )

    estimate_history = []

    # Feed bootstrap measurements
    # into the EKF.

    for position, measurement in zip(
        bootstrap_positions,
        bootstrap_measurements,
    ):

        ekf.predict()

        state = ekf.update(
            aoa_measurement=measurement,
            uav_position=position,
        )

        estimate_history.append(
            state
        )

    # 20 adaptive planning steps

    num_planning_steps = (
        MEASUREMENT_BUDGET
        - len(bootstrap_positions)
    )

    for _ in range(
        num_planning_steps
    ):

        current_position = np.array(
            path[-1]
        )

        current_estimate = (
            ekf.x.copy()
        )

        current_positions = np.array(
            path
        )

        (
            best_waypoint,
            _,
            _,
            _,
        ) = select_best_waypoint(
            current_positions=current_positions,
            current_position=current_position,
            emitter_estimate=current_estimate,
            radius=5.0,
            num_candidates=8,
            aoa_noise_std_deg=AOA_NOISE_STD_DEG,
        )

        path.append(
            best_waypoint.copy()
        )

        true_angle = calc_aoa(
            best_waypoint,
            EMITTER_POSITION,
        )

        noisy_angle = add_noise(
            true_angle,
            noise_std_deg=AOA_NOISE_STD_DEG,
        )

        ekf.predict()

        state = ekf.update(
            aoa_measurement=noisy_angle,
            uav_position=best_waypoint,
        )

        estimate_history.append(
            state
        )

    estimate_history = np.array(
        estimate_history
    )

    error_history = np.linalg.norm(
        estimate_history
        - EMITTER_POSITION,
        axis=1,
    )

    mean_error = np.mean(
        error_history
    )

    final_error = error_history[
        -1
    ]

    return (
        mean_error,
        final_error,
    )


# --------------------------------------------------
# Summary statistics
# --------------------------------------------------

def print_summary(
    name,
    mean_errors,
    final_errors,
):

    print()
    print(
        f"===== {name} ====="
    )

    print(
        f"Mean EKF error: "
        f"{np.mean(mean_errors):.4f} m"
    )

    print(
        f"Std of mean error: "
        f"{np.std(mean_errors, ddof=1):.4f} m"
    )

    print(
        f"Median mean error: "
        f"{np.median(mean_errors):.4f} m"
    )

    print(
        f"Best mean error: "
        f"{np.min(mean_errors):.4f} m"
    )

    print(
        f"Worst mean error: "
        f"{np.max(mean_errors):.4f} m"
    )

    print(
        f"Mean final error: "
        f"{np.mean(final_errors):.4f} m"
    )

    print(
        f"Std of final error: "
        f"{np.std(final_errors, ddof=1):.4f} m"
    )


# --------------------------------------------------
# Main evaluation
# --------------------------------------------------

def main():

    straight_path = (
        generate_straight_path()
    )

    curved_path = (
        generate_curved_path()
    )

    results = []

    straight_mean_errors = []
    straight_final_errors = []

    curved_mean_errors = []
    curved_final_errors = []

    fim_mean_errors = []
    fim_final_errors = []

    print(
        "Starting quantitative evaluation..."
    )

    print(
        f"Monte Carlo runs: {NUM_RUNS}"
    )

    print(
        f"Measurement budget: "
        f"{MEASUREMENT_BUDGET}"
    )

    print(
        f"AOA noise std: "
        f"{AOA_NOISE_STD_DEG:.1f} deg"
    )

    # --------------------------------------------------
    # Monte Carlo runs
    # --------------------------------------------------

    for run in range(
        NUM_RUNS
    ):

        seed = run

        # Straight

        np.random.seed(
            seed
        )

        (
            straight_mean,
            straight_final,
        ) = run_fixed_path_ekf(
            straight_path
        )

        # Curved
        #
        # Reset to the same seed so that
        # predefined paths receive the same
        # random-noise sequence.

        np.random.seed(
            seed
        )

        (
            curved_mean,
            curved_final,
        ) = run_fixed_path_ekf(
            curved_path
        )

        # FIM

        np.random.seed(
            seed
        )

        (
            fim_mean,
            fim_final,
        ) = run_fim_path()

        straight_mean_errors.append(
            straight_mean
        )

        straight_final_errors.append(
            straight_final
        )

        curved_mean_errors.append(
            curved_mean
        )

        curved_final_errors.append(
            curved_final
        )

        fim_mean_errors.append(
            fim_mean
        )

        fim_final_errors.append(
            fim_final
        )

        results.append(
            [
                run,
                seed,
                straight_mean,
                straight_final,
                curved_mean,
                curved_final,
                fim_mean,
                fim_final,
            ]
        )

        print(
            f"Run {run + 1:02d}/{NUM_RUNS} | "
            f"Straight={straight_mean:.3f} m | "
            f"Curved={curved_mean:.3f} m | "
            f"FIM={fim_mean:.3f} m"
        )

    # --------------------------------------------------
    # Save CSV
    # --------------------------------------------------

    os.makedirs(
        "results",
        exist_ok=True,
    )

    output_path = os.path.join(
        "results",
        "path_evaluation.csv",
    )

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:

        writer = csv.writer(
            file
        )

        writer.writerow(
            [
                "run",
                "seed",
                "straight_mean_error",
                "straight_final_error",
                "curved_mean_error",
                "curved_final_error",
                "fim_mean_error",
                "fim_final_error",
            ]
        )

        writer.writerows(
            results
        )

    # --------------------------------------------------
    # Print summary
    # --------------------------------------------------

    print_summary(
        "STRAIGHT PATH",
        straight_mean_errors,
        straight_final_errors,
    )

    print_summary(
        "CURVED PATH",
        curved_mean_errors,
        curved_final_errors,
    )

    print_summary(
        "FIM PATH",
        fim_mean_errors,
        fim_final_errors,
    )

    straight_average = np.mean(
        straight_mean_errors
    )

    curved_average = np.mean(
        curved_mean_errors
    )

    fim_average = np.mean(
        fim_mean_errors
    )

    fim_vs_straight = (
        (
            straight_average
            - fim_average
        )
        / straight_average
        * 100.0
    )

    fim_vs_curved = (
        (
            curved_average
            - fim_average
        )
        / curved_average
        * 100.0
    )

    print()
    print(
        "===== FIM IMPROVEMENT ====="
    )

    print(
        f"vs Straight: "
        f"{fim_vs_straight:.2f}%"
    )

    print(
        f"vs Curved: "
        f"{fim_vs_curved:.2f}%"
    )

    print()
    print(
        f"CSV saved to: "
        f"{output_path}"
    )


if __name__ == "__main__":
    main()