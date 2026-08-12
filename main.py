import numpy as np
import matplotlib.pyplot as plt

from sim.path import generate_uav_path
from sim.aoa import calc_aoa, add_noise

from loc.aoa_loc import estimate_emitter

from filter.kf import KalmanFilter2D
from filter.ekf import ExtendedKalmanFilter2D

from plan.planner import select_best_waypoint
from plan.fim import crlb_position_bound


def run_kalman_filter(
    measurements,
    process_noise,
    measurement_noise,
    initial_covariance=10.0,
):
    """
    Run linear Kalman Filter for
    2D LS position measurements.
    """

    kf = KalmanFilter2D(
        initial_state=measurements[0],
        initial_covariance=initial_covariance,
        process_noise=process_noise,
        measurement_noise=measurement_noise,
    )

    filtered_history = []

    for measurement in measurements:

        kf.predict()

        filtered_state = kf.update(
            measurement
        )

        filtered_history.append(
            filtered_state
        )

    return np.array(
        filtered_history
    )


def main():

    # --------------------------------------------------
    # Random seed
    # --------------------------------------------------

    np.random.seed(42)

    # --------------------------------------------------
    # Simulation settings
    # --------------------------------------------------

    emitter_pos = np.array(
        [30.0, 30.0]
    )

    aoa_noise_std_deg = 2.0

    # --------------------------------------------------
    # Baseline curved UAV path
    # --------------------------------------------------

    time, uav_positions = generate_uav_path(
        start=(0.0, 5.0),
        velocity_x=2.0,
        curve_rate=0.04,
        duration=20.0,
        dt=0.1,
    )

    true_aoa = []
    noisy_aoa = []

    # --------------------------------------------------
    # Generate AOA measurements
    # --------------------------------------------------

    for uav_pos in uav_positions:

        true_angle = calc_aoa(
            uav_pos,
            emitter_pos,
        )

        noisy_angle = add_noise(
            true_angle,
            noise_std_deg=aoa_noise_std_deg,
        )

        true_aoa.append(
            true_angle
        )

        noisy_aoa.append(
            noisy_angle
        )

    true_aoa = np.array(
        true_aoa
    )

    noisy_aoa = np.array(
        noisy_aoa
    )

    # --------------------------------------------------
    # Final LS estimation
    # --------------------------------------------------

    final_ls_estimate = estimate_emitter(
        uav_positions,
        noisy_aoa,
    )

    final_ls_error = np.linalg.norm(
        final_ls_estimate
        - emitter_pos
    )

    # --------------------------------------------------
    # Rolling-window LS localization
    # --------------------------------------------------

    window_size = 30

    raw_history = []

    for i in range(
        window_size,
        len(uav_positions) + 1,
    ):

        window_positions = uav_positions[
            i - window_size:i
        ]

        window_aoa = noisy_aoa[
            i - window_size:i
        ]

        current_estimate = estimate_emitter(
            window_positions,
            window_aoa,
        )

        raw_history.append(
            current_estimate
        )

    raw_history = np.array(
        raw_history
    )

    history_time = time[
        window_size - 1:
    ]

    # --------------------------------------------------
    # Rolling LS error
    # --------------------------------------------------

    raw_error_history = np.linalg.norm(
        raw_history
        - emitter_pos,
        axis=1,
    )

    raw_mean_error = np.mean(
        raw_error_history
    )

    # --------------------------------------------------
    # Linear Kalman Filter
    # --------------------------------------------------

    linear_kf_history = run_kalman_filter(
        measurements=raw_history,
        process_noise=0.01,
        measurement_noise=0.5,
        initial_covariance=10.0,
    )

    linear_kf_error_history = np.linalg.norm(
        linear_kf_history
        - emitter_pos,
        axis=1,
    )

    linear_kf_mean_error = np.mean(
        linear_kf_error_history
    )

    # --------------------------------------------------
    # EKF initialization
    # --------------------------------------------------

    initial_ekf_state = estimate_emitter(
        uav_positions[:window_size],
        noisy_aoa[:window_size],
    )

    ekf = ExtendedKalmanFilter2D(
        initial_state=initial_ekf_state,
        initial_covariance=100.0,
        process_noise=0.01,
        aoa_noise_std_deg=aoa_noise_std_deg,
    )

    ekf_history = []

    # --------------------------------------------------
    # EKF sequential AOA updates
    # --------------------------------------------------

    for i in range(
        window_size - 1,
        len(uav_positions),
    ):

        ekf.predict()

        ekf_state = ekf.update(
            aoa_measurement=noisy_aoa[i],
            uav_position=uav_positions[i],
        )

        ekf_history.append(
            ekf_state
        )

    ekf_history = np.array(
        ekf_history
    )

    # --------------------------------------------------
    # EKF error
    # --------------------------------------------------

    ekf_error_history = np.linalg.norm(
        ekf_history
        - emitter_pos,
        axis=1,
    )

    ekf_mean_error = np.mean(
        ekf_error_history
    )

    # --------------------------------------------------
    # FIM-based adaptive path
    # --------------------------------------------------

    bootstrap_positions = np.array(
        [
            [0.0, 5.0],
            [5.0, 5.0],
            [10.0, 7.0],
            [15.0, 10.0],
            [20.0, 14.0],
        ]
    )

    bootstrap_aoa = []

    # --------------------------------------------------
    # Bootstrap AOA measurements
    # --------------------------------------------------

    for position in bootstrap_positions:

        true_angle = calc_aoa(
            position,
            emitter_pos,
        )

        noisy_angle = add_noise(
            true_angle,
            noise_std_deg=aoa_noise_std_deg,
        )

        bootstrap_aoa.append(
            noisy_angle
        )

    bootstrap_aoa = np.array(
        bootstrap_aoa
    )

    # --------------------------------------------------
    # Initial estimate for FIM-path EKF
    # --------------------------------------------------

    fim_initial_state = estimate_emitter(
        bootstrap_positions,
        bootstrap_aoa,
    )

    fim_ekf = ExtendedKalmanFilter2D(
        initial_state=fim_initial_state,
        initial_covariance=100.0,
        process_noise=0.01,
        aoa_noise_std_deg=aoa_noise_std_deg,
    )

    # --------------------------------------------------
    # FIM path initialization
    # --------------------------------------------------

    fim_path = list(
        bootstrap_positions.copy()
    )

    fim_estimate_history = []

    # --------------------------------------------------
    # Feed bootstrap measurements into EKF
    # --------------------------------------------------

    for position, aoa in zip(
        bootstrap_positions,
        bootstrap_aoa,
    ):

        fim_ekf.predict()

        state = fim_ekf.update(
            aoa_measurement=aoa,
            uav_position=position,
        )

        fim_estimate_history.append(
            state
        )

    # --------------------------------------------------
    # Adaptive FIM path planning loop
    # --------------------------------------------------

    num_planning_steps = 20

    for step in range(
        num_planning_steps
    ):

        current_position = np.array(
            fim_path[-1]
        )

        current_estimate = (
            fim_ekf.x.copy()
        )

        current_positions_array = np.array(
            fim_path
        )

        (
            best_waypoint,
            best_score,
            candidates,
            scores,
        ) = select_best_waypoint(
            current_positions=current_positions_array,
            current_position=current_position,
            emitter_estimate=current_estimate,
            radius=5.0,
            num_candidates=8,
            aoa_noise_std_deg=aoa_noise_std_deg,
        )

        # ----------------------------------------------
        # Move UAV to selected waypoint
        # ----------------------------------------------

        fim_path.append(
            best_waypoint.copy()
        )

        # ----------------------------------------------
        # Generate new AOA measurement
        # ----------------------------------------------

        true_angle = calc_aoa(
            best_waypoint,
            emitter_pos,
        )

        noisy_angle = add_noise(
            true_angle,
            noise_std_deg=aoa_noise_std_deg,
        )

        # ----------------------------------------------
        # EKF update
        # ----------------------------------------------

        fim_ekf.predict()

        fim_state = fim_ekf.update(
            aoa_measurement=noisy_angle,
            uav_position=best_waypoint,
        )

        fim_estimate_history.append(
            fim_state
        )

    fim_path = np.array(
        fim_path
    )

    fim_estimate_history = np.array(
        fim_estimate_history
    )

    # --------------------------------------------------
    # FIM-path EKF error
    # --------------------------------------------------

    fim_error_history = np.linalg.norm(
        fim_estimate_history
        - emitter_pos,
        axis=1,
    )

    fim_mean_error = np.mean(
        fim_error_history
    )

    fim_final_error = np.linalg.norm(
        fim_estimate_history[-1]
        - emitter_pos
    )

    # --------------------------------------------------
    # Equal-budget CRLB comparison
    # --------------------------------------------------

    measurement_budget = len(
        fim_path
    )

    curved_indices = np.linspace(
        0,
        len(uav_positions) - 1,
        measurement_budget,
        dtype=int,
    )

    curved_path_equal = uav_positions[
        curved_indices
    ]

    curved_path_crlb = crlb_position_bound(
        uav_positions=curved_path_equal,
        emitter_position=emitter_pos,
        aoa_noise_std_deg=aoa_noise_std_deg,
    )

    fim_path_crlb = crlb_position_bound(
        uav_positions=fim_path,
        emitter_position=emitter_pos,
        aoa_noise_std_deg=aoa_noise_std_deg,
    )

    crlb_improvement = (
        (
            curved_path_crlb
            - fim_path_crlb
        )
        / curved_path_crlb
        * 100.0
    )

    # --------------------------------------------------
    # Print results
    # --------------------------------------------------

    print(
        "True emitter position:"
    )

    print(
        emitter_pos
    )

    print()

    print(
        "Final LS estimate:"
    )

    print(
        final_ls_estimate
    )

    print()

    print(
        f"Final LS localization error: "
        f"{final_ls_error:.3f} m"
    )

    print()

    print(
        f"Rolling LS mean error: "
        f"{raw_mean_error:.3f} m"
    )

    print(
        f"Linear KF mean error: "
        f"{linear_kf_mean_error:.3f} m"
    )

    print(
        f"Curved-path EKF mean error: "
        f"{ekf_mean_error:.3f} m"
    )

    print()

    print(
        "FIM-path initial estimate:"
    )

    print(
        fim_initial_state
    )

    print()

    print(
        "FIM-path final EKF estimate:"
    )

    print(
        fim_estimate_history[-1]
    )

    print()

    print(
        f"FIM-path EKF mean error: "
        f"{fim_mean_error:.3f} m"
    )

    print(
        f"FIM-path final error: "
        f"{fim_final_error:.3f} m"
    )

    print()

    print(
        "Equal-budget CRLB comparison:"
    )

    print(
        f"Measurement budget: "
        f"{measurement_budget}"
    )

    print(
        f"Curved-path CRLB position bound: "
        f"{curved_path_crlb:.4f} m"
    )

    print(
        f"FIM-path CRLB position bound: "
        f"{fim_path_crlb:.4f} m"
    )

    print(
        f"CRLB improvement: "
        f"{crlb_improvement:.2f}%"
    )

    # --------------------------------------------------
    # Plot 1
    # Baseline curved UAV path
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 6)
    )

    plt.plot(
        uav_positions[:, 0],
        uav_positions[:, 1],
        label="Curved UAV Path",
    )

    plt.scatter(
        emitter_pos[0],
        emitter_pos[1],
        marker="*",
        s=200,
        label="True Emitter",
    )

    plt.scatter(
        uav_positions[0, 0],
        uav_positions[0, 1],
        marker="o",
        s=80,
        label="UAV Start",
    )

    plt.xlabel(
        "X Position [m]"
    )

    plt.ylabel(
        "Y Position [m]"
    )

    plt.title(
        "Baseline Curved UAV Path"
    )

    plt.axis(
        "equal"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()

    # --------------------------------------------------
    # Plot 2
    # LS vs Linear KF vs EKF
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 6)
    )

    plt.plot(
        raw_history[:, 0],
        raw_history[:, 1],
        alpha=0.5,
        label="Rolling LS",
    )

    plt.plot(
        linear_kf_history[:, 0],
        linear_kf_history[:, 1],
        label="Linear KF",
    )

    plt.plot(
        ekf_history[:, 0],
        ekf_history[:, 1],
        label="EKF",
    )

    plt.scatter(
        emitter_pos[0],
        emitter_pos[1],
        marker="*",
        s=200,
        label="True Emitter",
    )

    plt.xlabel(
        "Estimated X [m]"
    )

    plt.ylabel(
        "Estimated Y [m]"
    )

    plt.title(
        "LS vs Linear KF vs EKF"
    )

    plt.axis(
        "equal"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()

    # --------------------------------------------------
    # Plot 3
    # Localization error comparison
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 5)
    )

    plt.plot(
        history_time,
        raw_error_history,
        label="Rolling LS",
    )

    plt.plot(
        history_time,
        linear_kf_error_history,
        label="Linear KF",
    )

    plt.plot(
        history_time,
        ekf_error_history,
        label="Curved-path EKF",
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Localization Error [m]"
    )

    plt.title(
        "Localization Error Comparison"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()

    # --------------------------------------------------
    # Plot 4
    # FIM adaptive path
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 6)
    )

    plt.plot(
        fim_path[:, 0],
        fim_path[:, 1],
        marker="o",
        label="FIM-based Path",
    )

    plt.scatter(
        emitter_pos[0],
        emitter_pos[1],
        marker="*",
        s=200,
        label="True Emitter",
    )

    plt.scatter(
        fim_estimate_history[-1, 0],
        fim_estimate_history[-1, 1],
        marker="x",
        s=150,
        label="Final EKF Estimate",
    )

    plt.xlabel(
        "X Position [m]"
    )

    plt.ylabel(
        "Y Position [m]"
    )

    plt.title(
        "FIM-based Adaptive UAV Path"
    )

    plt.axis(
        "equal"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()

    # --------------------------------------------------
    # Plot 5
    # FIM-path localization error
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 5)
    )

    plt.plot(
        np.arange(
            len(fim_error_history)
        ),
        fim_error_history,
        label="FIM-path EKF Error",
    )

    plt.xlabel(
        "Measurement Step"
    )

    plt.ylabel(
        "Localization Error [m]"
    )

    plt.title(
        "FIM-based Path Localization Error"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()

    # --------------------------------------------------
    # Plot 6
    # Equal-budget path comparison
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 6)
    )

    plt.plot(
        curved_path_equal[:, 0],
        curved_path_equal[:, 1],
        marker="o",
        label="Curved Path (Equal Budget)",
    )

    plt.plot(
        fim_path[:, 0],
        fim_path[:, 1],
        marker="o",
        label="FIM-based Path",
    )

    plt.scatter(
        emitter_pos[0],
        emitter_pos[1],
        marker="*",
        s=200,
        label="True Emitter",
    )

    plt.xlabel(
        "X Position [m]"
    )

    plt.ylabel(
        "Y Position [m]"
    )

    plt.title(
        "Equal-budget Curved Path vs FIM-based Path"
    )

    plt.axis(
        "equal"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()