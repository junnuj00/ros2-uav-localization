import numpy as np
import matplotlib.pyplot as plt

from sim.path import generate_uav_path
from sim.aoa import calc_aoa, add_noise
from loc.aoa_loc import estimate_emitter
from filter.kf import KalmanFilter2D
from filter.ekf import ExtendedKalmanFilter2D


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
            noise_std_deg=2.0,
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
    # Rolling LS localization
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
    # Rolling LS errors
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
    #
    # Best R from Experiment 04
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
    #
    # Use first 30 AOA measurements
    # to generate initial LS position.
    # --------------------------------------------------

    initial_ekf_state = estimate_emitter(
        uav_positions[:window_size],
        noisy_aoa[:window_size],
    )

    ekf = ExtendedKalmanFilter2D(
        initial_state=initial_ekf_state,
        initial_covariance=100.0,
        process_noise=0.01,
        aoa_noise_std_deg=2.0,
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
        f"EKF mean error: "
        f"{ekf_mean_error:.3f} m"
    )

    print()

    print(
        "Initial EKF state:"
    )
    print(
        initial_ekf_state
    )

    print()

    print(
        "Final EKF state:"
    )
    print(
        ekf_history[-1]
    )

    # --------------------------------------------------
    # Plot 1
    # UAV path + AOA localization
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 6)
    )

    plt.plot(
        uav_positions[:, 0],
        uav_positions[:, 1],
        label="UAV Path",
    )

    plt.scatter(
        uav_positions[0, 0],
        uav_positions[0, 1],
        marker="o",
        s=80,
        label="UAV Start",
    )

    plt.scatter(
        emitter_pos[0],
        emitter_pos[1],
        marker="*",
        s=200,
        label="True Emitter",
    )

    plt.scatter(
        final_ls_estimate[0],
        final_ls_estimate[1],
        marker="x",
        s=150,
        label="Final LS Estimate",
    )

    sample_indices = np.linspace(
        0,
        len(uav_positions) - 1,
        6,
        dtype=int,
    )

    ray_length = 30.0

    for idx in sample_indices:

        uav_x, uav_y = (
            uav_positions[idx]
        )

        angle = noisy_aoa[idx]

        ray_x = (
            uav_x
            + ray_length
            * np.cos(angle)
        )

        ray_y = (
            uav_y
            + ray_length
            * np.sin(angle)
        )

        plt.plot(
            [uav_x, ray_x],
            [uav_y, ray_y],
            linestyle="--",
            alpha=0.5,
        )

    plt.xlabel(
        "X Position [m]"
    )

    plt.ylabel(
        "Y Position [m]"
    )

    plt.title(
        "Curved UAV Path and AOA Measurements"
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
    # AOA measurement
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 5)
    )

    plt.plot(
        time,
        np.rad2deg(
            true_aoa
        ),
        label="True AOA",
    )

    plt.plot(
        time,
        np.rad2deg(
            noisy_aoa
        ),
        label="Noisy AOA",
        alpha=0.7,
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "AOA [deg]"
    )

    plt.title(
        "AOA Measurement Simulation"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.show()

    # --------------------------------------------------
    # Plot 3
    # Position estimates
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
    # Plot 4
    # Error comparison
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
        label="EKF",
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


if __name__ == "__main__":
    main()