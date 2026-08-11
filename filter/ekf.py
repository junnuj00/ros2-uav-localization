import numpy as np


class ExtendedKalmanFilter2D:
    """
    Extended Kalman Filter for stationary
    emitter localization using AOA measurements.

    State:
        x = [emitter_x,
             emitter_y]

    Measurement:
        z = AOA angle [rad]
    """

    def __init__(
        self,
        initial_state,
        initial_covariance=100.0,
        process_noise=0.01,
        aoa_noise_std_deg=2.0,
    ):
        # ----------------------------------------------
        # State estimate
        # ----------------------------------------------

        self.x = np.array(
            initial_state,
            dtype=float,
        )

        # ----------------------------------------------
        # State transition matrix
        #
        # Stationary emitter:
        #
        # x_k = x_(k-1)
        # ----------------------------------------------

        self.F = np.eye(2)

        # ----------------------------------------------
        # State covariance
        # ----------------------------------------------

        self.P = (
            np.eye(2)
            * initial_covariance
        )

        # ----------------------------------------------
        # Process noise covariance
        # ----------------------------------------------

        self.Q = (
            np.eye(2)
            * process_noise
        )

        # ----------------------------------------------
        # AOA measurement noise variance
        #
        # R is scalar because measurement
        # is one angle.
        # ----------------------------------------------

        noise_std_rad = np.deg2rad(
            aoa_noise_std_deg
        )

        self.R = noise_std_rad ** 2

    def predict(self):
        """
        EKF prediction step.
        """

        self.x = (
            self.F @ self.x
        )

        self.P = (
            self.F
            @ self.P
            @ self.F.T
            + self.Q
        )

        return self.x.copy()

    def measurement_function(
        self,
        uav_position,
    ):
        """
        Predict AOA measurement.

        h(x) = atan2(
            emitter_y - uav_y,
            emitter_x - uav_x
        )
        """

        emitter_x = self.x[0]
        emitter_y = self.x[1]

        uav_x = uav_position[0]
        uav_y = uav_position[1]

        dx = (
            emitter_x
            - uav_x
        )

        dy = (
            emitter_y
            - uav_y
        )

        predicted_angle = np.arctan2(
            dy,
            dx,
        )

        return predicted_angle

    def jacobian(
        self,
        uav_position,
    ):
        """
        Compute Jacobian of AOA measurement model.

        H = [
            -dy / (dx^2 + dy^2),
             dx / (dx^2 + dy^2)
        ]
        """

        emitter_x = self.x[0]
        emitter_y = self.x[1]

        uav_x = uav_position[0]
        uav_y = uav_position[1]

        dx = (
            emitter_x
            - uav_x
        )

        dy = (
            emitter_y
            - uav_y
        )

        distance_squared = (
            dx ** 2
            + dy ** 2
        )

        # Prevent division by zero
        distance_squared = max(
            distance_squared,
            1e-9,
        )

        H = np.array(
            [
                [
                    -dy / distance_squared,
                    dx / distance_squared,
                ]
            ]
        )

        return H

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi].
        """

        return (
            angle + np.pi
        ) % (
            2.0 * np.pi
        ) - np.pi

    def update(
        self,
        aoa_measurement,
        uav_position,
    ):
        """
        EKF measurement update.
        """

        # ----------------------------------------------
        # Predicted nonlinear measurement
        # ----------------------------------------------

        predicted_aoa = (
            self.measurement_function(
                uav_position
            )
        )

        # ----------------------------------------------
        # Measurement Jacobian
        # ----------------------------------------------

        H = self.jacobian(
            uav_position
        )

        # ----------------------------------------------
        # Innovation
        #
        # y = z - h(x)
        # ----------------------------------------------

        innovation = (
            aoa_measurement
            - predicted_aoa
        )

        innovation = (
            self.normalize_angle(
                innovation
            )
        )

        # ----------------------------------------------
        # Innovation covariance
        #
        # S = H P H^T + R
        # ----------------------------------------------

        S = (
            H
            @ self.P
            @ H.T
        )[0, 0] + self.R

        # ----------------------------------------------
        # Kalman gain
        #
        # K = P H^T S^-1
        # ----------------------------------------------

        K = (
            self.P
            @ H.T
        ) / S

        # ----------------------------------------------
        # State update
        # ----------------------------------------------

        self.x = (
            self.x
            + K[:, 0]
            * innovation
        )

        # ----------------------------------------------
        # Covariance update
        # ----------------------------------------------

        I = np.eye(2)

        self.P = (
            I - K @ H
        ) @ self.P

        return self.x.copy()