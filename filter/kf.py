import numpy as np


class KalmanFilter2D:
    """
    Simple 2D Kalman Filter for a stationary target.

    State:
        x = [position_x,
             position_y]

    Measurement:
        z = [measured_x,
             measured_y]
    """

    def __init__(
        self,
        initial_state,
        initial_covariance=10.0,
        process_noise=0.01,
        measurement_noise=4.0,
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
        # Measurement matrix
        #
        # We directly measure x and y.
        # ----------------------------------------------

        self.H = np.eye(2)

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
        # Measurement noise covariance
        # ----------------------------------------------

        self.R = (
            np.eye(2)
            * measurement_noise
        )

    def predict(self):
        """
        Prediction step.
        """

        # State prediction
        self.x = self.F @ self.x

        # Covariance prediction
        self.P = (
            self.F
            @ self.P
            @ self.F.T
            + self.Q
        )

        return self.x.copy()

    def update(self, measurement):
        """
        Measurement update step.
        """

        z = np.array(
            measurement,
            dtype=float,
        )

        # ----------------------------------------------
        # Innovation
        # y = z - Hx
        # ----------------------------------------------

        innovation = (
            z
            - self.H @ self.x
        )

        # ----------------------------------------------
        # Innovation covariance
        # S = HPH^T + R
        # ----------------------------------------------

        S = (
            self.H
            @ self.P
            @ self.H.T
            + self.R
        )

        # ----------------------------------------------
        # Kalman Gain
        # K = PH^T S^-1
        # ----------------------------------------------

        K = (
            self.P
            @ self.H.T
            @ np.linalg.inv(S)
        )

        # ----------------------------------------------
        # State update
        # ----------------------------------------------

        self.x = (
            self.x
            + K @ innovation
        )

        # ----------------------------------------------
        # Covariance update
        # ----------------------------------------------

        I = np.eye(2)

        self.P = (
            (I - K @ self.H)
            @ self.P
        )

        return self.x.copy()