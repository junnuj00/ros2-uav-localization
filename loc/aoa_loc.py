import numpy as np


def estimate_emitter(
    uav_positions,
    aoa_measurements,
):
    """
    Estimate emitter position using multiple AOA measurements.

    Parameters
    ----------
    uav_positions : np.ndarray
        UAV positions with shape (N, 2).

    aoa_measurements : np.ndarray
        AOA measurements in radians with shape (N,).

    Returns
    -------
    np.ndarray
        Estimated emitter position [x, y].
    """

    A = []
    b = []

    for uav_pos, angle in zip(
        uav_positions,
        aoa_measurements,
    ):

        x_u, y_u = uav_pos

        sin_theta = np.sin(angle)
        cos_theta = np.cos(angle)

        A.append(
            [
                sin_theta,
                -cos_theta,
            ]
        )

        b.append(
            sin_theta * x_u
            - cos_theta * y_u
        )

    A = np.array(A)
    b = np.array(b)

    estimate, _, _, _ = np.linalg.lstsq(
        A,
        b,
        rcond=None,
    )

    return estimate