import numpy as np


def calc_aoa(uav_pos, emitter_pos):
    """
    Calculate the true Angle of Arrival (AOA).

    Parameters
    ----------
    uav_pos : array-like
        UAV position [x, y].
    emitter_pos : array-like
        Emitter position [x, y].

    Returns
    -------
    float
        True AOA in radians.
    """

    ux, uy = uav_pos
    ex, ey = emitter_pos

    dx = ex - ux
    dy = ey - uy

    angle = np.arctan2(dy, dx)

    return angle


def add_noise(angle, noise_std_deg=2.0):
    """
    Add Gaussian measurement noise to AOA.

    Parameters
    ----------
    angle : float
        True AOA in radians.
    noise_std_deg : float
        Standard deviation of AOA noise in degrees.

    Returns
    -------
    float
        Noisy AOA measurement in radians.
    """

    noise_std_rad = np.deg2rad(noise_std_deg)

    noise = np.random.normal(
        loc=0.0,
        scale=noise_std_rad,
    )

    noisy_angle = angle + noise

    return noisy_angle