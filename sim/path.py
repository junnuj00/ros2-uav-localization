import numpy as np


def generate_uav_path(
    start=(0.0, 5.0),
    velocity_x=2.0,
    curve_rate=0.04,
    duration=20.0,
    dt=0.1,
):
    """
    Generate a curved 2D UAV path.

    x(t) = x0 + velocity_x * t
    y(t) = y0 + curve_rate * t^2

    Parameters
    ----------
    start : tuple
        Initial UAV position (x0, y0).

    velocity_x : float
        UAV velocity in the x direction.

    curve_rate : float
        Curvature coefficient.

    duration : float
        Total simulation time in seconds.

    dt : float
        Simulation time step.

    Returns
    -------
    time : np.ndarray
        Simulation timestamps.

    positions : np.ndarray
        UAV positions with shape (N, 2).
    """

    time = np.arange(
        0.0,
        duration + dt,
        dt,
    )

    x0, y0 = start

    x = (
        x0
        + velocity_x * time
    )

    y = (
        y0
        + curve_rate * time**2
    )

    positions = np.column_stack(
        (x, y)
    )

    return time, positions