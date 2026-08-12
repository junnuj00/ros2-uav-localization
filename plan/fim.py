import numpy as np


def aoa_jacobian(
    uav_position,
    emitter_position,
):
    """
    Compute the Jacobian of the AOA measurement model.
    """

    uav_x, uav_y = uav_position
    emitter_x, emitter_y = emitter_position

    dx = emitter_x - uav_x
    dy = emitter_y - uav_y

    distance_squared = (
        dx ** 2
        + dy ** 2
    )

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


def fisher_information(
    uav_position,
    emitter_position,
    aoa_noise_std_deg=2.0,
):
    """
    Compute Fisher Information Matrix
    for one AOA measurement.
    """

    H = aoa_jacobian(
        uav_position,
        emitter_position,
    )

    noise_std_rad = np.deg2rad(
        aoa_noise_std_deg
    )

    R = noise_std_rad ** 2

    J = (
        H.T
        @ H
    ) / R

    return J


def cumulative_fim(
    uav_positions,
    emitter_position,
    aoa_noise_std_deg=2.0,
):
    """
    Compute cumulative Fisher Information Matrix
    from multiple UAV measurement positions.
    """

    total_fim = np.zeros(
        (2, 2)
    )

    for uav_position in uav_positions:

        J = fisher_information(
            uav_position,
            emitter_position,
            aoa_noise_std_deg,
        )

        total_fim += J

    return total_fim


def fim_score(
    current_positions,
    candidate_position,
    emitter_position,
    aoa_noise_std_deg=2.0,
):
    """
    Evaluate a candidate UAV waypoint
    using determinant of total FIM.
    """

    current_fim = cumulative_fim(
        current_positions,
        emitter_position,
        aoa_noise_std_deg,
    )

    candidate_fim = fisher_information(
        candidate_position,
        emitter_position,
        aoa_noise_std_deg,
    )

    total_fim = (
        current_fim
        + candidate_fim
    )

    score = np.linalg.det(
        total_fim
    )

    return score


def crlb_covariance(
    uav_positions,
    emitter_position,
    aoa_noise_std_deg=2.0,
):
    """
    Calculate CRLB covariance matrix.

    CRLB = inverse(FIM)
    """

    fim = cumulative_fim(
        uav_positions,
        emitter_position,
        aoa_noise_std_deg,
    )

    # Pseudo-inverse is used for numerical robustness.
    crlb = np.linalg.pinv(
        fim
    )

    return crlb


def crlb_position_bound(
    uav_positions,
    emitter_position,
    aoa_noise_std_deg=2.0,
):
    """
    Calculate scalar position uncertainty bound.

    Position bound:
        sqrt(trace(CRLB))

    Unit:
        meters
    """

    crlb = crlb_covariance(
        uav_positions,
        emitter_position,
        aoa_noise_std_deg,
    )

    position_bound = np.sqrt(
        np.trace(
            crlb
        )
    )

    return position_bound