import numpy as np

from plan.fim import fim_score


def generate_candidates(
    current_position,
    radius=5.0,
    num_candidates=8,
):
    """
    Generate candidate UAV waypoints
    around the current position.
    """

    angles = np.linspace(
        0.0,
        2.0 * np.pi,
        num_candidates,
        endpoint=False,
    )

    candidates = []

    for angle in angles:

        x = (
            current_position[0]
            + radius * np.cos(angle)
        )

        y = (
            current_position[1]
            + radius * np.sin(angle)
        )

        candidates.append(
            [x, y]
        )

    return np.array(
        candidates
    )


def select_best_waypoint(
    current_positions,
    current_position,
    emitter_estimate,
    radius=5.0,
    num_candidates=8,
    aoa_noise_std_deg=2.0,
):
    """
    Select the candidate waypoint
    that maximizes the determinant
    of the cumulative FIM.
    """

    candidates = generate_candidates(
        current_position=current_position,
        radius=radius,
        num_candidates=num_candidates,
    )

    scores = []

    for candidate in candidates:

        score = fim_score(
            current_positions=current_positions,
            candidate_position=candidate,
            emitter_position=emitter_estimate,
            aoa_noise_std_deg=aoa_noise_std_deg,
        )

        scores.append(
            score
        )

    scores = np.array(
        scores
    )

    best_index = np.argmax(
        scores
    )

    best_waypoint = candidates[
        best_index
    ]

    best_score = scores[
        best_index
    ]

    return (
        best_waypoint,
        best_score,
        candidates,
        scores,
    )