from geometry import dot, get_normal_of_three_points
from orientSolverHelpers import (
    SOME_LEG_ID_TRIOS,
    ADJACENT_LEG_ID_TRIOS,
    is_stable,
    find_legs_on_ground,
    is_lower,
)

LEG_ID_TRIOS = SOME_LEG_ID_TRIOS + ADJACENT_LEG_ID_TRIOS

def compute_orientation_properties(legs_no_gravity):
    """
    Computes orientation properties of the hexapod.

    Args:
        legs_no_gravity: List of legs with their pose known, relative to the hexapod body.

    Returns:
        A dictionary with:
            - n_axis: Normal vector of the plane defined by the foot tips of legs on the ground.
            - height: Distance from the hexapod body plane to the plane defined by the foot tips.
            - ground_legs_no_gravity: Legs in contact with the ground.
        Or None if no valid orientation is found.
    """
    result = compute_plane_properties(legs_no_gravity)
    if result is None:
        return None

    ground_legs_no_gravity = find_legs_on_ground(
        legs_no_gravity, result["normal"], result["height"]
    )

    return {
        "n_axis": result["normal"],
        "height": result["height"],
        "ground_legs_no_gravity": ground_legs_no_gravity,
    }


def compute_plane_properties(legs):
    """
    Computes the plane properties defined by the foot tips of the legs.

    Args:
        legs: List of legs with a `maybe_ground_contact_point` attribute.

    Returns:
        A dictionary with:
            - normal: Normal vector of the plane.
            - height: Distance from the hexapod body plane to the plane.
        Or None if no valid plane is found.
    """
    maybe_ground_contact_points = [leg.maybe_ground_contact_point for leg in legs]

    for leg_trio in LEG_ID_TRIOS:
        p0, p1, p2 = (maybe_ground_contact_points[j] for j in leg_trio)

        if not is_stable(p0, p1, p2):
            continue

        normal = get_normal_of_three_points(p0, p1, p2, "normal_vector")

        # Compute the height using p0 (should be the same for p1 or p2)
        height = -dot(normal, p0)

        # Check if no other leg's foot tip is lower than the plane
        other_trio = [j for j in range(6) if j not in leg_trio]
        other_foot_tips = [maybe_ground_contact_points[j] for j in other_trio]

        no_other_leg_lower = all(
            not is_lower(foot_tip, normal, height) for foot_tip in other_foot_tips
        )

        if no_other_leg_lower:
            return {"normal": normal, "height": height}

    return None