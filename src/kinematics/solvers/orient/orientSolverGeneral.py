"""
/* * *

A more general algorithm to account for the cases
that are not handled correctly by the orientSolverSpecific.

- Only used by the kinematics-page of the app.
- Can be optimized or replaced if a more elegant
algorithm is available.

............
    OVERVIEW
............

Find:
- The orientation of the hexapod (normal axis of the hexapod body plane)
- distance of hexapod center of gravity to the ground plane (height)
- All the legs which are in contact in the ground

How?

We have 18 points total.
(6 legs, three possible points per leg (femurPoint))

We have a total of 540 combinations
- get three legs out of six (20 combinations)
    - we have three possible points for each leg,
        (coxiaPoint, femurPoint, footTip),
        that's 27 (3^3) combinations
    -  27 * 20 is 540

For each combination:
    1. Check if stable. If not, try the next combination
        - Check if the projection of the center of gravity to the plane
            defined by the three points lies inside the triangle,
            if not stable, try the next combination

    2. Get the HEIGHT and normal of the height and normal of the triangle plane
        (We need this for the next part)

    3. For each of the three legs, check if the two other points on the leg is not
        lower than HEIGHT, (6 points total)
        if condition if broken, try the next combination.

    4. For each of the three other legs, check if all points (3 points of each leg)
        are not lower than HEIGHT
        if this condition is broken, try the next combination. (9 points total)

    5. If no condition is violated, then this is good, return this!

 * * */
"""
from itertools import product
from random import shuffle
from geometry import dot, get_normal_of_three_points
from orientSolverHelpers import SOME_LEG_ID_TRIOS, ADJACENT_LEG_ID_TRIOS, is_stable, is_lower, find_legs_on_ground

# Helper to create all joint index trios
def make_joint_index_trios():
    return list(product(range(3), repeat=3))

JOINT_INDEX_TRIOS = make_joint_index_trios()

# Shuffles an array in place
def shuffle_array(array):
    shuffle(array)
    return array

# Main function to compute orientation properties
def compute_orientation_properties(legs_no_gravity, flags=None):
    flags = flags or {}
    some_leg_trios = (
        shuffle_array(SOME_LEG_ID_TRIOS.copy())
        if flags.get("shuffle", False)
        else SOME_LEG_ID_TRIOS
    )

    leg_index_trios = some_leg_trios + ADJACENT_LEG_ID_TRIOS
    fallback = None

    for three_leg_indices in leg_index_trios:
        three_legs, other_three_legs = get_two_leg_sets(three_leg_indices, legs_no_gravity)

        for three_joint_indices in JOINT_INDEX_TRIOS:
            p0, p1, p2 = get_three_points(three_legs, three_joint_indices)

            if not is_stable(p0, p1, p2):
                continue

            normal = get_normal_of_three_points(p0, p1, p2, "normal_vector")
            height = -dot(normal, p0)

            if another_point_of_same_leg_is_lower(three_legs, three_joint_indices, normal, height):
                continue

            if another_point_of_other_legs_is_lower(other_three_legs, normal, height):
                continue

            # Hack fallback for height == 0
            if height == 0:
                if fallback is None:
                    fallback = {"p0": p0, "p1": p1, "p2": p2, "normal": normal, "height": height}
                continue

            ground_legs_no_gravity = find_legs_on_ground(legs_no_gravity, normal, height)
            return {"n_axis": normal, "height": height, "ground_legs_no_gravity": ground_legs_no_gravity}

    if fallback is None:
        return None

    return {
        "n_axis": fallback["normal"],
        "height": fallback["height"],
        "ground_legs_no_gravity": find_legs_on_ground(
            legs_no_gravity, fallback["normal"], fallback["height"]
        ),
    }

# Extract three points from legs and joint indices
def get_three_points(three_legs, three_joint_indices):
    return [leg["all_points_list"][joint_id] for leg, joint_id in zip(three_legs, three_joint_indices)]

# Split six legs into two sets
def get_two_leg_sets(three_leg_indices, six_legs):
    three_legs = [six_legs[n] for n in three_leg_indices]
    other_three_leg_indices = [n for n in range(6) if n not in three_leg_indices]
    other_three_legs = [six_legs[n] for n in other_three_leg_indices]
    return three_legs, other_three_legs

# Check if any other point of the same leg is lower
def another_point_of_same_leg_is_lower(three_legs, three_joint_indices, normal, height):
    for leg, joint_index in zip(three_legs, three_joint_indices):
        has_lower = any(
            is_lower(point, normal, height)
            for idx, point in enumerate(leg["all_points_list"])
            if idx != 0 and idx != joint_index
        )
        if has_lower:
            return True
    return False

# Check if any point of other legs is lower
def another_point_of_other_legs_is_lower(other_three_legs, normal, height):
    for leg in other_three_legs:
        has_lower = any(
            is_lower(point, normal, height) for point in leg["all_points_list"][1:]
        )
        if has_lower:
            return True
    return False