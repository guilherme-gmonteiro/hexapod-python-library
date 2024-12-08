from ...constants import POSITION_NAMES_LIST
from ...geometry import (
    t_rot_z_matrix,
    t_rot_xyz_matrix,
    vector_from_to,
    angle_between,
    is_counter_clockwise,
)
from ...vector import Vector
from ...virtualhexapod import VirtualHexapod
from .IKSolver import IKSolver

def solve_inverse_kinematics(dimensions, raw_ik_params, flags={"rotateThenShift": True}):
    ik_solver, target_ground_contact_points = solve_hexapod_params(
        dimensions, raw_ik_params, flags["rotateThenShift"]
    )

    if not ik_solver.found_solution:
        return {
            "pose": None,
            "obtainedSolution": False,
            "message": ik_solver.message,
            "hexapod": None,
        }
    # How the hexapod looks like if the center of gravity is at (0, 0, _)
    current_hexapod = VirtualHexapod(dimensions, ik_solver.pose)
    excluded_positions = ik_solver.leg_positions_off_ground

    pivots = find_two_pivot_points(
        current_hexapod.ground_contact_points,
        target_ground_contact_points,
        excluded_positions
    )

    hexapod = (
        rotate_shift_hexapod_given_pivots(
            current_hexapod, pivots["points1"], pivots["points2"]
        )
        if pivots["foundTwoPoints"]
        else current_hexapod
    )

    return {
        "pose": ik_solver.pose,
        "obtainedSolution": True,
        "message": ik_solver.message,
        "hexapod": hexapod,
    }

"""
/* * *
    Returns a two-element array
    1. ikSolver: IKSolver object
    2. An array of target ground contact points
 * * */
"""
def solve_hexapod_params(dimensions, raw_ik_params, rotate_then_shift):
    t_vec, rot_matrix, start_pose = convert_ik_params(dimensions, raw_ik_params)
    start_hexapod = VirtualHexapod(dimensions, start_pose)

    targets = build_hexapod_targets(start_hexapod, rot_matrix, t_vec, {"rotateThenShift": rotate_then_shift})

    ik_solver = IKSolver().solve(
        start_hexapod.leg_dimensions,
        targets["bodyContactPoints"],
        targets["groundContactPoints"],
        targets["axes"]
    )

    return ik_solver, targets["groundContactPoints"]

# Make sure all parameter values are numbers
def raw_params_to_numbers(raw_params):
    return {key: float(val) for key, val in raw_params.items()}

"""
 tx, ty, and tz are within the range of (-1, 1)
 return the actual values we want the hexapod's center of gravity to be at
"""
def convert_from_percent_to_translate_values(tx, ty, tz, middle, side, tibia):
    shift_x = tx * middle
    shift_y = ty * side
    shift_z = tz * tibia
    return Vector(shift_x, shift_y, shift_z)

"""
/* * *

startPose:
    - The pose of the hexapod before we
        rotate and translate the hexapod
    - The body (hexagon) is flat at this point
    - At the very end, we want the hexapod
        to step on the same place as at this pose
        (ie same ground contact points)

 * * */
"""
def build_start_pose(hip_stance, leg_stance):
    beta_and_gamma = {"beta": leg_stance, "gamma": -leg_stance}
    alphas = [0, -hip_stance, hip_stance, 0, -hip_stance, hip_stance]

    return {
        POSITION_NAMES_LIST[index]: {"alpha": alpha, **beta_and_gamma}
        for index, alpha in enumerate(alphas)
    }

"""
/* * *

compute for the following:

startPose:
    - The pose of the hexapod before we
        rotate and translate the hexapod
    - see function buildStartPose() for details

rotateMatrix:
    - The transformation matrix we would use to
        rotate the hexapod's body

tVec
    - The translation vector we would use to
        shift the hexapod's body

 * * */
"""
def convert_ik_params(dimensions, raw_ik_params):
    ik_params = raw_params_to_numbers(raw_ik_params)

    middle, side, tibia = dimensions["middle"], dimensions["side"], dimensions["tibia"]
    tx, ty, tz = ik_params["tx"], ik_params["ty"], ik_params["tz"]

    t_vec = convert_from_percent_to_translate_values(tx, ty, tz, middle, side, tibia)

    hip_stance, leg_stance = ik_params["hipStance"], ik_params["legStance"]
    start_pose = build_start_pose(hip_stance, leg_stance)

    rx, ry, rz = ik_params["rx"], ik_params["ry"], ik_params["rz"]
    rot_matrix = t_rot_xyz_matrix(rx, ry, rz)

    return t_vec, start_pose, rot_matrix


"""
/* * *

compute the parameters required to solve
for the hexapod's inverse kinematics

see IKSolver() class for details.

 * * */
"""
def build_hexapod_targets(hexapod, rot_matrix, t_vec, flags):
    ground_contact_points = [leg.maybe_ground_contact_point for leg in hexapod.legs]

    body_contact_points = (
        hexapod.body.clone_trot(rot_matrix).clone_shift(t_vec.x, t_vec.y, t_vec.z).vertices_list
        if flags["rotateThenShift"]
        else hexapod.body.clone_shift(t_vec.x, t_vec.y, t_vec.z).clone_trot(rot_matrix).vertices_list
    )

    axes = {
        "xAxis": Vector(1, 0, 0).clone_trot(rot_matrix),
        "zAxis": Vector(0, 0, 1).clone_trot(rot_matrix),
    }

    return {"groundContactPoints": ground_contact_points, "bodyContactPoints": body_contact_points, "axes": axes}

"""
/* * *

We know 2 point positions that we know are
foot tip ground contact points
(position ie "rightMiddle" etc)

The given `hexapod` is stepping at the `current` points

We want to return a hexapod that is
shifted and rotated it so that those
two points would be stepping at their
respective `target` points

 * * */
"""
def rotate_shift_hexapod_given_pivots(hexapod, points1, points2):
    target_vector = vector_from_to(points1["target"], points2["target"])
    current_vector = vector_from_to(points1["current"], points2["current"])

    twist_angle_absolute = angle_between(current_vector, target_vector)
    is_ccw = is_counter_clockwise(current_vector, target_vector, Vector(0, 0, 1))
    twist_angle = twist_angle_absolute if is_ccw else -twist_angle_absolute
    twist_matrix = t_rot_z_matrix(twist_angle)

    twisted_current_point1 = points1["current"].clone_trot(twist_matrix)
    translate_vector = vector_from_to(twisted_current_point1, points1["target"])

    return hexapod.clone_trot(twist_matrix).clone_shift(translate_vector.x, translate_vector.y, 0)


"""
/* * *

given the points where the hexapod should step on

Find two foot tips as pivot points
that we can use to shift and twist the current Hexapod

 * * */
"""
def find_two_pivot_points(current_points, target_points, excluded_positions):
    target_points_map = {point.name: point for point in target_points}

    current_point1, current_point2 = None, None
    target_point1, target_point2 = None, None

    for current_point in current_points:
        current_name = current_point.name
        if current_name in excluded_positions:
            continue

        if current_name in target_points_map:
            if current_point1 is None:
                current_point1 = current_point
                target_point1 = target_points_map[current_name]
            else:
                current_point2 = current_point
                target_point2 = target_points_map[current_name]
                break

    if current_point2 is None:
        return {"foundTwoPoints": False}

    return {
        "points1": {"target": target_point1, "current": current_point1},
        "points2": {"target": target_point2, "current": current_point2},
        "foundTwoPoints": True,
    }