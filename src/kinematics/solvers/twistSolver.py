import math
from ..geometry import degrees
from ..constants import POSITION_NAME_TO_ID_MAP

def might_twist(legs_on_ground):
    """
    mightTwist()
        params: list of legs that are known to touch the ground
        returns: boolean
            false if we are sure it won't twist
            true if there's a possibility that it might twist

    given the starting pose is such that:
        - all alphas are zero
        - all foot tips are touching the ground

    then the hexapod will only twist if:
        - ATLEAST three alphas of the legs touching the ground
        - the point of contact of these legs are foot tips
        - All the twist must be only in one directions
          ie: the threes alphas are either all positive or all negative
          (NOT a mix of both)

    It will definitely not twist if:
        - If only two or less of the legs has an alpha !== 0
        - less than three alphas are twisting towards one direction
    """
    negative_alpha_count = 0
    positive_alpha_count = 0

    for leg in legs_on_ground:
        point_type = leg.maybe_ground_contact_point.name.split("-")[1]

        foot_tip_is_on_ground = point_type == "footTipPoint"
        changed_alpha = leg.pose.alpha != 0

        if foot_tip_is_on_ground and changed_alpha:
            if leg.pose.alpha > 0:
                positive_alpha_count += 1
            else:
                negative_alpha_count += 1

    return positive_alpha_count >= 3 or negative_alpha_count >= 3


def complex_twist(current_points, default_points):
    """
    complexTwist()

    Params:
        defaultPoints:
            This is the list of the six points which are on the ground
            when a hexapod of the same dimensions has the default pose
            (all angles == 0 ), because it is that pose all of these points
            are of type footTipPoint

        currentPoints:
            This are the list of points (NOT necessarily six and
            NOT necessarily footTipPoints) that will be on the ground
            when a hexapod of this dimensions is on a specified pose
            BUT is NOT rotated twisted along the zAxis

    Algorithm:
        Find a point that is on the ground at
        the current pose, and at the default pose.
        (samePointPosition)

        Find the angle to align that currentPoint
        to the defaultPoint.

        (this means that the hexapod would have twisted about its zAxis
        so that the point in the ground in this legPosition
        is the same before (default pose) and after
        (current pose) moving to the current pose)
    """
    current_same_point = next(
        (point for point in current_points if point.name.split("-")[1] == "footTipPoint"),
        None,
    )

    if current_same_point is None:
        return 0

    same_point_position = current_same_point.name.split("-")[0]
    same_point_index = POSITION_NAME_TO_ID_MAP[same_point_position]
    default_same_point = default_points[same_point_index]

    theta_radians = (
        math.atan2(default_same_point.y, default_same_point.x)
        - math.atan2(default_same_point.y, default_same_point.x)
    )

    return degrees(theta_radians)


def simple_twist(ground_legs_no_gravity):
    """
    simpleTwist()

    We twist in the condition that:
      - All the legs pose has same alpha
      - the ground contact points are either all femurPoints or all footTipPoints
       if all femurPoints on ground, make sure bodyContactPoint.z != femurPoint.z
        (ie  if hexapod body is not on the ground we should not twist)
    """
    first_leg = ground_legs_no_gravity[0]

    all_same_alpha = all(
        leg.pose.alpha == first_leg.pose.alpha for leg in ground_legs_no_gravity
    )

    if not all_same_alpha:
        return 0

    all_point_types = [
        leg.maybe_ground_contact_point.name.split("-")[1]
        for leg in ground_legs_no_gravity
    ]

    first_point_type = all_point_types[0]

    all_points_same_type = all(
        point_type == first_point_type for point_type in all_point_types
    )

    if not all_points_same_type:
        return 0

    # At this point, all ground points are of the same type
    if first_point_type in ["coxiaPoint", "bodyContactPoint"]:
        return 0

    # At this point, all ground points are either ALL femurPoint or ALL footTipPoint
    if first_point_type == "femurPoint":
        hexapod_body_plane_on_ground = (
            first_leg.body_contact_point.z == first_leg.femur_point.z
        )
        if hexapod_body_plane_on_ground:
            return 0

    return -first_leg.pose.alpha
