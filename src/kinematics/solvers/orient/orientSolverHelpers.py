from vector import Vector
from geometry import dot, cross, vector_from_to

def is_lower(point, normal, height, tol=1):
    return -dot(normal, point) > height + tol

def same_height(point, normal, height, tol=1):
    _height = -dot(normal, point)
    return abs(height - _height) <= tol

def find_legs_on_ground(legs, normal, height):
    """
    Determine which legs are on the ground.

    Each leg has four points: footTip, femurPoint, coxiaPoint, bodyContact.
    Starting from footTip to bodyContact, check if one point of the leg
    is at the same height as the ground.

    Args:
        legs: List of leg objects, each with an all_points_list attribute.
        normal: Normal vector of the ground plane.
        height: Height of the hexapod's center of gravity to the ground plane.

    Returns:
        List of legs that are on the ground.
    """
    legs_on_ground = []
    for leg in legs:
        reversed_points = list(reversed(leg.all_points_list[1:]))  # Skip bodyContact
        on_ground = any(same_height(point, normal, height) for point in reversed_points)
        if on_ground:
            legs_on_ground.append(leg)
    return legs_on_ground

SOME_LEG_ID_TRIOS = [
    [0, 1, 3], [0, 1, 4], [0, 2, 3], [0, 2, 4], [0, 2, 5],
    [0, 3, 4], [0, 3, 5], [1, 2, 4], [1, 2, 5], [1, 3, 4],
    [1, 3, 5], [1, 4, 5], [2, 3, 5], [2, 4, 5],
]

ADJACENT_LEG_ID_TRIOS = [
    [0, 1, 2], [1, 2, 3], [2, 3, 4],
    [3, 4, 5], [0, 4, 5], [0, 1, 5],
]

def is_stable(p0, p1, p2, tol=0.001):
    """
    Determine if the hexapod's center of gravity (COG) is stable.

    Stability is defined as the projection of the COG onto the plane
    defined by p0, p1, p2 being inside the triangle formed by these points.

    Args:
        p0, p1, p2: Points defining the triangle.
        tol: Tolerance for determining if the projection is within the triangle.

    Returns:
        True if stable, False otherwise.
    """
    cog = Vector(0, 0, 0)

    u = vector_from_to(p0, p1)
    v = vector_from_to(p0, p2)
    w = vector_from_to(p0, cog)
    n = cross(u, v)
    n2 = dot(n, n)

    beta = dot(cross(u, w), n) / n2
    gamma = dot(cross(w, v), n) / n2
    alpha = 1 - beta - gamma

    min_val = -tol
    max_val = 1 + tol

    cond0 = min_val <= alpha <= max_val
    cond1 = min_val <= beta <= max_val
    cond2 = min_val <= gamma <= max_val

    return cond0 and cond1 and cond2