from .linkageIKSolver import LinkageIKSolver
from .hexapodSupportCheck import HexapodSupportCheck
from .IKInfo import IKMessage
from ...constants import (
    POSITION_NAMES_LIST,
    NUMBER_OF_LEGS,
    POSITION_NAME_TO_AXIS_ANGLE_MAP,
    MAX_ANGLES,
)
from ...geometry import (
    vector_from_to,
    projected_vector_onto_plane,
    get_unit_vector,
    scale_vector,
    add_vectors,
    angle_between,
    vector_length,
    is_counter_clockwise,
)

"""
* * *

Given:
.......
Dimensions of each leg { femur, tibia, gamma }
[] bodyContactPoints
[] groundContactPoints
   - there are two lists which contains six items each. One item for each leg.
{} axes { xAxis, zAxis }
   xAxis and zAxis of the rotated hexapod's local coordinate frame wrt
   world coordinate frame.

.......
Definition:
.......

bodyContactPoint (x, y, z)
- The point in 3d space which is a vertex of the hexagon.
  This is where the leg is in contact with the body of the hexapod.

groundContactPoint (x, y, z)
- The point in 3d space which we want the foot tip of
  the leg to be. Where the leg is in contact with the ground plane of the world.

.......
Find:
.......

18 angles that represent the pose of the hexapod. Three angles for each leg.
    {
      rightMiddle: { alpha, beta, gamma },
      ...
      rightBack: { alpha, beta, gamma },
    }

    If no solution is found, make sure to explain why.

.......
Algorithm:
.......

If one of the vertices is below the ground z < 0,
then there is no solution. Early exit.

For each leg:
    1. Derive a few properties about the leg given what you already know
       which you'd later (see computeInitialProperties() for details )

       This includes the coxiaPoint. If this coxiaPoint is below the ground
        - then there is no solution. Early exit.

    2. Compute the alpha of this leg. see (computeAlpha())
       If alpha is not within range, then there is no solution. Early exit.

    3. Solve for beta and gamma of this leg (see LegIKSolver module)
      If a problem was encountered within this module, then there is no solution. Early exit.
      If the beta and gamma are not within range, then there is no solution, early exit.

    4. Sometimes the LegIKSolver module would return a solution where the leg
       would not reach the target ground contact point. (this leg would be on the air)
       If the combination of the legs in the air would produce an unstable pose
       (e.g 4 legs are in the air or all left legs are in the air)
       Then there is no solution. Early exit.
       (see also HexapodSupportChecker)

    If no problems are encountered, we have found a solution! Return!

* * *
"""


class IKSolver:
    def __init__(self):
        self.params = {}
        self.partial_pose = {}
        self.pose = {}
        self.found_solution = False
        self.leg_positions_off_ground = []
        self.message = IKMessage.initialized

    def solve(self, leg_dimensions, body_contact_points, ground_contact_points, axes):
        self.params = {
            "body_contact_points": body_contact_points,
            "ground_contact_points": ground_contact_points,
            "axes": axes,
            "leg_dimensions": leg_dimensions,
        }

        if self._has_bad_vertex(body_contact_points):
            return self

        coxia, femur, tibia = (
            leg_dimensions["coxia"],
            leg_dimensions["femur"],
            leg_dimensions["tibia"],
        )

        for i in range(NUMBER_OF_LEGS):
            leg_position = POSITION_NAMES_LIST[i]

            known = compute_initial_leg_properties(
                body_contact_points[i], ground_contact_points[i], axes["z_axis"], coxia
            )

            if known["coxia_point"].z < 0:
                self._handle_bad_point(known["coxia_point"])
                return self

            leg_x_axis_angle = POSITION_NAME_TO_AXIS_ANGLE_MAP[leg_position]

            alpha = compute_alpha(
                known["coxia_unit_vector"],
                leg_x_axis_angle,
                axes["x_axis"],
                axes["z_axis"],
            )

            if abs(alpha) > MAX_ANGLES["alpha"]:
                self._finalize_failure(
                    IKMessage.alpha_not_in_range(
                        leg_position, alpha, MAX_ANGLES["alpha"]
                    )
                )
                return self

            solved_leg_params = LinkageIKSolver(leg_position).solve(
                coxia, femur, tibia, known["summa"], known["rho"]
            )

            if not solved_leg_params.obtained_solution:
                self._finalize_failure(IKMessage.bad_leg(solved_leg_params.message))
                return self

            if not solved_leg_params.reached_target:
                if self._has_no_more_support(leg_position):
                    return self

            self.partial_pose[leg_position] = {
                "alpha": alpha,
                "beta": solved_leg_params.beta,
                "gamma": solved_leg_params.gamma,
            }

        self._finalize_success()
        return self

    @property
    def has_legs_off_ground(self):
        return bool(self.leg_positions_off_ground)

    def _has_no_more_support(self, leg_position):
        self.leg_positions_off_ground.append(leg_position)
        no_support, reason = HexapodSupportCheck.check_support(
            self.leg_positions_off_ground
        )
        if no_support:
            message = IKMessage.no_support(reason, self.leg_positions_off_ground)
            self._finalize_failure(message)
            return True
        return False

    def _handle_bad_point(self, point):
        self._finalize_failure(IKMessage.bad_point(point))

    def _has_bad_vertex(self, body_contact_points):
        for vertex in body_contact_points:
            if vertex.z < 0:
                self._handle_bad_point(vertex)
                return True
        return False

    def _finalize_failure(self, message):
        self.message = message
        self.found_solution = False

    def _finalize_success(self):
        self.pose = self.partial_pose
        self.found_solution = True
        if not self.has_legs_off_ground:
            self.message = IKMessage.success
        else:
            self.message = IKMessage.success_legs_on_air(self.leg_positions_off_ground)

"""
* * *

computeInitialLegProperties()

.......
Given:
.......

1. pB : bodyContactPoint in 3d space
2. pG : groundContactPoint in 3d space
3. coxia: distance from pB to pC
4. zAxis: The vector normal to the hexapodBodyPlane

.......
Find:
.......

1. pC : coxiaPoint in 3d space
2. coxiaVector: the vector from pB to Pc with a length of one
3. coxiaUnitVector: A vector with the length of one
    pointing at the direction of the unit vector
4. rho: The angle made by pC, pB and pG, with pB at the center
5. summa: The distance from pB to pG

pB   pC
 *---* -------- hexapodBodyPlane
  \   \
   \   *
    \  /
      * ------- groundPlane
      pG

.......
Idea:
.......

1. Get the vector from pB to pG (bodyToFootVector)
2. Project that vector to the hexapodBodyPlane (coxiaDirectionVector)
   The direction of this vector is the direction of
   coxiaVector and coxiaUnitVector

   And with a little bit of geometry you derive verything you need.

* * *
"""

def compute_initial_leg_properties(body_contact_point, ground_contact_point, z_axis, coxia):
    body_to_foot_vector = vector_from_to(body_contact_point, ground_contact_point)
    coxia_direction_vector = projected_vector_onto_plane(body_to_foot_vector, z_axis)
    coxia_unit_vector = get_unit_vector(coxia_direction_vector)
    coxia_vector = scale_vector(coxia_unit_vector, coxia)
    coxia_point = add_vectors(body_contact_point, coxia_vector)
    rho = angle_between(coxia_unit_vector, body_to_foot_vector)
    summa = vector_length(body_to_foot_vector)

    return {
        "coxia_unit_vector": coxia_unit_vector,
        "coxia_vector": coxia_vector,
        "coxia_point": coxia_point,
        "rho": rho,
        "summa": summa,
    }

"""
* * *

computeAlpha()

  hexapodYaxis
  ^
  |
  * --> hexapodXaxis (xAxis)
 /
hexapodZaxis (zAxis)

...............
Example #1 :
...............

             coxiaVector
              ^
              | legXaxis
              |  /        * legXaxisAngle
              | /            - Angle between legXaxis and hexapodXaxis
    * -- * -- *                (in this example: +45 degrees )
   /           \
  /             \          * Alpha
 *       *       *            - Angle between legXaxis and coxiaVector
  \             /                (in this example: +45 degrees)
   \           /
    * -- * -- *

...............
Example #2
............

    * -- * -- *       * legXaxisAngle
   /           \          - (in this example: -45 degrees or +315 degrees)
  /             \      * Alpha
 *       *       *         - (in this example: -45 degrees)
  \             /
   \           /
    * -- * -- *
              |\              - (in this example: +45 degrees )
              | \
              | legXaxis
              V
              coxiaVector
* * *
"""
def compute_alpha(coxia_vector, leg_x_axis_angle, x_axis, z_axis):
    sign = -1 if is_counter_clockwise(coxia_vector, x_axis, z_axis) else 1
    alpha_wrt_hexapod = sign * angle_between(coxia_vector, x_axis)
    alpha = (alpha_wrt_hexapod - leg_x_axis_angle) % 360

    if alpha > 180:
        return alpha - 360
    if alpha < -180:
        return alpha + 360

    # ❗❗❗THIS IS A HACK ❗❗❗
    # THERE IS A BUG HERE SOMEWHERE, FIND IT
    if alpha in (180, -180):
        return 0

    return alpha
