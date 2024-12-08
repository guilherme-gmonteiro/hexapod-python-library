from .constants import POSITION_NAMES_LIST, POSITION_NAME_TO_ID_MAP
from .geometry import matrix_to_align_vector_a_to_b, t_rot_z_matrix
from .vector import Vector
from .hexagon import Hexagon
from .linkage import Linkage

from .solvers.orient.orientSolverGeneral import *
from .solvers.orient.orientSolverSpecific import *

from .solvers.twistSolver import simple_twist, might_twist, complex_twist

DEFAULT_POSE = {
    "leftFront": {"alpha": 0, "beta": 0, "gamma": 0},
    "rightFront": {"alpha": 0, "beta": 0, "gamma": 0},
    "leftMiddle": {"alpha": 0, "beta": 0, "gamma": 0},
    "rightMiddle": {"alpha": 0, "beta": 0, "gamma": 0},
    "leftBack": {"alpha": 0, "beta": 0, "gamma": 0},
    "rightBack": {"alpha": 0, "beta": 0, "gamma": 0},
}

DEFAULT_LOCAL_AXES = {
    "xAxis": Vector(1, 0, 0, "hexapodXaxis"),
    "yAxis": Vector(0, 1, 0, "hexapodYaxis"),
    "zAxis": Vector(0, 0, 1, "hexapodZaxis"),
}

def transform_local_axes(local_axes, twist_matrix):
    return {
        "xAxis": local_axes["xAxis"].clone_trot(twist_matrix),
        "yAxis": local_axes["yAxis"].clone_trot(twist_matrix),
        "zAxis": local_axes["zAxis"].clone_trot(twist_matrix),
    }

def build_legs_list(body_contact_points, pose, leg_dimensions):
    return [
        Linkage(leg_dimensions, position, body_contact_points[i], pose[position])
        for i, position in enumerate(POSITION_NAMES_LIST)
    ]

def hexapod_error_info():
    return {
        "isAlert": True,
        "subject": "Unstable position.",
        "body": "error in solving for orientation ",
    }

def hexapod_success_info():
    return {
        "isAlert": False,
        "subject": "Success!",
        "body": "Stable orientation found.",
    }

"""
/* * *

............................
    Virtual Hexapod properties
............................

Property types:
{}: hash map / object / dictionary
[]: array / list
##: number
"": string

{} this.dimensions: {front, side, middle, coxia, femur, tibia}

{} this.pose: A hash mapping the position name to a hash map of three angles
    which define the pose of the hexapod
    i.e.
    {
        rightMiddle: {alpha, beta, gamma },
        leftBack: { alpha, beta, gamma },
        ...
    }

[] this.body: A hexagon object
    which contains all the info of the 8 points defining the hexapod body
    (6 vertices, 1 head, 1 center of gravity)

[] this.legs: A list which has elements that point to six Linkage objects.
    The order goes counter clockwise starting from the first element
    which is the rightMiddle leg up until the last element which is rightBack leg.
    Each leg contains the points that define that leg
    as well as other properties pertaining it (see Linkage class)

[] this.legPositionsOnGround: A list of the leg positions (strings)
    that are known to be in contact with the ground

{} this.localAxes: A hash containing three vectors defining the local
    coordinate frame of the hexapod wrt the world coordinate frame
    i.e. {
        xAxis: {x, y, z, name="hexapodXaxis", id="no-id"},
        yAxis: {x, y, z, name="hexapodYaxis", id="no-id"},
        zAxis: {x, y, z, name="hexapodZaxis", id="no-id"},
    }

....................
(virtual hexapod derived properties)
....................

{} this.bodyDimensions: { front, side, middle }
{} this.legDimensions: { coxia, femur, tibia }

## this.distanceFromGround: A number which is the perpendicular distance
    from the hexapod's center of gravity to the ground plane

{} this.cogProjection: a point that represents the projection
    of the hexapod's center of gravity point to the ground plane
    i.e { x, y, z, name="centerOfGravityProjectionPoint", id="no-id"}

[] this.groundContactPoints: a list whose elements point to points
    from the leg which contacts the ground.
    This list can contain 6 or less elements.
    (It can have a length of 3, 4, 5 or 6)
    i.e. [
        { x, y, z, name="rightMiddle-femurPoint", id="0-2"},
        { x, y, z, name="leftBack-footTipPoint", id=4-3},
        ...
    ]

 * * */
"""
class VirtualHexapod:
    def __init__(self, dimensions, pose, flags=None):
        if flags is None:
            flags = {"hasNoPoints": False, "assumeKnownGroundPoints": False, "wontRotate": False}
        
        self.dimensions = dimensions
        self.pose = pose
        self.flags = flags

        if self.flags["hasNoPoints"]:
            return
        
        """
        
        // .................
        // STEP 1: Build a flatHexagon and 'dangling' linkages
        // then find  properties we can derive from this
        // .................

        """
        flat_hexagon = Hexagon(self.body_dimensions)

        """
        // legsNoGravity are linkages have the correct pose but
        // are not necessarily correctly oriented wrt the world
        // prettier-ignore
        """
        legs_no_gravity = build_legs_list(
            flat_hexagon.vertices_list, self.pose, self.leg_dimensions
        )

        """
        // `solved` has:
        // - new orientation of the body (nAxis)
        // - which legs are on the ground (groundLegsNoGravity)
        // - distance of center of gravity to the ground (height)

        """

        if self.flags["assumeKnownGroundPoints"]:
            solved = compute_orientation_properties(legs_no_gravity)
        else:
            solved = compute_orientation_properties(legs_no_gravity)

        if solved is None:
            self.found_solution = False
            return

        self.found_solution = True
        self.leg_positions_on_ground = [leg.position for leg in solved["groundLegsNoGravity"]]

        """
        
        // .................
        // STEP 2: Rotate and shift legs and body given what we've solved
        // .................

        """
        transform_matrix = matrix_to_align_vector_a_to_b(solved["nAxis"], DEFAULT_LOCAL_AXES["zAxis"])
        self.legs = [leg.clone_trot_shift(transform_matrix, 0, 0, solved["height"]) for leg in legs_no_gravity]
        self.body = flat_hexagon.clone_trot_shift(transform_matrix, 0, 0, solved["height"])
        self.local_axes = transform_local_axes(DEFAULT_LOCAL_AXES, transform_matrix)

        """
        // .................
        // STEP 3: Twist around the zAxis if you have to
        // .................
        """
        if self.flags["wontRotate"]:
            return
        # case 1: hexapod will not twist about z axis

        if all(leg.pose["alpha"] == 0 for leg in self.legs):
            return
        # case 2: When all alpha angles are the same for all legs
        twist_angle = simple_twist(solved["groundLegsNoGravity"])
        if twist_angle != 0:
            self._twist(twist_angle)
            return
        #case 3: All other cases
        if might_twist(solved["groundLegsNoGravity"]):
            self._handle_complex_twist(flat_hexagon.vertices_list)

    @property
    def distance_from_ground(self):
        return self.body.cog.z

    @property
    def cog_projection(self):
        return Vector(self.body.cog.x, self.body.cog.y, 0, "centerOfGravityProjectionPoint")

    @property
    def info(self):
        return hexapod_success_info() if self.found_solution else hexapod_error_info()

    @property
    def body_dimensions(self):
        return self.dimensions["front"], self.dimensions["middle"], self.dimensions["side"]

    @property
    def leg_dimensions(self):
        return self.dimensions["coxia"], self.dimensions["femur"], self.dimensions["tibia"]

    @property
    def ground_contact_points(self):
        return [
            self.legs[POSITION_NAME_TO_ID_MAP[position]].maybe_ground_contact_point
            for position in self.leg_positions_on_ground
        ]

    def clone_trot(self, transform_matrix):
        #Note: transform matrix passed should be purely rotational
        body = self.body.clone_trot(transform_matrix)
        legs = [leg.clone_trot(transform_matrix) for leg in self.legs]
        local_axes = transform_local_axes(self.local_axes, transform_matrix)
        return self._build_clone(body, legs, local_axes)

    def clone_shift(self, tx, ty, tz):
        body = self.body.clone_shift(tx, ty, tz)
        legs = [leg.clone_shift(tx, ty, tz) for leg in self.legs]
        return self._build_clone(body, legs, self.local_axes)

    def _build_clone(self, body, legs, local_axes):
        """
         FIXME:
        // After shifting and/or rotating the hexapod
        // We can no longer guarrantee that the legPositionsOnGround
        // is the same as before
        // must handle this soon!!
        """
        clone = VirtualHexapod(self.dimensions, self.pose, {"hasNoPoints": True})
        clone.body = body
        clone.legs = legs
        clone.local_axes = local_axes
        clone.leg_positions_on_ground = self.leg_positions_on_ground
        clone.found_solution = self.found_solution
        return clone

    def _handle_complex_twist(self, vertices_list):
        """
         // DefaultLegs: The list of legs when a hexapod
        // of these dimensions is at the default pose
        // (ie all angles are zero)
        // DefaultPoints: the corresponding ground contact
        // points of defaultLegs       
        """
        default_legs = build_legs_list(vertices_list, DEFAULT_POSE, self.leg_dimensions)
        default_points = [leg.clone_shift(0, 0, self.dimensions["tibia"]).maybe_ground_contact_point for leg in default_legs]
        #currentPoints: Where the ground contact points are currently
        # given all the transformations we have done so far
        current_points = self.ground_contact_points
        twist_angle = complex_twist(current_points, default_points)

        if twist_angle != 0:
            self._twist(twist_angle)

    def _twist(self, twist_angle):
        twist_matrix = t_rot_z_matrix(twist_angle)
        self.body = self.body.clone_trot(twist_matrix)
        self.legs = [leg.clone_trot(twist_matrix) for leg in self.legs]
        self.local_axes = transform_local_axes(self.local_axes, twist_matrix)
