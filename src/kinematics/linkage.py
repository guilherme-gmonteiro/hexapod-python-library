import math
from geometry import tRotYmatrix, tRotZmatrix, multiply4x4
from constants import LEG_POINT_TYPES_LIST, POSITION_NAME_TO_ID_MAP, POSITION_NAME_TO_AXIS_ANGLE_MAP
from vector import Vector

class Linkage:
    """
    * * * * *
    ..................
     LINKAGE
    ..................

       p0 *----* p1
                \       * p0 = origin / bodyContactPoint
                 * p2   * p1 = coxiaPoint
                 |      * p2 = femurPoint
                 * p3   * p3 = tibiaPoint / footTipPoint
                        * coxiaVector = vector from p0 to p1
       localZ           * femurVector = vector from p1 to p2
       |  localY        * tibiaVector = vector from p2 to p3
       | /
       |/
       |------ localX   * LegPointId = {legId}-{pointId}

               2           1       * legId - legName     - localXaxisAngle
                \   head  /        *  0    - rightMiddle - 0
                 *---*---*         *  1    - rightFront  - 45
                /    |    \        *  2    - leftFront   - 135
               /     |     \       *  3    - leftMiddle  - 180
           3 -*-----cog-----*- 0   *  4    - leftBack    - 225
               \     |     /       *  5    - rightBack   - 315
                \    |    /
                 *---*---*          ^ hexapodY
                /         \         |
               4           5        *---> hexapodX
                                     /
                                    * hexapodZ

                     * localXaxisAngle = angle made by hexapodXaxis and localXaxis
                     * alpha = angle made by coxia Vector and localXaxis
             p2      * beta = angle made by coxiaVector and femurVector
             *              = angle made by points p2, p1 and pxPrime
            / \
       *---*---\---> pxPrime
      p0   p1   * p3


      p0   p1         * gamma = angle made by vector perpendicular to
       *---*                    coxiaVector and tibiaVector
           | \                = angle made by points pzPrime, p1, p3
           |  \
           V   * p3
          pzPrime

    ..................
     LINKAGE PROPERTIES
    ..................

    {} this.dimensions: { coxia, femur, tibia }
    {} this.pose: { alpha, beta, gamma }
    "" this.position: "rightMiddle" from POSITION_NAMES_LIST or "linkage-position-not-defined"

    [] this.allPointsList: A list pointing to each of the four points in the map
        which the first element being the bodyContactPoint, the last element being the footTipPoint

        [
            {x, y, z, id: "5-0", name: "rightBack-bodyContactPoint"},
            {x, y, z, id: "5-1", name: "rightBack-coxiaPoint"},
            {x, y, z, id: "5-2", name: "rightBack-femurPoint"},
            {x, y, z, id: "5-3", name: "rightBack-footTipPoint"},
        ]
        each id is prefixed with 5 because the leg point id corresponding to "rightBack"
        position is 5.

    ....................
    (linkage derived properties)
    ....................

    {} this.maybeGroundContactPoint: The point which probably is the one in contact
        with the ground, but not necessarily the case (no guarantees)
    "" this.name: "{position}Leg" e.g. "rightMiddleLeg"
    "" this.id : a number from 0 to 5 corresponding to a particular position

    * * * * *
    """
    def __init__(self, dimensions, position, origin_point=None, pose=None, flags=None):
        if origin_point is None:
            origin_point = {"x": 0, "y": 0, "z": 0}
        if pose is None:
            pose = {"alpha": 0, "beta": 0, "gamma": 0}
        if flags is None:
            flags = {"hasNoPoints": False}

        self.dimensions = dimensions
        self.pose = pose
        self.position = position

        if flags.get("hasNoPoints"):
            return

        self.all_points_list = self._compute_points(pose, origin_point)

    @property
    def body_contact_point(self):
        return self.all_points_list[0]

    @property
    def coxia_point(self):
        return self.all_points_list[1]

    @property
    def femur_point(self):
        return self.all_points_list[2]

    @property
    def foot_tip_point(self):
        return self.all_points_list[3]

    @property
    def id(self):
        return POSITION_NAME_TO_ID_MAP[self.position]

    @property
    def name(self):
        return f"{self.position}Leg"

    @property
    def maybe_ground_contact_point(self):
        reversed_list = list(reversed(self.all_points_list))
        test_point = reversed_list[0]
        maybe_ground_contact_point = min(reversed_list, key=lambda point: point["z"])
        return maybe_ground_contact_point

    def clone_trot_shift(self, transform_matrix, tx, ty, tz):
        return self._do_transform("clone_trot_shift", transform_matrix, tx, ty, tz)

    def clone_trot(self, transform_matrix):
        return self._do_transform("clone_trot", transform_matrix)

    def clone_shift(self, tx, ty, tz):
        return self._do_transform("clone_shift", tx, ty, tz)

    def _do_transform(self, transform_function, *args):
        new_points_list = [
            getattr(old_point, transform_function)(*args)
            for old_point in self.all_points_list
        ]
        return self._build_clone(new_points_list)

    def _build_clone(self, all_points_list):
        clone = Linkage(
            self.dimensions,
            self.position,
            self.body_contact_point,
            self.pose,
            {"hasNoPoints": True},
        )
        clone.all_points_list = all_points_list
        return clone

    def _build_name_id(self, point_name, id_):
        return {"name": f"{self.position}-{point_name}", "id": f"{self.id}-{id_}"}

    def _build_point_name_ids(self):
        return [
            self._build_name_id(point_type, index)
            for index, point_type in enumerate(LEG_POINT_TYPES_LIST)
        ]

    def _compute_points_wrt_body_contact(self, beta, gamma):
        matrix01 = tRotYmatrix(-beta, self.dimensions["coxia"], 0, 0)
        matrix12 = tRotYmatrix(90 - gamma, self.dimensions["femur"], 0, 0)
        matrix23 = tRotYmatrix(0, self.dimensions["tibia"], 0, 0)
        matrix02 = multiply4x4(matrix01, matrix12)
        matrix03 = multiply4x4(matrix02, matrix23)

        origin_point = Vector(0, 0, 0)

        local_points = [
            origin_point,  # bodyContactPoint
            origin_point.clone_trot(matrix01),  # coxiaPoint
            origin_point.clone_trot(matrix02),  # femurPoint
            origin_point.clone_trot(matrix03),  # footTipPoint
        ]

        return local_points

    def _compute_points_wrt_hexapod_cog(self, alpha, origin_point, local_points, point_name_ids):
        z_angle = POSITION_NAME_TO_AXIS_ANGLE_MAP[self.position] + alpha

        twist_matrix = tRotZmatrix(
            z_angle, origin_point["x"], origin_point["y"], origin_point["z"]
        )

        all_points_list = [
            local_point.new_trot(twist_matrix, name, id_)
            for local_point, (name, id_) in zip(local_points, point_name_ids)
        ]

        return all_points_list

    def _compute_points(self, pose, origin_point):
        alpha, beta, gamma = pose["alpha"], pose["beta"], pose["gamma"]
        point_name_ids = self._build_point_name_ids()
        local_points = self._compute_points_wrt_body_contact(beta, gamma)
        all_points_list = self._compute_points_wrt_hexapod_cog(
            alpha, origin_point, local_points, point_name_ids
        )
        return all_points_list
