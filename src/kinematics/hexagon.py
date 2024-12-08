from .vector import Vector  # Importando a classe Vector
from .constants import POSITION_NAMES_LIST
"""
/* * *
  ..................
   Hexagon
  ..................

         |-f-|
         *---*---*--------   * f - front
        /    |    \     |    * s - side
       /     |     \    s    * m - middle
      /      |      \   |
     *------cog------* ---
      \      |      /|
       \     |     / |
        \    |    /  |
         *---*---*   |
             |       |
             |---m---|


      (leftFront)     (rightFront)
           v2          v1
            \   head  /
             *---*---*
            /    |    \
  (left    /     |     \
  Middle) /      |      \
    v3 --*------cog------*-- v0 (rightMiddle)
          \      |      /
           \     |     /
            \    |    /
             *---*---*
            /         \
          v4           v5
       (leftBack)   (rightBack)

 * * */

"""

class Hexagon:
    def __init__(self, dimensions, flags={"hasNoPoints": False}):
        self.dimensions = dimensions

        if flags.get("hasNoPoints", False):
            return

        front, middle, side = self.dimensions.values()
        vertex_x = [middle, front, -front, -middle, -front, front]
        vertex_y = [0, side, side, 0, -side, -side]

        self.vertices_list = [
            Vector(vertex_x[i], vertex_y[i], 0, f"{position}Vertex", i)
            for i, position in enumerate(POSITION_NAMES_LIST)
        ]
        self.head = Vector(0, side, 0, "headPoint", 7)
        self.cog = Vector(0, 0, 0, "centerOfGravityPoint", 6)

    @property
    def closed_points_list(self):
        return self.vertices_list + [self.vertices_list[0]]

    @property
    def all_points_list(self):
        return self.vertices_list + [self.cog, self.head]

    def clone_trot_shift(self, transform_matrix, tx, ty, tz):
        return self._do_transform("clone_trot_shift", transform_matrix, tx, ty, tz)

    def clone_trot(self, transform_matrix):
        return self._do_transform("clone_trot", transform_matrix)

    def clone_shift(self, tx, ty, tz):
        return self._do_transform("clone_shift", tx, ty, tz)

    def _do_transform(self, transform_function, *args):
        clone = Hexagon(self.dimensions, flags={"hasNoPoints": True})
        clone.cog = getattr(self.cog, transform_function)(*args)
        clone.head = getattr(self.head, transform_function)(*args)
        clone.vertices_list = [
            getattr(point, transform_function)(*args) for point in self.vertices_list
        ]
        return clone