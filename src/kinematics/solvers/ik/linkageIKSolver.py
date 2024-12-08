"""
* * *

ALIASES:
    p0: bodyContactPoint (local)
    p1: coxaPoint (local)
    p2: femurPoint (local)
    p3: footTipPoint (local)

           p2x'
           /
          /          GIVEN:
         * p2         * (coxia) distance from p0 to p1
        /|            * (femur) distance from p1 to p2
       / |            * (tibia) distance from p2 to p3
p0  p1/  |            * (summa) distance from p0 to p3
 *---*   | ---> p1x'_  * rho: angle between
  \   \  |                 -> p0 to p1 and p0 to p3
    \  \ |            * p0 is (0, 0, 0)
      \ \|            * p0 to p1 vector is inline with legXaxis
        * p3
                    INTERMEDIATE:
legZaxis              * (pars) distance from p1 to p3
 ^                    * theta: angle between
 |                          -> p1 to p2 and p1 and p3
 * - > legXaxis       * phi: angle between
                            -> p1 to p3 and p1 to p1x'_ (legXaxis)
                      * epsi: angle between
                            -> p2 to p1 and p2 to p3

FIND: (counter clockwise is positive)
  * beta: Angle betweenlegXaxis and p1 to p2
          beta > 0 if p1 to p2 is above legXaxis
          beta < 0 if p1 to p2 is below the legXaxis
  * gamma: Angle between p1 to p2 and axis perpendicular to p1 p2

EXAMPLE: When p0, p2, p3, and p3 are configured this way then:
    * p2 to p3z' axis is 180 degrees wrt legZaxis
    * beta = 0
    * gamma = +90

p0   p1   p2   p3
*----*----*----*
          |
          |
          V p3z'
          
 * * *
"""
import math
from vector import Vector  
from geometry import (
    vector_from_to,
    angle_between,
    radians,
    vector_length,
    angle_opposite_of_last_side,
    is_triangle
)
from IKInfo import LegIKInfo


class LinkageIKSolver:
    def __init__(self, leg_position):
        self.info = LegIKInfo.initialized(leg_position)
        self.vectors = {
            "legXaxis": Vector(1, 0, 0, "legXaxis"),
            "parsVector": None
        }
        self.points = {
            "bodyContactPoint": None,
            "coxiaPoint": None,
            "targetFootTipPoint": None
        }
        self.dimensions = {
            "coxia": 0,
            "femur": 0,
            "tibia": 0,
            "summa": 0,
            "pars": 0
        }
        self.angles = {
            "beta": None,
            "gamma": None,
            "rho": None
        }

    def solve(self, coxia, femur, tibia, summa, rho):
        self.angles["rho"] = rho
        self.dimensions = {"coxia": coxia, "femur": femur, "tibia": tibia, "summa": summa}
        coxia_point = Vector(coxia, 0, 0, "coxiaPoint")
        target_foot_tip_point = self._compute_target_foot_tip_point()

        pars_vector = vector_from_to(coxia_point, target_foot_tip_point)
        pars = vector_length(pars_vector)

        self.dimensions["pars"] = pars
        self.points.update({"coxiaPoint": coxia_point, "targetFootTipPoint": target_foot_tip_point})
        self.vectors.update({"parsVector": pars_vector})

        if is_triangle(pars, femur, tibia):
            self._handle_case_triangle_can_form()
        else:
            self._handle_edge_case()

        return self

    @property
    def leg_position(self):
        return self.info.leg_position

    @property
    def beta(self):
        return self.angles["beta"]

    @property
    def gamma(self):
        return self.angles["gamma"]

    @property
    def obtained_solution(self):
        return self.info.obtained_solution

    @property
    def reached_target(self):
        return self.info.reached_target

    @property
    def message(self):
        return self.info.message

    def _compute_target_foot_tip_point(self):
        summa, rho = self.dimensions["summa"], self.angles["rho"]
        px = summa * math.cos(radians(rho))
        pz = -summa * math.sin(radians(rho))
        return Vector(px, 0, pz, "targetLocalFootTipPoint")

    def _handle_case_triangle_can_form(self):
        femur, pars, tibia = self.dimensions["femur"], self.dimensions["pars"], self.dimensions["tibia"]
        pars_vector, leg_xaxis = self.vectors["parsVector"], self.vectors["legXaxis"]
        target_foot_tip_point = self.points["targetFootTipPoint"]

        theta = angle_opposite_of_last_side(femur, pars, tibia)
        phi = angle_between(pars_vector, leg_xaxis)
        beta = theta - phi if target_foot_tip_point.z < 0 else theta + phi

        epsi = angle_opposite_of_last_side(femur, tibia, pars)
        femur_point_z = femur * math.sin(radians(beta))

        self.angles["beta"] = beta

        if target_foot_tip_point.z > femur_point_z:
            self.info = LegIKInfo.blocked(self.leg_position)
            return

        self.angles["gamma"] = epsi - 90
        self.info = LegIKInfo.target_reached(self.leg_position)

    def _handle_edge_case(self):
        pars, tibia, femur = self.dimensions["pars"], self.dimensions["tibia"], self.dimensions["femur"]

        if pars + tibia < femur:
            self.info = LegIKInfo.femur_too_long(self.leg_position)
            return

        if pars + femur < tibia:
            self.info = LegIKInfo.tibia_too_long(self.leg_position)
            return

        pars_vector, leg_xaxis = self.vectors["parsVector"], self.vectors["legXaxis"]
        self.angles.update({
            "beta": -angle_between(pars_vector, leg_xaxis),
            "gamma": 90
        })

        self.info = LegIKInfo.target_not_reached(self.leg_position)
