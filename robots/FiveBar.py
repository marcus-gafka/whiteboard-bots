import math

import numpy as np

from robots import Kinematics as kin
from robots.Robot import Robot


class FiveBar(Robot):

    name = "FiveBar"

    # Link lengths
    l1 = 3.0
    l2 = 6.0
    l3 = 7.5
    l4 = 7.5
    l5 = 6.0
    a  = 1.125

    # Drawing Area
    DRAWABLE_WIDTH = 10.0
    DRAWABLE_HEIGHT = 6.5

    # Translation adjustment Robot -> Board
    OFFSET_X = -3.5
    OFFSET_Y = 5.0

    # Plot bounds
    MIN_PLOT_X = -7
    MAX_PLOT_X = 10
    MIN_PLOT_Y = -2
    MAX_PLOT_Y = 12

    def ik(self, x, y):
        x, y = self.apply_board_offset(x, y)

        d = math.hypot(x - self.l1, y)
        theta2 = (
            math.atan2(y, x - self.l1)
            - math.acos(
                (-d**2 - self.l2**2 + (self.l3 + self.a)**2)
                / (-2.0 * self.l2 * d)
            )
        )

        x3 = (
            (self.a / (self.l3 + self.a)) * self.l2 * math.cos(theta2)
            + (self.a / (self.l3 + self.a)) * self.l1
            + (self.l3 / (self.l3 + self.a)) * x
        )

        y3 = (
            (self.a / (self.l3 + self.a)) * self.l2 * math.sin(theta2)
            + (self.l3 / (self.l3 + self.a)) * y
        )

        r = math.hypot(x3, y3)
        theta1 = (
            math.atan2(y3, x3)
            + math.acos(
                (-r**2 - self.l5**2 + self.l4**2)
                / (-2.0 * self.l5 * r)
            )
        )

        return math.degrees(theta1), math.degrees(theta2)

    def fk(self, theta1_deg, theta2_deg):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)

        # Base joints
        A = np.array([0.0, 0.0])
        B = np.array([self.l1, 0.0])

        # Elbow joints
        C = A + self.l5 * np.array([math.cos(t1), math.sin(t1)])
        D = B + self.l2 * np.array([math.cos(t2), math.sin(t2)])

        # Loop closure (l3-l4 intersection)
        E = kin.circle_intersection(C, self.l4, D, self.l3)

        # The tip extends past E along l3 by "a"
        vec_l3 = E - D
        tip = E + (self.a / np.linalg.norm(vec_l3)) * vec_l3

        return {"A": A, "B": B, "C": C, "D": D, "E": E, "tip": tip}

    def get_links(self, theta1_deg, theta2_deg):
        pts = self.fk(theta1_deg, theta2_deg)
        return [
            (pts["tip"], pts["D"]),
            (pts["A"], pts["C"]),
            (pts["C"], pts["E"]),
            (pts["D"], pts["B"]),
        ]

    def get_bases(self):
        return [(0.0, 0.0), (self.l1, 0.0)]
