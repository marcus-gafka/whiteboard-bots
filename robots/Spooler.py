import math

import numpy as np

from robots import Kinematics as kin
from robots.Robot import Robot


class Spooler(Robot):

    name = "Spooler"

    # Drawing Area
    DRAWABLE_WIDTH = 12.0
    DRAWABLE_HEIGHT = 8.0

    # Translation adjustment Robot -> Board
    OFFSET_X = 0.0
    OFFSET_Y = -10.0

    # Plot bounds
    MIN_PLOT_X = -1
    MAX_PLOT_X = 13
    MIN_PLOT_Y = -11
    MAX_PLOT_Y = 1

    LINK_KIND = "cable"

    # Spools: left at the robot origin, right SPOOL_DISTANCE along +x.
    SPOOL_DISTANCE = 12
    SPOOL_DIAMETER = 2

    def __init__(self):
        super().__init__()
        # Cable lengths with the pen at home (motor angle 0).
        self.ORIGIN_L1 = self.calc_l1(self.OFFSET_X, self.OFFSET_Y)
        self.ORIGIN_L2 = self.calc_l2(self.OFFSET_X, self.OFFSET_Y)

    def calc_l1(self, x, y):
        return math.hypot(x, y)

    def calc_l2(self, x, y):
        return math.hypot(self.SPOOL_DISTANCE - x, y)

    def _length_to_theta(self, length, origin_length):
        # Cable paid out = arc length on the spool: angle = arc_length / radius.
        return math.degrees((length - origin_length) / (self.SPOOL_DIAMETER / 2))

    def _theta_to_length(self, theta_deg, origin_length):
        return origin_length + math.radians(theta_deg) * (self.SPOOL_DIAMETER / 2)

    def ik(self, x, y):
        x, y = self.apply_board_offset(x, y)
        return (self._length_to_theta(self.calc_l1(x, y), self.ORIGIN_L1),
                self._length_to_theta(self.calc_l2(x, y), self.ORIGIN_L2))

    def fk(self, theta1_deg, theta2_deg):
        A = np.array([0.0, 0.0])
        B = np.array([float(self.SPOOL_DISTANCE), 0.0])
        # elbow_up=False: the pen hangs below the spools.
        tip = kin.circle_intersection(A, self._theta_to_length(theta1_deg, self.ORIGIN_L1),
                                      B, self._theta_to_length(theta2_deg, self.ORIGIN_L2),
                                      elbow_up=False)
        if tip is None:
            raise ValueError("FK impossible: cable lengths do not intersect")
        return {"A": A, "B": B, "tip": tip}

    def get_links(self, theta1_deg, theta2_deg):
        pts = self.fk(theta1_deg, theta2_deg)
        return [
            (pts["tip"], pts["A"]),  # left cable
            (pts["tip"], pts["B"]),  # right cable
        ]

    def get_bases(self):
        return [(0.0, 0.0), (float(self.SPOOL_DISTANCE), 0.0)]
