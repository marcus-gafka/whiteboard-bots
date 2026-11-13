import math

import numpy as np

from robots.Robot import Robot


class TwoLink(Robot):
    name = "TwoLink"

    # Link lengths
    l1 = 8.0
    l2 = 8.0

    # Drawing Area
    DRAWABLE_WIDTH = 7.0
    DRAWABLE_HEIGHT = 7.0

    # Translation adjustment Robot -> Board
    OFFSET_X = 2.0
    OFFSET_Y = 2.0

    # Plot bounds
    MIN_PLOT_X = -7
    MAX_PLOT_X = 11
    MIN_PLOT_Y = -2
    MAX_PLOT_Y = 12

    def ik(self, x, y):
        x, y = self.apply_board_offset(x, y)

        # Law of cosines. acos raises ValueError when (x, y) is out of reach.
        r = math.hypot(x, y)
        theta2 = -math.acos((r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2))  # elbow up

        theta1 = math.atan2(y, x) - math.atan2(self.l2 * math.sin(theta2), self.l1 + self.l2 * math.cos(theta2))
        return math.degrees(theta1), math.degrees(theta2)

    def fk(self, theta1_deg, theta2_deg):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        A = np.array([0.0, 0.0])
        B = A + self.l1 * np.array([math.cos(t1), math.sin(t1)])
        tip = B + self.l2 * np.array([math.cos(t1 + t2), math.sin(t1 + t2)])
        return {"A": A, "B": B, "tip": tip}

    def get_links(self, theta1_deg, theta2_deg):
        pts = self.fk(theta1_deg, theta2_deg)
        return [
            (pts["tip"], pts["B"]),
            (pts["A"], pts["B"]),
        ]

    def get_bases(self):
        return [(0.0, 0.0)]
