import math
import numpy as np
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

    # Link lengths
    SPOOL_DISTANCE = 12
    SPOOL_DIAMETER = 2

    def __init__(self):

        self.ORIGIN_L1 = self.calc_l1(self.OFFSET_X, self.OFFSET_Y)
        self.ORIGIN_L2 = self.calc_l2(self.OFFSET_X, self.OFFSET_Y)

    def calc_l1(self, x, y):
        return math.sqrt(x**2 + y**2)

    def calc_l2(self, x, y):
        return math.sqrt((self.SPOOL_DISTANCE - x)**2 + y**2)

    def ik(self, x, y):
        x, y = self.apply_board_offset(x,y)

        l1 = self.calc_l1(x, y)
        l2 = self.calc_l2(x, y)

        t1 = ((l1 - self.ORIGIN_L1) * 360) / (math.pi * self.SPOOL_DIAMETER)
        t2 = ((l2 - self.ORIGIN_L2) * 360) / (math.pi * self.SPOOL_DIAMETER)

        return t1, t2

    def fk(self, theta1_deg, theta2_deg):
        # Convert angles to cable lengths
        l1 = self.ORIGIN_L1 + (theta1_deg * math.pi * self.SPOOL_DIAMETER) / 360
        l2 = self.ORIGIN_L2 + (theta2_deg * math.pi * self.SPOOL_DIAMETER) / 360

        # Anchor points
        A = np.array([0.0, 0.0])
        B = np.array([self.SPOOL_DISTANCE, 0.0])

        # Circle-circle intersection
        d = self.SPOOL_DISTANCE
        x = (l1**2 - l2**2 + d**2) / (2 * d)

        y_sq = l1**2 - x**2
        if y_sq < 0:
            raise ValueError("FK impossible: cable lengths do not intersect")

        # Choose lower solution (below spools)
        y = -math.sqrt(y_sq)

        # Apply board offset removal
        E = np.array([x, y])

        return {
            "A": A,
            "B": B,
            "tip": E,      # end effector / marker
            "l1": l1,
            "l2": l2
        }
    
    def jacobian(self, x, y):

        l1 = self.calc_l1(x, y)
        l2 = self.calc_l2(x, y)
        S = self.SPOOL_DISTANCE
        D = self.SPOOL_DIAMETER

        # Partial derivatives of lengths w.r.t x, y
        dl1_dx = x / l1
        dl1_dy = y / l1
        dl2_dx = (x - S) / l2
        dl2_dy = y / l2

        # Jacobian from tip velocities to length changes
        J_lengths = np.array([[dl1_dx, dl1_dy],
                            [dl2_dx, dl2_dy]])

        # Convert Δl → Δt (degrees)
        scale = 360 / (math.pi * D)
        J = np.linalg.inv(J_lengths) * scale

        return J


    def get_links(self, theta1_deg, theta2_deg):
        pts = self.fk(theta1_deg, theta2_deg)

        return [
            (pts["tip"], pts["A"]),  # left cable
            (pts["tip"], pts["B"])   # right cable
        ]
