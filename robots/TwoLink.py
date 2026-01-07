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

    # -------------------------------------------------
    # Inverse kinematics
    # -------------------------------------------------
    def ik(self, x, y):
        x += self.OFFSET_X
        y += self.OFFSET_Y

        # Distance to wrist
        r = math.hypot(x, y)

        # Law of cosines
        cos_theta2 = (r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # clamp

        # Elbow-up: take negative acos
        theta2 = -math.acos(cos_theta2)

        # Compute theta1
        phi = math.atan2(y, x)
        psi = math.atan2(self.l2 * math.sin(theta2), self.l1 + self.l2 * math.cos(theta2))
        theta1 = phi - psi

        return math.degrees(theta1), math.degrees(theta2)
    
    # -------------------------------------------------
    # Forward kinematics
    # -------------------------------------------------
    def fk(self, theta1_deg, theta2_deg, elbow_up=True):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)

        # Base
        A = np.array([0.0, 0.0])

        # Elbow joint
        B = A + self.l1 * np.array([math.cos(t1), math.sin(t1)])

        # Determine elbow-up or elbow-down
        if not elbow_up:
            t2 = -t2  # flip second link for "elbow down"

        # Tip position
        tip = B + self.l2 * np.array([math.cos(t1 + t2), math.sin(t1 + t2)])

        return {
            "A": A,
            "B": B,
            "tip": tip
        }
    
    def jacobian(self, theta1_deg, theta2_deg):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)

        J = np.array([
            [
                -self.l1 * math.sin(t1) - self.l2 * math.sin(t1 + t2),
                -self.l2 * math.sin(t1 + t2)
            ],
            [
                self.l1 * math.cos(t1) + self.l2 * math.cos(t1 + t2),
                self.l2 * math.cos(t1 + t2)
            ]
        ])

        return J
    
    # -------------------------------------------------
    # Plot-ready link structure
    # -------------------------------------------------
    def get_links(self, theta1_deg, theta2_deg):
        pts = self.fk(theta1_deg, theta2_deg)
        return [
            (pts["tip"], pts["B"]),
            (pts["A"], pts["B"])
        ]
