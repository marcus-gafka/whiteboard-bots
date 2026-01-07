import math
import numpy as np
from robots.Robot import Robot
from robots import Kinematics as kin


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
        x, y = self.apply_board_offset(x,y)

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

    def fk(self, theta1_deg, theta2_deg, elbow_up=True):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)

        # Base joints
        A = np.array([0.0, 0.0])
        B = np.array([self.l1, 0.0])

        # Elbow joints
        C = A + self.l5 * np.array([math.cos(t1), math.sin(t1)])
        D = B + self.l2 * np.array([math.cos(t2), math.sin(t2)])

        # End-effector via loop closure (l3-l4 intersection)
        E = kin.circle_intersection(
            C, self.l4,
            D, self.l3,
            elbow_up=elbow_up
        )

        # Tip position offset along l3 (adds "a")
        vec_l3 = E - D           # vector along l3
        length_l3 = np.linalg.norm(vec_l3)
        tip = E + (self.a / length_l3) * vec_l3  # tip extends along l3

        return {
            "A": A,
            "B": B,
            "C": C,
            "D": D,
            "E": E,
            "tip": tip
        }

    # -------------------------------------------------
    # Plot-ready link structure
    # -------------------------------------------------
    def get_links(self, theta1_deg, theta2_deg):
        pts = self.fk(theta1_deg, theta2_deg)

        return [
            (pts["tip"], pts["D"]), # always put "tip" first
            (pts["A"], pts["C"]),  
            (pts["C"], pts["E"]),  
            (pts["D"], pts["B"])   
        ]
    
    def jacobian(self, theta1_deg, theta2_deg, elbow_up=True):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)

        # Base joints
        A = np.array([0.0, 0.0])
        B = np.array([self.l1, 0.0])

        # Elbow joints
        C = A + self.l5 * np.array([math.cos(t1), math.sin(t1)])
        D = B + self.l2 * np.array([math.cos(t2), math.sin(t2)])

        # Tip via loop closure
        E = kin.circle_intersection(C, self.l4, D, self.l3, elbow_up=elbow_up)

        # Partial derivatives of F = |E-C|^2 - l4^2
        Fx = 2 * (E[0] - C[0])
        Fy = 2 * (E[1] - C[1])
        Ftheta1 = -2 * (E[0] - C[0]) * (-self.l5 * math.sin(t1)) \
                -2 * (E[1] - C[1]) * (self.l5 * math.cos(t1))

        # Partial derivatives of G = |E-D|^2 - l3^2
        Gx = 2 * (E[0] - D[0])
        Gy = 2 * (E[1] - D[1])
        Gtheta2 = -2 * (E[0] - D[0]) * (-self.l2 * math.sin(t2)) \
                -2 * (E[1] - D[1]) * (self.l2 * math.cos(t2))

        # Solve for dE/dθ1
        J = np.zeros((2, 2))
        M = np.array([[Fx, Fy],
                    [Gx, Gy]])

        # dE/dθ1
        rhs1 = np.array([-Ftheta1, 0.0])
        dE_dtheta1 = np.linalg.solve(M, rhs1)

        # dE/dθ2
        rhs2 = np.array([0.0, -Gtheta2])
        dE_dtheta2 = np.linalg.solve(M, rhs2)

        J[:, 0] = dE_dtheta1
        J[:, 1] = dE_dtheta2

        return J
