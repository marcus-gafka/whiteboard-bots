import numpy as np
import math

class Robot:

    name = ""

    # Initial marker position
    START_XY = (0.0, 0.0)
    START_X = START_XY[0]
    START_Y = START_XY[1]

    # Motor values
    MAX_RPM = 5.0
    STEPS_PER_REV = 4096.0
    MAX_STEPS_PER_SEC = MAX_RPM * STEPS_PER_REV / 60.0
    DEGREES_PER_STEP = 360.0 / STEPS_PER_REV

    # Required geometry
    DRAWABLE_WIDTH = None
    DRAWABLE_HEIGHT = None
    OFFSET_X = None
    OFFSET_Y = None

    MIN_PLOT_X = 0
    MAX_PLOT_X = 0
    MIN_PLOT_Y = 0
    MAX_PLOT_Y = 0

    def __init__(self):
        for attr in ("DRAWABLE_WIDTH", "DRAWABLE_HEIGHT", "OFFSET_X", "OFFSET_Y"):
            if getattr(self, attr) is None:
                raise NotImplementedError(f"{self.__class__.__name__} must define {attr}")

    def get_aspect_ratio(self):
        return self.DRAWABLE_HEIGHT / self.DRAWABLE_WIDTH

    def get_drawable_center(self):
        return (
            self.OFFSET_X + self.DRAWABLE_WIDTH / 2.0,
            self.OFFSET_Y + self.DRAWABLE_HEIGHT / 2.0
        )
    
    def get_drawable_box(self):
        x0, y0 = self.OFFSET_X, self.OFFSET_Y
        x1, y1 = x0 + self.DRAWABLE_WIDTH, y0 + self.DRAWABLE_HEIGHT
        return [((x0, y0), (x0, y1)),
                ((x0, y1), (x1, y1)),
                ((x1, y1), (x1, y0)),
                ((x1, y0), (x0, y0))]
    
    def apply_board_offset(self, board_x, board_y):
        robot_x = board_x + self.OFFSET_X
        robot_y = board_y + self.OFFSET_Y
        return robot_x, robot_y
    
    def apply_inverse_board_offset(self, robot_x, robot_y):
        board_x = robot_x - self.OFFSET_X
        board_y = robot_y - self.OFFSET_Y
        return board_x, board_y

    @classmethod
    def calc_steps(cls, currentTheta1, currentTheta2, targetTheta1, targetTheta2):
        deltaTheta1 = targetTheta1 - currentTheta1
        deltaTheta2 = targetTheta2 - currentTheta2

        steps1 = int(round(deltaTheta1 / cls.DEGREES_PER_STEP))
        steps2 = int(round(deltaTheta2 / cls.DEGREES_PER_STEP))

        max_steps = max(abs(steps1), abs(steps2))
        interval = 0 if max_steps == 0 else int((max_steps / cls.MAX_STEPS_PER_SEC) * 1000)

        return -steps1, -steps2, interval
    
    def is_singular(self, theta1_deg, theta2_deg, tol=1e-6):
        tip = self.fk(theta1_deg, theta2_deg)["tip"]
        J = self.jacobian(tip[0], tip[1])
        sigma_min = np.linalg.svd(J, compute_uv=False)[-1]
        return sigma_min < tol

    def mobility(self, theta1_deg, theta2_deg):
        tip = self.fk(theta1_deg, theta2_deg)["tip"]
        J = self.jacobian(tip[0], tip[1])
        return np.linalg.matrix_rank(J)

    def manipulability(self, theta1_deg, theta2_deg):
        tip = self.fk(theta1_deg, theta2_deg)["tip"]
        J = self.jacobian(tip[0], tip[1])
        return math.sqrt(np.linalg.det(J @ J.T))

