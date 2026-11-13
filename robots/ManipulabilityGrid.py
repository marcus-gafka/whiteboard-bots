import numpy as np


class ManipulabilityGrid:
    """Board-space manipulability heatmap for a robot, computed independent
    of any plotting library so it can be built/tested without matplotlib.

    values[j, i] is the manipulability at board sample (i, j); robot_x/y
    hold the same sample points converted to robot-frame coordinates (for
    rendering against a robot-frame axes, as Plotter2D does).
    """

    def __init__(self, robot, resolution_x=200, resolution_y=200):
        self.robot = robot

        board_x, board_y = np.meshgrid(
            np.linspace(0, robot.DRAWABLE_WIDTH, resolution_x),
            np.linspace(0, robot.DRAWABLE_HEIGHT, resolution_y),
        )
        self.robot_x = np.zeros_like(board_x)
        self.robot_y = np.zeros_like(board_y)
        self.values = np.zeros_like(board_x)

        for j in range(resolution_y):
            for i in range(resolution_x):
                bx, by = board_x[j, i], board_y[j, i]
                self.robot_x[j, i], self.robot_y[j, i] = robot.apply_board_offset(bx, by)
                try:
                    theta1, theta2 = robot.ik(bx, by)
                    self.values[j, i] = robot.manipulability(theta1, theta2)
                except (ValueError, np.linalg.LinAlgError):
                    # Expected at board positions the robot can't reach, or
                    # where the Jacobian is singular — anything else (a real
                    # bug in ik/manipulability) is left to propagate.
                    self.values[j, i] = 0.0

    @property
    def extent(self):
        """(xmin, xmax, ymin, ymax) in robot-frame coordinates, for imshow."""
        return (
            np.min(self.robot_x), np.max(self.robot_x),
            np.min(self.robot_y), np.max(self.robot_y),
        )
