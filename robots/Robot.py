import abc


class Robot(abc.ABC):
    """Shared contract for the interchangeable drawing mechanisms.

    Subclasses implement the kinematics (ik/fk/get_links/get_bases); step and
    timing calculation and the board <-> robot frame offset live here.
    """

    name = ""

    # Pen home position, in board coordinates. Every trajectory starts here.
    START_XY = (0.0, 0.0)

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

    # Robot-frame region the web preview frames (bases plus reach).
    MIN_PLOT_X = 0
    MAX_PLOT_X = 0
    MIN_PLOT_Y = 0
    MAX_PLOT_Y = 0

    # How the linkage is drawn: "rigid" links or flexible "cable"s.
    LINK_KIND = "rigid"

    def __init__(self):
        for attr in ("DRAWABLE_WIDTH", "DRAWABLE_HEIGHT", "OFFSET_X", "OFFSET_Y"):
            if getattr(self, attr) is None:
                raise NotImplementedError(f"{self.__class__.__name__} must define {attr}")

    def apply_board_offset(self, board_x, board_y):
        return board_x + self.OFFSET_X, board_y + self.OFFSET_Y

    def apply_inverse_board_offset(self, robot_x, robot_y):
        return robot_x - self.OFFSET_X, robot_y - self.OFFSET_Y

    @abc.abstractmethod
    def ik(self, x, y):
        """Inverse kinematics: board (x, y) -> (theta1_deg, theta2_deg).
        Raises ValueError when the point is out of reach."""

    @abc.abstractmethod
    def fk(self, theta1_deg, theta2_deg):
        """Forward kinematics: joint angles -> dict of named robot-frame points, including 'tip'."""

    @abc.abstractmethod
    def get_links(self, theta1_deg, theta2_deg):
        """List of (point, point) robot-frame segments describing the linkage; the first starts at the tip."""

    @abc.abstractmethod
    def get_bases(self):
        """Fixed (motor-mounted) points of the linkage, in robot coordinates."""

    @classmethod
    def calc_steps(cls, current_t1, current_t2, target_t1, target_t2):
        """(steps1, steps2, interval_ms) for one move. Steps are negated to
        match the motors' wiring; the interval runs the busier motor at full speed."""
        steps1 = int(round((target_t1 - current_t1) / cls.DEGREES_PER_STEP))
        steps2 = int(round((target_t2 - current_t2) / cls.DEGREES_PER_STEP))
        max_steps = max(abs(steps1), abs(steps2))
        interval = 0 if max_steps == 0 else int((max_steps / cls.MAX_STEPS_PER_SEC) * 1000)
        return -steps1, -steps2, interval
