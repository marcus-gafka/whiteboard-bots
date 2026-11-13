import pytest

from robots.Robot import Robot
from robots.FiveBar import FiveBar
from robots.Spooler import Spooler
from robots.TwoLink import TwoLink

ROBOT_CLASSES = [FiveBar, Spooler, TwoLink]


def board_sample_points(robot):
    w, h = robot.DRAWABLE_WIDTH, robot.DRAWABLE_HEIGHT
    return [
        (w * 0.5, h * 0.5),
        (w * 0.3, h * 0.4),
        (w * 0.7, h * 0.6),
    ]


@pytest.mark.parametrize("robot_cls", ROBOT_CLASSES)
def test_required_geometry_attrs_are_defined(robot_cls):
    robot = robot_cls()
    for attr in ("DRAWABLE_WIDTH", "DRAWABLE_HEIGHT", "OFFSET_X", "OFFSET_Y"):
        assert getattr(robot, attr) is not None


def test_robot_is_abstract_and_rejects_subclass_missing_kinematics_methods():
    class MissingKinematics(Robot):
        DRAWABLE_WIDTH = 1.0
        DRAWABLE_HEIGHT = 1.0
        OFFSET_X = 0.0
        OFFSET_Y = 0.0
        # ik/fk/get_links/get_bases intentionally not implemented

    with pytest.raises(TypeError):
        MissingKinematics()


def test_robot_init_rejects_subclass_missing_geometry_attr():
    class MissingGeometry(Robot):
        DRAWABLE_WIDTH = 1.0
        DRAWABLE_HEIGHT = 1.0
        OFFSET_X = 0.0
        # OFFSET_Y intentionally left as the base class's None

        def ik(self, x, y):
            return 0.0, 0.0

        def fk(self, theta1_deg, theta2_deg):
            return {"tip": (0.0, 0.0)}

        def get_links(self, theta1_deg, theta2_deg):
            return [((0.0, 0.0), (0.0, 0.0))]

        def get_bases(self):
            return [(0.0, 0.0)]

    with pytest.raises(NotImplementedError):
        MissingGeometry()


@pytest.mark.parametrize("robot_cls", ROBOT_CLASSES)
def test_ik_fk_round_trip_recovers_board_point(robot_cls):
    robot = robot_cls()
    for x, y in board_sample_points(robot):
        theta1, theta2 = robot.ik(x, y)
        tip = robot.fk(theta1, theta2)["tip"]
        recovered_x, recovered_y = robot.apply_inverse_board_offset(*tip)
        assert recovered_x == pytest.approx(x, abs=1e-3)
        assert recovered_y == pytest.approx(y, abs=1e-3)


@pytest.mark.parametrize("robot_cls", ROBOT_CLASSES)
def test_get_links_returns_segment_pairs(robot_cls):
    robot = robot_cls()
    x, y = board_sample_points(robot)[0]
    theta1, theta2 = robot.ik(x, y)
    links = robot.get_links(theta1, theta2)
    assert len(links) >= 2
    for p1, p2 in links:
        assert len(p1) == 2
        assert len(p2) == 2


def test_calc_steps_sign_and_interval():
    # currentTheta -> targetTheta of +1 full step should yield -1 encoded step
    # (Robot.calc_steps negates), with zero interval when there's no motion.
    s1, s2, interval = Robot.calc_steps(0.0, 0.0, 0.0, 0.0)
    assert (s1, s2, interval) == (0, 0, 0)

    step_deg = Robot.DEGREES_PER_STEP
    s1, s2, interval = Robot.calc_steps(0.0, 0.0, step_deg, 0.0)
    assert s1 == -1
    assert s2 == 0
    assert interval >= 0
