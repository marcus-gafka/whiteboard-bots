from paths import PathOrdering as po
from robots.TwoLink import TwoLink


def test_reduce_points_keeps_first_and_last():
    points = [(i, 0) for i in range(100)]
    reduced = po.reduce_points(points, 10)
    assert len(reduced) == 10
    assert reduced[0] == [0, 0]
    assert reduced[-1] == [99, 0]


def test_reduce_points_noop_when_n_exceeds_length():
    points = [(0, 0), (1, 0), (2, 0)]
    assert po.reduce_points(points, 10) == [[0, 0], [1, 0], [2, 0]]


def test_tsp_geometry_order_visits_every_geometry_once():
    robot = TwoLink()
    geometries = [
        [(0, 0), (1, 0)],
        [(10, 0), (11, 0)],
        [(5, 5), (5, 6)],
    ]
    ordered = po.tsp_geometry_order(robot, geometries)
    assert len(ordered) == len(geometries)
    original_sets = [set(map(tuple, g)) for g in geometries]
    ordered_sets = [set(map(tuple, g)) for g in ordered]
    for s in original_sets:
        assert s in ordered_sets


def test_tsp_geometry_order_picks_nearest_next_geometry():
    class OriginRobot:
        START_XY = (0.0, 0.0)

    near = [(1, 0), (1, 1)]
    far = [(100, 0), (100, 1)]
    ordered = po.tsp_geometry_order(OriginRobot(), [far, near])
    first_points = set(map(tuple, ordered[0]))
    assert first_points == set(map(tuple, near))


def test_tsp_geometry_order_uses_robot_start_not_class_default():
    # Regression test: geometry_dist_to_start must read the robot INSTANCE's
    # START_XY, not a hardcoded/class-level default, so a hypothetical robot
    # with a non-origin start position is still ordered correctly.
    class OffsetStartRobot:
        START_XY = (100.0, 0.0)

    near_offset_start = [(99, 0), (99, 1)]
    near_origin = [(1, 0), (1, 1)]
    ordered = po.tsp_geometry_order(OffsetStartRobot(), [near_origin, near_offset_start])
    first_points = set(map(tuple, ordered[0]))
    assert first_points == set(map(tuple, near_offset_start))
