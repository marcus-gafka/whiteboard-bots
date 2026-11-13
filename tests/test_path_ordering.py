from paths import PathOrdering as po


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
    geometries = [
        [(0, 0), (1, 0)],
        [(10, 0), (11, 0)],
        [(5, 5), (5, 6)],
    ]
    ordered = po.tsp_geometry_order((0.0, 0.0), geometries)
    assert len(ordered) == len(geometries)
    ordered_sets = [set(map(tuple, g)) for g in ordered]
    for g in geometries:
        assert set(map(tuple, g)) in ordered_sets


def test_tsp_geometry_order_starts_nearest_the_given_start():
    near_origin = [(1, 0), (1, 1)]
    near_100 = [(99, 0), (99, 1)]
    assert po.tsp_geometry_order((0.0, 0.0), [near_100, near_origin])[0] == near_origin
    assert po.tsp_geometry_order((100.0, 0.0), [near_origin, near_100])[0] == near_100


def test_tsp_order_indices_starts_from_given_pen_position():
    geometries = [[(0, 0), (1, 0)], [(10, 0), (9, 0)]]
    # Pen near (10, 0): piece 1 first, already oriented, then piece 0
    # entered from its nearer end (1, 0) -> reversed.
    assert po.tsp_order_indices((10.5, 0), geometries) == [(1, False), (0, True)]
    assert po.tsp_order_indices((0, 0), []) == []
