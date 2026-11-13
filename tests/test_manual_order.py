import colorsys

import matplotlib.pyplot as plt
import numpy as np
import pytest

from paths import ManualOrder as mo


class FakeRobot:
    DRAWABLE_WIDTH = 20
    DRAWABLE_HEIGHT = 20
    START_XY = (0.0, 0.0)


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def test_split_piece():
    points = [(0, 0), (1, 0), (2, 0), (3, 0)]
    a, b = mo.split_piece(points, 1)
    assert a == [(0, 0), (1, 0)]
    assert b == [(2, 0), (3, 0)]


def test_cuttable_and_valid_range():
    long_enough = [(i, 0) for i in range(6)]
    too_short = [(0, 0), (1, 0), (2, 0)]
    assert mo.cuttable(long_enough) is True
    assert mo.cuttable(too_short) is False

    lo, hi = mo.valid_cut_index_range(long_enough)
    assert (lo, hi) == (mo.MIN_PIECE_LEN - 1, len(long_enough) - 1 - mo.MIN_PIECE_LEN)


def test_nearest_point_index():
    points = [(0, 0), (5, 0), (10, 0)]
    assert mo.nearest_point_index(points, (4.4, 0)) == 1
    assert mo.nearest_point_index(points, (9.9, 0)) == 2


def test_renumber_pool_is_compact_and_sequential():
    display_of, reverse_of = mo.renumber_pool({7, 3, 19})
    assert sorted(display_of.values(), key=int) == ["1", "2", "3"]
    assert set(reverse_of.keys()) == {"1", "2", "3"}
    for label, piece_id in reverse_of.items():
        assert display_of[piece_id] == label


def test_build_ordered_path_applies_reverse_flags():
    all_pieces = {0: [(0, 0), (1, 1)], 1: [(2, 2), (3, 3)]}
    placed_order = [(1, True), (0, False)]
    path = mo.build_ordered_path(all_pieces, placed_order)
    assert path == [[(3, 3), (2, 2)], [(0, 0), (1, 1)]]


def test_pick_colors_count_and_uniqueness():
    colors_20 = mo.pick_colors(20)
    colors_25 = mo.pick_colors(25)
    assert len(colors_20) == 20
    assert len(colors_25) == 25
    assert len({tuple(c) for c in colors_25}) == 25


def test_color_for_piece_siblings_diverge_same_hue():
    colors_by_root = {0: mo.pick_colors(1)[0]}
    root_of = {0: 0, 1: 0, 2: 0}
    child_path = {0: (), 1: (0,), 2: (1,)}

    root_color = mo.color_for_piece(0, colors_by_root, root_of, child_path)
    child_a = mo.color_for_piece(1, colors_by_root, root_of, child_path)
    child_b = mo.color_for_piece(2, colors_by_root, root_of, child_path)

    assert not np.allclose(child_a, child_b)

    def hue(c):
        h, _l, _s = colorsys.rgb_to_hls(c[0], c[1], c[2])
        return round(h, 4)

    assert hue(root_color) == hue(child_a) == hue(child_b)


# ---------------------------------------------------------------------------
# End-to-end simulated sessions (scripted input()/ginput())
# ---------------------------------------------------------------------------

def _run_session(geometries, robot, inputs, click_point=(2.1, 0.05)):
    remaining_inputs = iter(inputs)

    def fake_input(prompt=""):
        return next(remaining_inputs)

    def fake_ginput(n, timeout=0):
        return [click_point]

    import builtins
    original_input = builtins.input
    original_ginput = plt.ginput
    builtins.input = fake_input
    plt.ginput = fake_ginput
    try:
        return mo.prompt_manual_order(geometries, robot)
    finally:
        builtins.input = original_input
        plt.ginput = original_ginput


def test_session_place_cut_undo_end_to_end():
    # geom id0 -> pool label '1' (6 pts, 0..5); geom id1 -> pool label '2'
    # (10..13). Cutting '1' near (2,0) splits it into new pool entries
    # relabeled '2'/'3' (renumbered), pushing the untouched second geometry
    # to '1'. A place-then-undo of one of the cut pieces must restore it to
    # the pool under its exact prior label, unaffected by anything else.
    geometries = [
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)],
        [(10, 0), (11, 0), (12, 0), (13, 0)],
    ]
    result = _run_session(
        geometries, FakeRobot(),
        inputs=["cut 1", "1", "3r", "undo", "3", "2"],
    )
    assert result == [
        [(10, 0), (11, 0), (12, 0), (13, 0)],
        [(3, 0), (4, 0), (5, 0)],
        [(0, 0), (1, 0), (2, 0)],
    ]


def test_session_plain_placement_does_not_renumber_pool():
    # Regression test for the exact bug reported in this session: placing
    # one pool item must not shift the numbers of the others. Cuts pool
    # entry '1' (creating two new entries, renumbering to ['1'..'5']), then
    # places out of order — if placement ever renumbered the pool, later
    # tokens would silently point at the wrong piece and this would fail
    # with "not a placeable geometry number" or StopIteration.
    geometries = [
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
        [(10, 0), (11, 0), (12, 0), (13, 0), (14, 0)],
        [(20, 0), (21, 0), (22, 0), (23, 0), (24, 0)],
        [(30, 0), (31, 0), (32, 0), (33, 0), (34, 0)],
    ]
    result = _run_session(
        geometries, FakeRobot(),
        inputs=["cut 1", "cut 5", "4", "3", "2", "1", "5"],
    )
    assert len(result) == 5
    assert sum(len(piece) for piece in result) == 20


def test_session_undo_of_cut_restores_original_numbering():
    geometries = [
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)],
        [(10, 0), (11, 0), (12, 0), (13, 0)],
    ]
    result = _run_session(
        geometries, FakeRobot(),
        inputs=["cut 1", "undo", "1", "2"],
    )
    assert result == [
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)],
        [(10, 0), (11, 0), (12, 0), (13, 0)],
    ]


def test_session_cancelled_cut_does_not_change_state():
    geometries = [[(0, 0), (1, 0), (2, 0)]]

    inputs = iter(["cut 1", "1"])
    clicks = iter([[]])  # empty click list simulates Escape/cancel

    import builtins
    original_input = builtins.input
    original_ginput = plt.ginput
    builtins.input = lambda prompt="": next(inputs)
    plt.ginput = lambda n, timeout=0: next(clicks)
    try:
        result = mo.prompt_manual_order(geometries, FakeRobot())
    finally:
        builtins.input = original_input
        plt.ginput = original_ginput

    assert result == [[(0, 0), (1, 0), (2, 0)]]


def test_session_raises_manual_order_aborted_when_window_closed():
    geometries = [[(0, 0), (1, 0)], [(5, 0), (6, 0)]]
    calls = {"n": 0}

    def fake_input(prompt=""):
        calls["n"] += 1
        if calls["n"] == 1:
            for num in list(plt.get_fignums()):
                plt.close(num)
            return "1"
        raise AssertionError("should not be prompted again after the window closed")

    import builtins
    original_input = builtins.input
    builtins.input = fake_input
    try:
        with pytest.raises(mo.ManualOrderAborted):
            mo.prompt_manual_order(geometries, FakeRobot())
    finally:
        builtins.input = original_input


# ---------------------------------------------------------------------------
# ManualOrderState (UI-independent API used by the web app)
# ---------------------------------------------------------------------------

def _state():
    return mo.ManualOrderState([
        [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)],
        [(10, 0), (11, 0), (12, 0), (13, 0)],
    ])


def test_state_place_and_pen_position():
    st = _state()
    assert st.pen_position((9, 9)) == (9, 9)
    st.place(1, reverse=True)
    assert st.pen_position((9, 9)) == (10, 0)
    with pytest.raises(mo.ManualOrderError):
        st.place(1)


def test_state_cut_at_point_and_undo():
    st = _state()
    a, b = st.cut_at_point(0, (2.1, 0))
    assert st.all_pieces[a] == [(0, 0), (1, 0), (2, 0)]
    assert st.all_pieces[b] == [(3, 0), (4, 0), (5, 0)]
    assert st.pool_ids == {1, a, b}
    assert st.undo() == "cut"
    assert st.pool_ids == {0, 1} and a not in st.all_pieces


def test_state_rejects_bad_cuts():
    st = _state()
    with pytest.raises(mo.ManualOrderError):
        st.cut(0, 0)          # too close to the start
    st.place(1)
    short = mo.ManualOrderState([[(0, 0), (1, 0), (2, 0)]])
    with pytest.raises(mo.ManualOrderError):
        short.cut(0, 1)       # 3 points is too short to cut
    st.undo()
    with pytest.raises(mo.ManualOrderError):
        st.undo()             # failed cuts leave nothing to undo


def test_state_place_many_is_one_undo_step():
    st = _state()
    st.place_many([(1, False), (0, True)])
    assert st.is_complete
    assert st.ordered_path() == [[(10, 0), (11, 0), (12, 0), (13, 0)],
                                 [(5, 0), (4, 0), (3, 0), (2, 0), (1, 0), (0, 0)]]
    assert st.undo() == "batch"
    assert st.pool_ids == {0, 1} and st.placed_order == [] and not st.history
    with pytest.raises(mo.ManualOrderError):
        st.place_many([(0, False), (0, True)])
