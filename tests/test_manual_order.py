import pytest

from paths import ManualOrder as mo


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


def test_build_ordered_path_applies_reverse_flags():
    all_pieces = {0: [(0, 0), (1, 1)], 1: [(2, 2), (3, 3)]}
    placed_order = [(1, True), (0, False)]
    path = mo.build_ordered_path(all_pieces, placed_order)
    assert path == [[(3, 3), (2, 2)], [(0, 0), (1, 1)]]


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


def test_state_flip_reverses_a_placed_piece_and_undoes():
    st = _state()
    st.place(0)
    st.place(1)
    assert st.pen_position((9, 9)) == (13, 0)
    st.flip(1)
    assert st.placed_order == [(0, False), (1, True)]
    assert st.ordered_path()[1] == [(13, 0), (12, 0), (11, 0), (10, 0)]
    assert st.pen_position((9, 9)) == (10, 0)
    assert st.undo() == "flip"
    assert st.placed_order == [(0, False), (1, False)]
    with pytest.raises(mo.ManualOrderError):
        _state().flip(0)          # still in the pool

