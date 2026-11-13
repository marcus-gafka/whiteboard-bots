import numpy as np
import pytest

from robots import Kinematics as kin


def test_circle_intersection_known_symmetric_case():
    # Two circles of equal radius centered 10 apart intersect symmetrically
    # above/below the line joining their centers.
    c1 = np.array([0.0, 0.0])
    c2 = np.array([10.0, 0.0])
    r = 6.0

    up = kin.circle_intersection(c1, r, c2, r, elbow_up=True)
    down = kin.circle_intersection(c1, r, c2, r, elbow_up=False)

    assert up is not None and down is not None
    assert up[0] == pytest.approx(5.0)
    assert down[0] == pytest.approx(5.0)
    assert up[1] == pytest.approx(-down[1])
    assert up[1] > 0 > down[1]

    # Both solutions must actually lie on both circles.
    for p in (up, down):
        assert np.linalg.norm(p - c1) == pytest.approx(r, abs=1e-9)
        assert np.linalg.norm(p - c2) == pytest.approx(r, abs=1e-9)


def test_circle_intersection_too_far_apart_returns_none():
    c1 = np.array([0.0, 0.0])
    c2 = np.array([100.0, 0.0])
    assert kin.circle_intersection(c1, 1.0, c2, 1.0) is None


def test_circle_intersection_one_inside_other_returns_none():
    c1 = np.array([0.0, 0.0])
    c2 = np.array([1.0, 0.0])
    # r1 and r2 differ by more than the center distance -> one circle
    # entirely contains the other, no intersection.
    assert kin.circle_intersection(c1, 10.0, c2, 1.0) is None
