import math

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
        assert kin.dist(p, c1) == pytest.approx(r, abs=1e-9)
        assert kin.dist(p, c2) == pytest.approx(r, abs=1e-9)


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


def test_dist_matches_euclidean_norm():
    assert kin.dist((0, 0), (3, 4)) == pytest.approx(5.0)


def test_rot2_is_a_valid_rotation_matrix():
    R = kin.rot2(math.pi / 2)
    p = np.array([1.0, 0.0])
    rotated = R @ p
    assert rotated[0] == pytest.approx(0.0, abs=1e-9)
    assert rotated[1] == pytest.approx(1.0, abs=1e-9)


def test_apply_transform_translates_and_rotates():
    T = kin.tf2(0.0, (5.0, 2.0))
    assert kin.apply(T, (1.0, 1.0)) == pytest.approx([6.0, 3.0])
