import math

import numpy as np


def circle_intersection(c1, r1, c2, r2, elbow_up=True):
    """One intersection point of two circles (the one left of c1 -> c2 when
    elbow_up), or None if they don't meet. Used by the parallel mechanisms."""
    c1, c2 = np.asarray(c1, dtype=float), np.asarray(c2, dtype=float)
    d = np.linalg.norm(c2 - c1)
    if d > r1 + r2 or d < abs(r1 - r2):
        return None

    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(max(r1**2 - a**2, 0.0))
    p2 = c1 + a * (c2 - c1) / d
    offset = h * np.array([-(c2[1] - c1[1]) / d, (c2[0] - c1[0]) / d])
    return p2 + offset if elbow_up else p2 - offset
