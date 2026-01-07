import numpy as np
import math

# -----------------------------
# Basic 2D rotation
# -----------------------------
def rot2(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s],
        [s,  c]
    ])

# -----------------------------
# 2D homogeneous transform
# -----------------------------
def tf2(theta, p):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, p[0]],
        [s,  c, p[1]],
        [0,  0,  1]
    ])

# -----------------------------
# Apply transform to point
# -----------------------------
def apply(T, p):
    ph = np.array([p[0], p[1], 1.0])
    return (T @ ph)[:2]

# -----------------------------
# Distance helper
# -----------------------------
def dist(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

# -----------------------------
# Circle–circle intersection
# Used by parallel mechanisms
# -----------------------------
def circle_intersection(c1, r1, c2, r2, elbow_up=True):
    d = dist(c1, c2)
    if d > r1 + r2 or d < abs(r1 - r2):
        return None

    a = (r1**2 - r2**2 + d**2) / (2*d)
    h = math.sqrt(max(r1**2 - a**2, 0.0))

    p2 = c1 + a * (c2 - c1) / d

    offset = h * np.array([
        -(c2[1] - c1[1]) / d,
         (c2[0] - c1[0]) / d
    ])

    return p2 + offset if elbow_up else p2 - offset
