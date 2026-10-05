"""Rotations and frame transforms.

Conventions (same as SDF and ROS REP-103):
  * boat frame (base_link): x forward, y left, z up, origin midway between the
    hulls, at hull-axis height, at mid hull length
  * an orientation is given as roll-pitch-yaw (rpy) in radians and means
    R = Rz(yaw) * Ry(pitch) * Rx(roll)   (rotate about x, then y, then z, all
    about the fixed parent axes)
  * a pose (xyz, rpy) of a child frame maps a point p_child given in the child
    frame to the parent frame:  p_parent = xyz + R(rpy) * p_child
"""
import math

Vec = list  # 3 floats


def rot(rpy):
    """Rotation matrix (3x3, row-major) from roll/pitch/yaw."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_vec(m, v):
    """m * v"""
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_t_vec(m, v):
    """transpose(m) * v  (= inverse rotation, because R is orthonormal)"""
    return [sum(m[k][i] * v[k] for k in range(3)) for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def to_parent(xyz, rpy, p_child):
    """Point given in a child frame -> the same point in the parent frame."""
    return add(xyz, mat_vec(rot(rpy), p_child))


def to_child(xyz, rpy, p_parent):
    """Point given in the parent frame -> the same point in the child frame."""
    return mat_t_vec(rot(rpy), sub(p_parent, xyz))


def elevation(v):
    """Angle of a direction above the horizontal x-y plane, radians."""
    return math.atan2(v[2], math.hypot(v[0], v[1]))


def ray(azimuth, elev):
    """Unit direction from azimuth (about z, from +x) and elevation."""
    return [math.cos(elev) * math.cos(azimuth), math.cos(elev) * math.sin(azimuth), math.sin(elev)]


def is_upside_down(rpy):
    """True if the frame's z axis points downwards (e.g. roll = pi)."""
    return rot(rpy)[2][2] < 0
