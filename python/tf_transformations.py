"""Subset of tf_transformations (quaternions are [x, y, z, w], axes 'sxyz')."""
import math


def quaternion_from_euler(ai, aj, ak, axes="sxyz"):
    cr, sr = math.cos(ai / 2), math.sin(ai / 2)
    cp, sp = math.cos(aj / 2), math.sin(aj / 2)
    cy, sy = math.cos(ak / 2), math.sin(ak / 2)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def euler_from_quaternion(q, axes="sxyz"):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def quaternion_multiply(q1, q0):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return [x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0]


def quaternion_inverse(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    return [-x / n, -y / n, -z / n, w / n]
