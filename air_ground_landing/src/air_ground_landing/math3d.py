"""Small coordinate helpers kept independent from NumPy and ROS."""

from __future__ import annotations

import math
from typing import Iterable

from .models import Quaternion, Vector3


Matrix3 = tuple[Vector3, Vector3, Vector3]


def clamp(value: float, lower: float, upper: float) -> float:
    if lower > upper:
        raise ValueError("lower must not exceed upper")
    return max(lower, min(upper, value))


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def subtract(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(vector: Vector3, factor: float) -> Vector3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def norm(vector: Vector3) -> float:
    return math.sqrt(sum(component * component for component in vector))


def horizontal_norm(vector: Vector3) -> float:
    return math.hypot(vector[0], vector[1])


def finite_vector(values: Iterable[float]) -> Vector3:
    converted = tuple(float(value) for value in values)
    if len(converted) != 3 or not all(math.isfinite(value) for value in converted):
        raise ValueError("expected three finite vector components")
    return converted  # type: ignore[return-value]


def validate_rotation(matrix: Iterable[Iterable[float]]) -> Matrix3:
    rows = tuple(finite_vector(row) for row in matrix)
    if len(rows) != 3:
        raise ValueError("rotation matrix must be 3x3")
    for row_index in range(3):
        for other_index in range(3):
            dot = sum(rows[row_index][column] * rows[other_index][column] for column in range(3))
            expected = 1.0 if row_index == other_index else 0.0
            if abs(dot - expected) > 1.0e-5:
                raise ValueError("rotation matrix must be orthonormal")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(determinant - 1.0) > 1.0e-5:
        raise ValueError("rotation matrix determinant must be +1")
    return rows  # type: ignore[return-value]


def transform_point(rotation: Matrix3, translation: Vector3, point: Vector3) -> Vector3:
    return tuple(
        sum(rotation[row][column] * point[column] for column in range(3)) + translation[row]
        for row in range(3)
    )  # type: ignore[return-value]


def rotate_by_quaternion(vector: Vector3, quaternion: Quaternion) -> Vector3:
    """Rotate a BODY_FRD vector into LOCAL_NED with a w,x,y,z quaternion."""

    w, x, y, z = (float(value) for value in quaternion)
    length = math.sqrt(w * w + x * x + y * y + z * z)
    if length <= 1.0e-9 or not math.isfinite(length):
        raise ValueError("invalid body-to-NED quaternion")
    w, x, y, z = w / length, x / length, y / length, z / length
    vx, vy, vz = vector
    # Equivalent to q * [0,v] * conjugate(q), expanded to avoid dependencies.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def rotate_ned_to_body(vector: Vector3, quaternion_body_to_ned: Quaternion) -> Vector3:
    """Rotate a LOCAL_NED vector into BODY_FRD using the inverse quaternion."""

    w, x, y, z = (float(value) for value in quaternion_body_to_ned)
    return rotate_by_quaternion(vector, (w, -x, -y, -z))


def ros_enu_to_local_ned(vector_enu: Vector3) -> Vector3:
    """Convert an ENU vector after both systems have been given a common origin."""

    east, north, up = vector_enu
    return (north, east, -up)
