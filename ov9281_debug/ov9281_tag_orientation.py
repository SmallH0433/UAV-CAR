"""Measured tag -> common pad -> BODY_FRD pose, shared with the vision API.

Pad axes: X is the upright outer print's top, Y its right, Z into the pad.
PnP axes follow the existing pupil corner/object-point pairing (X print left,
Y print up, Z into the pad). The inner print is CCW 45 degrees on the paper.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np


def validate_rotation(value):
    r = np.asarray(value, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or not math.isclose(float(np.linalg.det(r)), 1.0, abs_tol=1e-6)):
        raise ValueError('camera-to-body must be a proper rotation')
    return r


def load_body_extrinsics(path: Path):
    config = json.loads(path.read_text(encoding='utf-8-sig'))
    camera = config['landing_target_bridge']['camera_to_body']
    r = validate_rotation(camera['rotation_camera_optical_to_body_frd'])
    ibvs = config['hybrid_guidance']['camera']['rotation_camera_optical_to_body_frd']
    if not np.allclose(r, validate_rotation(ibvs), atol=1e-6):
        raise ValueError('landing and IBVS camera-to-body rotations disagree')
    t = np.asarray(camera['translation_m'], dtype=float)
    if t.shape != (3,) or not np.isfinite(t).all():
        raise ValueError('invalid camera translation')
    return r, t


def quaternion_wxyz(rotation):
    # Rodrigues axis/angle avoids fragile trace branches at 180 degrees.
    vector = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        return [1.0, 0.0, 0.0, 0.0]
    xyz = vector * (math.sin(angle / 2) / angle)
    q = np.r_[math.cos(angle / 2), xyz]
    return (q / np.linalg.norm(q)).tolist()


def body_tag_orientation(rvec, translation_camera, role, size_m,
                         rotation_camera_to_body, translation_camera_in_body,
                         camera_matrix, distortion):
    if role not in ('outer', 'inner'):
        return None
    rct = validate_rotation(cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))[0])
    rbc = validate_rotation(rotation_camera_to_body)
    angle = math.pi / 4 if role == 'inner' else 0.0
    c, s = math.cos(angle), math.sin(angle)
    undo_inner = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
    # Columns express pad forward/right/down in the unrotated PnP tag frame.
    pad_to_tag = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    rcp = rct @ undo_inner @ pad_to_tag
    rbp = rbc @ rcp
    camera_position = np.asarray(translation_camera, dtype=float).reshape(3)
    offset = np.asarray(translation_camera_in_body, dtype=float).reshape(3)
    body_position = rbc @ camera_position + offset
    forward = rbp[:, 0]
    yaw = math.degrees(math.atan2(float(forward[1]), float(forward[0]))) if np.linalg.norm(forward[:2]) > 1e-6 else None
    # Render from the BODY pose by transforming back through the same extrinsic.
    # This keeps translation out of direction vectors and respects lens distortion.
    body_points = np.array([body_position, body_position + forward * size_m * .35])
    camera_points = (rbc.T @ (body_points - offset).T).T
    arrow = None
    if np.isfinite(camera_points).all() and np.all(camera_points[:, 2] > 0):
        projected = cv2.fisheye.projectPoints(
            camera_points.reshape(1, 2, 3), np.zeros((3, 1)), np.zeros((3, 1)),
            camera_matrix, distortion)[0].reshape(2, 2)
        if np.isfinite(projected).all():
            arrow = projected.tolist()
    return {
        'frame': 'BODY_FRD',
        'pad_axes': 'X=outer_print_top,Y=outer_print_right,Z=into_pad',
        'source': 'PNP_TAG_TO_PAD_TO_BODY',
        'inner_layout_ccw_deg': 45.0 if role == 'inner' else 0.0,
        'rotation_camera_optical_to_body_frd': rbc.tolist(),
        'rotation_pad_to_camera': rcp.tolist(),
        'rotation_pad_to_body_frd': rbp.tolist(),
        'quaternion_pad_to_body_frd_wxyz': quaternion_wxyz(rbp),
        'position_body_frd_m': body_position.tolist(),
        'raw_tag_up_body_frd': (rbc @ rct @ np.array([0., 1., 0.])).tolist(),
        'pad_forward_body_frd': forward.tolist(),
        'pad_heading_body_deg': yaw,
        'arrow_image_px': arrow,
        'yaw_control_enabled': False,
    }
