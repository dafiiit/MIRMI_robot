"""Rigid-body transform and quaternion utilities for the docking test suite."""

import numpy as np
import cv2


# ---------------------------------------------------------------------------
# Quaternion helpers  (convention: [qx, qy, qz, qw])
# ---------------------------------------------------------------------------

def quat_to_rotation_matrix(q):
    """Convert quaternion [qx, qy, qz, qw] to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y],
    ])


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion [qx, qy, qz, qw]."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quat_multiply(q1, q2):
    """Multiply two quaternions [qx, qy, qz, qw]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def quat_conjugate(q):
    """Return the conjugate (inverse for unit quaternions) [qx, qy, qz, qw]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


# ---------------------------------------------------------------------------
# 4x4 homogeneous transforms
# ---------------------------------------------------------------------------

def pose_to_matrix(position, orientation):
    """Build a 4x4 transform from position [x,y,z] and quaternion [qx,qy,qz,qw]."""
    T = np.eye(4)
    T[:3, :3] = quat_to_rotation_matrix(orientation)
    T[:3, 3] = position
    return T


def matrix_to_pose(T):
    """Extract position [x,y,z] and quaternion [qx,qy,qz,qw] from 4x4 transform."""
    pos = T[:3, 3].copy()
    q = rotation_matrix_to_quat(T[:3, :3])
    return pos, q


def pose_msg_to_matrix(pose_msg):
    """Convert a geometry_msgs/Pose (or .pose from PoseStamped) to 4x4 matrix."""
    p = pose_msg.position
    o = pose_msg.orientation
    return pose_to_matrix(
        [p.x, p.y, p.z],
        [o.x, o.y, o.z, o.w],
    )


def build_offset_matrix(cfg_offset):
    """Build 4x4 transform from a config offset dict {translation, rotation}."""
    t = cfg_offset['translation']
    q = cfg_offset['rotation']
    return pose_to_matrix(t, q)


# ---------------------------------------------------------------------------
# Relative-pose computation with calibration offsets
# ---------------------------------------------------------------------------

def compute_ground_truth_relative_pose(
    T_world_robot, T_world_target,
    T_robot_to_cam, T_target_offset,
):
    """Compute the ground-truth relative pose of the real target in camera frame.

    T_camera_target = inv(T_world_robot * T_robot_to_cam)
                       * (T_world_target * T_target_offset)

    Returns (position [3], quaternion [4]) in camera frame.
    """
    T_world_cam = T_world_robot @ T_robot_to_cam
    T_world_real_target = T_world_target @ T_target_offset
    T_cam_target = np.linalg.inv(T_world_cam) @ T_world_real_target
    return matrix_to_pose(T_cam_target)


# ---------------------------------------------------------------------------
# solvePnP wrapper
# ---------------------------------------------------------------------------

def estimate_tag_pose_pnp(corners_px, camera_matrix, dist_coeffs, tag_size):
    """Estimate AprilTag 6-DoF pose from 2-D corner detections via solvePnP.

    Parameters
    ----------
    corners_px : array-like, shape (4, 2)
        Pixel coordinates of the four tag corners.
    camera_matrix : np.ndarray (3, 3)
    dist_coeffs : np.ndarray
    tag_size : float
        Physical tag size in metres.

    Returns
    -------
    tvec : np.ndarray (3,) or None
    rvec : np.ndarray (3,) or None
    R    : np.ndarray (3,3) or None
    """
    s = tag_size / 2.0
    obj_points = np.array([
        [-s,  s, 0],
        [ s,  s, 0],
        [ s, -s, 0],
        [-s, -s, 0],
    ], dtype=np.float32)

    image_points = np.asarray(corners_px, dtype=np.float32).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(
        obj_points, image_points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None, None, None

    R, _ = cv2.Rodrigues(rvec)
    return tvec.flatten(), rvec.flatten(), R


# ---------------------------------------------------------------------------
# Euler helpers (for human-readable error reporting)
# ---------------------------------------------------------------------------

def rotation_matrix_to_euler(R):
    """Extract roll, pitch, yaw (in degrees) from rotation matrix (ZYX convention)."""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0.0
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def compute_pose_error(pos_gt, pos_est):
    """Compute position error components and Euclidean distance."""
    err = np.asarray(pos_est) - np.asarray(pos_gt)
    return err[0], err[1], err[2], float(np.linalg.norm(err))


def yaw_from_quaternion(q):
    """Extract yaw (degrees) from quaternion [qx, qy, qz, qw]."""
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.degrees(np.arctan2(siny_cosp, cosy_cosp))
