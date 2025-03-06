import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(BASE_DIR)

import numpy as np

cam2gripper_path = os.path.join(PARENT_DIR, "./params/cam2gripper.npz")
with np.load(cam2gripper_path) as X:
    R_cam2gripper, t_cam2gripper = [X[i]
                                    for i in ('R_cam2gripper', 't_cam2gripper')]


def create_robot_pose(x, y, z, theta):
    """
    Create a homogeneous transformation matrix from position and rotation (about Z-axis).

    Parameters:
    x, y, z : float  -> Translation in meters
    theta   : float  -> Rotation about the Z-axis in radians

    Returns:
    numpy.ndarray (4x4) -> Homogeneous transformation matrix
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    T_robot = np.array([
        [cos_t, -sin_t, 0, x],
        [sin_t, cos_t, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ], dtype=np.float32)
    return T_robot


def angle_thres(yaw):
    offset = 115
    while yaw > 180 + offset:
        yaw = yaw - 360
    while yaw < offset - 180:
        yaw = yaw + 360
    return yaw


def camera_to_gripper(kpt):
    kpt = kpt.reshape((-1, 3)).T
    kpt_gripper = R_cam2gripper @ kpt + t_cam2gripper
    kpt_gripper = kpt_gripper.T.squeeze()
    return kpt_gripper


def gripper_to_base(kpt, actual_pose):
    offset = np.array([0, 0, 50])
    x, y, z, theta = actual_pose
    theta = theta * np.pi / 180
    T_gripper2base = create_robot_pose(x, y, z, theta)
    R_gripper2base = T_gripper2base[:3, :3]
    t_gripper2base = T_gripper2base[:3, -1].reshape((3, 1))
    kpt = kpt.reshape((3, 1))
    kpt_base = R_gripper2base @ kpt + t_gripper2base
    return kpt_base.flatten() + offset


def kpt_matching(kpt_list, target):
    kpts_array = np.array(kpt_list)
    target_array = np.array(target)
    distances = np.linalg.norm(kpts_array[:,:2] - target_array[:2], axis=1)
    closest_index = np.argmin(distances)
    closest_point = kpt_list[closest_index]
    return closest_point


def kpt_trans(kpt, yaw, offset):
    # offset = 180  # mm
    yaw_radian = yaw * np.pi / 180

    kpt[0] += offset / 2 * np.cos(yaw_radian)
    kpt[1] += offset / 2 * np.sin(yaw_radian)
    return kpt

kpt_list = [np.array([ 83.66831426, -30.8121206 ,  0]), np.array([169.86603354, -82.89479143, 0]), np.array([125.4776092 , -47.54951216,  0])]
target = [91.19491196, -66.17415619, 0]
closest_point = kpt_matching(kpt_list, target)
print(closest_point)
