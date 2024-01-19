import math

import numpy as np
from geometry_msgs.msg import Quaternion
from ros2_numpy import msgify
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import mat2quat, qinverse, quat2mat, rotate_vector


def wxyz2xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    return quat_wxyz[[1, 2, 3, 0]]


def xyzw2wxyz(quat_xyzw: np.ndarray) -> np.ndarray:
    return quat_xyzw[[3, 0, 1, 2]]


def quat2sixd(quat_wxyz: np.ndarray) -> np.ndarray:
    # see https://openaccess.thecvf.com/content_CVPR_2019/supplemental/Zhou_On_the_Continuity_CVPR_2019_supplemental.pdf
    # first get matrix
    m = quat2mat(quat_wxyz)
    # 6D represenation is first 2 coloumns of matrix
    return m[[[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1]]]


def sixd2quat(sixd):
    # see https://openaccess.thecvf.com/content_CVPR_2019/supplemental/Zhou_On_the_Continuity_CVPR_2019_supplemental.pdf
    # compute the three column vectors
    a_1 = sixd[:3]
    a_2 = sixd[3:]
    b_1 = a_1 / np.linalg.norm(a_1)
    b_2_no_norm = a_2 - (np.dot(b_1, a_2) * b_1)
    b_2 = b_2_no_norm / np.linalg.norm(b_2_no_norm)
    b_3 = np.cross(b_1, b_2)
    # create matrix from column vectors
    mat = np.stack((b_1, b_2, b_3), axis=-1)
    return mat2quat(mat)


def quat2fused(q, order="wxyz"):
    # Check quaternion order
    if order == "xyzw":
        q_xyzw = q
    elif order == "wxyz":
        q_xyzw = wxyz2xyzw(q)
    else:
        raise ValueError(f"Unknown quaternion order: {order}")

    # Fused yaw of Quaternion
    fused_yaw = 2.0 * math.atan2(
        q_xyzw[2], q_xyzw[3]
    )  # Output of atan2 is [-tau/2,tau/2], so this expression is in [-tau,tau]
    if fused_yaw > math.tau / 2:
        fused_yaw -= math.tau  # fused_yaw is now in[-2* pi, pi]
    if fused_yaw <= -math.tau / 2:
        fused_yaw += math.tau  # fused_yaw is now in (-pi, pi]

    # Calculate the fused pitch and roll
    stheta = 2.0 * (q_xyzw[1] * q_xyzw[3] - q_xyzw[0] * q_xyzw[2])
    sphi = 2.0 * (q_xyzw[1] * q_xyzw[2] + q_xyzw[0] * q_xyzw[3])
    if stheta >= 1.0:  # Coerce stheta to[-1, 1]
        stheta = 1.0
    elif stheta <= -1.0:
        stheta = -1.0
    if sphi >= 1.0:  # Coerce sphi to[-1, 1]
        sphi = 1.0
    elif sphi <= -1.0:
        sphi = -1.0
    fused_pitch = math.asin(stheta)
    fused_roll = math.asin(sphi)

    # compute hemi parameter
    hemi = 0.5 - (q_xyzw[0] * q_xyzw[0] + q_xyzw[1] * q_xyzw[1]) >= 0.0
    return fused_roll, fused_pitch, fused_yaw, hemi


# Conversion: Fused angles (3D/4D) --> Quaternion (wxyz)
def fused2quat(fused_roll, fused_pitch, fused_yaw, hemi):
    # Precalculate the sine values
    sth = math.sin(fused_pitch)
    sphi = math.sin(fused_roll)

    # Calculate the sine sum criterion
    crit = sth * sth + sphi * sphi

    # Calculate the tilt angle alpha
    if crit >= 1.0:
        alpha = math.pi / 2
    else:
        if hemi:
            alpha = math.acos(math.sqrt(1.0 - crit))
        else:
            alpha = math.acos(-math.sqrt(1.0 - crit))

    # Calculate the tilt axis angle gamma
    gamma = math.atan2(sth, sphi)

    # Evaluate the required intermediate angles
    halpha = 0.5 * alpha
    hpsi = 0.5 * fused_yaw
    hgampsi = gamma + hpsi

    # Precalculate trigonometric terms involved in the quaternion expression
    chalpha = math.cos(halpha)
    shalpha = math.sin(halpha)
    chpsi = math.cos(hpsi)
    shpsi = math.sin(hpsi)
    chgampsi = math.cos(hgampsi)
    shgampsi = math.sin(hgampsi)

    # Calculate and return the required quaternion
    return np.array([chalpha * chpsi, shalpha * chgampsi, shalpha * shgampsi, chalpha * shpsi])  # Order: (w,x,y,z)


def compute_imu_orientation_from_world(robot_quat_in_world):
    # imu orientation has roll and pitch relative to gravity vector. yaw in world frame
    # get global yaw
    yrp_world_frame = quat2euler(robot_quat_in_world, axes="szxy")
    # remove global yaw rotation from roll and pitch
    yaw_quat = euler2quat(yrp_world_frame[0], 0, 0, axes="szxy")
    rp = rotate_vector((yrp_world_frame[1], yrp_world_frame[2], 0), qinverse(yaw_quat))
    # save in correct order
    return [rp[0], rp[1], 0], yaw_quat


def quat_from_yaw(yaw: float) -> Quaternion:
    return msgify(Quaternion, wxyz2xyzw(euler2quat(yaw, 0, 0, axes="szxy")))
