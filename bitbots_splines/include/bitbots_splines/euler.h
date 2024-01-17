/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef EULER_H
#define EULER_H

#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace bitbots_splines {

/**
 * All combinations
 * of Euler angles types
 * in same order as rotation application
 *
 * EulerYawPitchRoll is built as
 * Roll * Pitch * Yaw.
 */
enum EulerType {
  EulerYawPitchRoll,
  EulerYawRollPitch,
  EulerRollPitchYaw,
  EulerRollYawPitch,
  EulerPitchRollYaw,
  EulerPitchYawRoll,
};

/**
 * Valid Euler angles (0, 1, 2) range
 * are (bound included):
 * 0: -M_PI : M_PI
 * 1: -M_PI/2.0 : M_PI/2.0
 * 2: 0.0 : M_PI
 */

/**
 * Return false if given Euler angles range are no valid
 */
inline bool CheckEulerBounds(const Eigen::Vector3d &angles) {
  return
      (angles(0) >= -M_PI && angles(0) <= M_PI) &&
          ((angles(1) > -M_PI / 2.0 && angles(1) < M_PI / 2.0) ||
              (angles(0) == 0 && angles(2) == 0 &&
                  (angles(1) == -M_PI / 2.0 || angles(1) == M_PI / 2.0))) &&
          (angles(2) >= 0.0 && angles(2) <= M_PI);
}

/**
 * Convert given Euler angles of given
 * convention type to rotation matrix
 */
inline Eigen::Matrix3d EulerToMatrix(
    const Eigen::Vector3d &angles, EulerType eulerType) {
  Eigen::Quaternion<double> quat;
  switch (eulerType) {
    case EulerYawPitchRoll: {
      Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
      quat = rollRot * pitchRot * yawRot;
    }
      break;
    case EulerYawRollPitch: {
      Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
      quat = pitchRot * rollRot * yawRot;
    }
      break;
    case EulerRollPitchYaw: {
      Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
      quat = yawRot * pitchRot * rollRot;
    }
      break;
    case EulerRollYawPitch: {
      Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
      quat = pitchRot * yawRot * rollRot;
    }
      break;
    case EulerPitchRollYaw: {
      Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
      quat = yawRot * rollRot * pitchRot;
    }
      break;
    case EulerPitchYawRoll: {
      Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
      quat = rollRot * yawRot * pitchRot;
    }
      break;
    default: {
      throw std::logic_error("Euler invalid type");
    }
  }
  return quat.matrix();
}

/**
 * Convert the given rotation matrix into
 * Euler angles of given convention
 */
inline Eigen::Vector3d MatrixToEuler(
    const Eigen::Matrix3d &mat, EulerType eulerType) {
  Eigen::Vector3d tmp(0.0, 0.0, 0.0);
  switch (eulerType) {
    case EulerYawPitchRoll: {
      tmp = mat.eulerAngles(0, 1, 2);
    }
      break;
    case EulerYawRollPitch: {
      tmp = mat.eulerAngles(1, 0, 2);
    }
      break;
    case EulerRollPitchYaw: {
      tmp = mat.eulerAngles(2, 1, 0);
    }
      break;
    case EulerRollYawPitch: {
      tmp = mat.eulerAngles(1, 2, 0);
    }
      break;
    case EulerPitchRollYaw: {
      tmp = mat.eulerAngles(2, 0, 1);
    }
      break;
    case EulerPitchYawRoll: {
      tmp = mat.eulerAngles(0, 2, 1);
    }
      break;
  }
  Eigen::Vector3d angles;
  angles(0) = tmp(2);
  angles(1) = tmp(1);
  angles(2) = tmp(0);
  return angles;
}

/**
 * Manually convert the given rotation matrix
 * to [Roll, Pitch, Yaw] ZYX intrinsic euler 
 * angle (Better range than Eigen conversion).
 */
inline Eigen::Vector3d MatrixToEulerIntrinsic(
    const Eigen::Matrix3d &mat) {
  //Eigen euler angles and with better range)
  Eigen::Vector3d angles;
  //Roll
  angles.x() = atan2(mat(2, 1), mat(2, 2));
  //Pitch
  angles.y() = atan2(-mat(2, 0),
                     sqrt(mat(0, 0) * mat(0, 0)
                              + mat(1, 0) * mat(1, 0)));
  //Yaw
  angles.z() = atan2(mat(1, 0), mat(0, 0));

  return angles;
}

/**
 * Convert given Euler angles in [Roll, Pitch, Yaw]
 * ZYX intrinsic format to rotation matrix
 */
inline Eigen::Matrix3d EulerIntrinsicToMatrix(
    const Eigen::Vector3d &angles) {
  Eigen::AngleAxisd yawRot(angles.z(), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchRot(angles.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollRot(angles.x(), Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> quat = yawRot * pitchRot * rollRot;
  return quat.matrix();
}

}

#endif

