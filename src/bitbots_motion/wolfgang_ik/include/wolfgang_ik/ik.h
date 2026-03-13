/**
 * Leg Inverse Kinematics Solver - C++ port of the Python implementation.
 *
 * Euler angle conventions used (all are from transforms3d, verified analytically):
 *
 *   euler2mat / mat2euler 'sxyz':
 *     R = Rz(c) * Ry(b) * Rx(a)      (static/extrinsic, applied x then y then z)
 *     Decompose: b = arcsin(-R[2,0])
 *                a = arctan2( R[2,1],  R[2,2])
 *                c = arctan2( R[1,0],  R[0,0])
 *
 *   euler2mat / mat2euler 'rzxy':
 *     R = Rz(a) * Rx(b) * Ry(c)
 *     Decompose: b = arcsin( R[2,1])
 *                a = arctan2(-R[0,1],  R[1,1])
 *                c = arctan2(-R[2,0],  R[2,2])
 *
 *   euler2mat / mat2euler 'rxyz':
 *     R = Rx(a) * Ry(b) * Rz(c)
 *     Decompose: b = arcsin( R[0,2])
 *                a = arctan2(-R[1,2],  R[2,2])
 *                c = arctan2(-R[0,1],  R[0,0])
 *
 * All matrix indices are row-major (M[row][col]).
 */

#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

// ── Model constants ────────────────────────────────────────────────────────
constexpr double UPPER_LEG_LENGTH  = 0.1688;  // Hip to knee  [m]
constexpr double LOWER_LEG_LENGTH  = 0.17;  // Knee to ankle [m]
constexpr double HIP_PITCH_OFFSET  = 0.023;  // Hip-pitch joint offset [m]



// Motor offsets (left leg)
constexpr double HIP_PITCH_ANGLE_OFFSET = 0.025795568467720936;
constexpr double KNEE_OFFSET = 0.26180265358979327;



using Mat4 = Eigen::Matrix4d;
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

constexpr double BASE_LINK_SIDE_OFFSET = 0.055;
constexpr double BASE_LINK_Z_OFFSET = -0.0414;
constexpr double SOLE_X_OFFSET = 0.0216683;
constexpr double SOLE_Y_OFFSET = 0.0152219;
constexpr double SOLE_Z_OFFSET = -0.0529;

// ── Custom exception ───────────────────────────────────────────────────────
class SolverError : public std::runtime_error {
public:
    explicit SolverError(const std::string& msg) : std::runtime_error(msg) {}
};

// ── Elementary rotation matrices ───────────────────────────────────────────
static Mat3 Rx(double t) {
    Mat3 R = Mat3::Identity();
    R(1,1) =  std::cos(t); R(1,2) = -std::sin(t);
    R(2,1) =  std::sin(t); R(2,2) =  std::cos(t);
    return R;
}
static Mat3 Ry(double t) {
    Mat3 R = Mat3::Identity();
    R(0,0) =  std::cos(t); R(0,2) =  std::sin(t);
    R(2,0) = -std::sin(t); R(2,2) =  std::cos(t);
    return R;
}
static Mat3 Rz(double t) {
    Mat3 R = Mat3::Identity();
    R(0,0) =  std::cos(t); R(0,1) = -std::sin(t);
    R(1,0) =  std::sin(t); R(1,1) =  std::cos(t);
    return R;
}

// ── Euler composition helpers ──────────────────────────────────────────────

/**
 * euler2mat 'sxyz'(a, b, c) = Rz(c) * Ry(b) * Rx(a)
 */
static Mat3 euler2mat_sxyz(double a, double b, double c) {
    return Rz(c) * Ry(b) * Rx(a);
}

/**
 * mat2euler 'sxyz'(M) → (a, b, c)  such that  M = Rz(c)*Ry(b)*Rx(a)
 *   b = arcsin(-M[2,0])
 *   a = arctan2( M[2,1],  M[2,2])
 *   c = arctan2( M[1,0],  M[0,0])
 */
static std::array<double,3> mat2euler_sxyz(const Mat3& M) {
    double b = std::asin(std::clamp(-M(2,0), -1.0, 1.0));
    double a = std::atan2( M(2,1),  M(2,2));
    double c = std::atan2( M(1,0),  M(0,0));
    return {a, b, c};
}

/**
 * euler2mat 'rzxy'(a, b, c) = Rz(a) * Rx(b) * Ry(c)
 */
static Mat3 euler2mat_rzxy(double a, double b, double c) {
    return Rz(a) * Rx(b) * Ry(c);
}

/**
 * mat2euler 'rzxy'(M) → (a, b, c)  such that  M = Rz(a)*Rx(b)*Ry(c)
 *   b = arcsin( M[2,1])
 *   a = arctan2(-M[0,1],  M[1,1])
 *   c = arctan2(-M[2,0],  M[2,2])
 */
static std::array<double,3> mat2euler_rzxy(const Mat3& M) {
    double b = std::asin(std::clamp(M(2,1), -1.0, 1.0));
    double a = std::atan2(-M(0,1),  M(1,1));
    double c = std::atan2(-M(2,0),  M(2,2));
    return {a, b, c};
}

/**
 * euler2mat 'rxyz'(a, b, c) = Rx(a) * Ry(b) * Rz(c)
 */
static Mat3 euler2mat_rxyz(double a, double b, double c) {
    return Rx(a) * Ry(b) * Rz(c);
}

/**
 * mat2euler 'rxyz'(M) → (a, b, c)  such that  M = Rx(a)*Ry(b)*Rz(c)
 *   b = arcsin( M[0,2])
 *   a = arctan2(-M[1,2],  M[2,2])
 *   c = arctan2(-M[0,1],  M[0,0])
 */
static std::array<double,3> mat2euler_rxyz(const Mat3& M) {
    double b = std::asin(std::clamp(M(0,2), -1.0, 1.0));
    double a = std::atan2(-M(1,2),  M(2,2));
    double c = std::atan2(-M(0,1),  M(0,0));
    return {a, b, c};
}

// ── axangle2mat ────────────────────────────────────────────────────────────
/**
 * Rotation matrix from axis-angle (axis need not be unit; if zero vector, identity).
 */
static Mat3 axangle2mat(Vec3 axis, double angle) {
    double n = axis.norm();
    if (n < 1e-12) return Mat3::Identity();
    axis /= n;
    // Rodrigues' formula
    Mat3 K;
    K <<         0, -axis(2),  axis(1),
          axis(2),         0, -axis(0),
         -axis(1),  axis(0),        0;
    return Mat3::Identity() + std::sin(angle) * K + (1.0 - std::cos(angle)) * K * K;
}

// ── foot_to_leg_from_vec_subproblem ────────────────────────────────────────
/**
 * Returns a 4×4 transform T whose rotation aligns a reference frame to
 * the given leg_vector direction.
 *
 * Mirrors the Python:
 *   roll  = arctan2(v[1], v[2])
 *   pitch = arctan2(-v[0], sqrt(v[1]²+v[2]²))
 *   yaw   = 0
 *   R     = euler2mat(roll, pitch, yaw, 'sxyz') = Rz(0)*Ry(pitch)*Rx(roll)
 *                                               = Ry(pitch)*Rx(roll)
 */
static Mat4 foot_to_leg_from_vec_subproblem(const Vec3& leg_vector_in) {
    Vec3 v = leg_vector_in.normalized();

    if (std::abs(v(0)) > 0.99 || std::abs(v(1)) > 0.99) {
        throw SolverError(
            "Leg vector is too close to the x or y axis, cannot determine roll and pitch.");
    }

    double roll  = std::atan2(v(1), v(2));
    double pitch = std::atan2(-v(0), std::sqrt(v(1)*v(1) + v(2)*v(2)));
    double yaw   = 0.0;

    Mat4 T = Mat4::Identity();
    T.block<3,3>(0,0) = euler2mat_sxyz(roll, pitch, yaw);
    return T;
}

// ── Joint angle map type ───────────────────────────────────────────────────
using JointAngles = std::map<std::string, double>;

// ── Forward kinematics ─────────────────────────────────────────────────────
/**
 * Mirrors calculate_fk() exactly.
 * Joint order (all around standard axes):
 *   HipYaw   → Rz
 *   HipRoll  → Rx
 *   HipPitch → Ry  (+ translation HIP_PITCH_OFFSET along local x)
 *   Knee→ Ry  (+ translation -UPPER_LEG_LENGTH along local z)
 *   AnklePitch→Ry  (+ translation -LOWER_LEG_LENGTH along local z)
 *   AnkleRoll→ Rx
 */
static Mat4 calculate_fk(const JointAngles& q) {
    auto make_T = [](const Mat3& R, const Vec3& t = Vec3::Zero()) {
        Mat4 T = Mat4::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        return T;
    };

    Mat4 T = Mat4::Identity();

    // Base link transform (translation from base_link to hip axis intersection)
    T = T * make_T(Mat3::Identity(), {BASE_LINK_SIDE_OFFSET, BASE_LINK_Z_OFFSET, 0});

    // HipYaw: rotation about Z
    T = T * make_T(axangle2mat({0,0,1}, q.at("HipYaw")));

    // HipRoll: rotation about X
    T = T * make_T(axangle2mat({1,0,0}, q.at("HipRoll")));

    // HipPitch: rotation about Y + offset along x
    T = T * make_T(axangle2mat({0,1,0}, q.at("HipPitch")), {HIP_PITCH_OFFSET, 0, 0});

    // Knee: rotation about Y + translation down z
    T = T * make_T(axangle2mat({0,1,0}, q.at("Knee")), {0, 0, -UPPER_LEG_LENGTH});

    // AnklePitch: rotation about Y + translation down z
    T = T * make_T(axangle2mat({0,1,0}, q.at("AnklePitch")), {0, 0, -LOWER_LEG_LENGTH});

    // AnkleRoll: rotation about X
    T = T * make_T(axangle2mat({1,0,0}, q.at("AnkleRoll")));

    // End effector offset
    T = T * make_T(Mat3::Identity(), {SOLE_X_OFFSET, SOLE_Y_OFFSET, SOLE_Z_OFFSET});

    return T;
}

// ── Inverse kinematics ─────────────────────────────────────────────────────
static JointAngles calculate_ik(const Mat4& target_transform, bool left) {
    JointAngles joint_angles = {
        {"HipYaw",    0.0},
        {"HipRoll",   0.0},
        {"HipPitch",  0.0},
        {"Knee", 0.0},
        {"AnklePitch",0.0},
        {"AnkleRoll", 0.0}
    };

    std::cout << "Target transformation matrix:\n" << target_transform << "\n";

    auto make_T = [](const Mat3& R, const Vec3& t = Vec3::Zero()) {
        Mat4 T = Mat4::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        return T;
    };

    // Remove the ends of the kinematic chain up to the first and last joint
    Mat4 T_ankle_to_sole, T_base_link_to_hip_intersection;

    if (left) {
        T_ankle_to_sole = make_T(Mat3::Identity(), {SOLE_X_OFFSET, SOLE_Y_OFFSET, SOLE_Z_OFFSET});
        T_base_link_to_hip_intersection = make_T(Mat3::Identity(), {0, BASE_LINK_SIDE_OFFSET, BASE_LINK_Z_OFFSET});
    } else {
        T_ankle_to_sole = make_T(Mat3::Identity(), {SOLE_X_OFFSET, -SOLE_Y_OFFSET, SOLE_Z_OFFSET});
        T_base_link_to_hip_intersection = make_T(Mat3::Identity(), {0, -BASE_LINK_SIDE_OFFSET, BASE_LINK_Z_OFFSET});
    }

    auto target_transform_leg_only = (T_base_link_to_hip_intersection.inverse() * target_transform) * T_ankle_to_sole.inverse();

    std::cout << "Target transformation matrix (leg only):\n" << target_transform_leg_only << "\n";

    // Axis-intersection transforms (identity = at origin)
    Mat4 T_hip_axis_intersection   = Mat4::Identity();
    Mat4 T_ankle_axis_intersection = target_transform_leg_only;

    // ankle-to-hip transform
    Mat4 T_ankle_to_hip = T_ankle_axis_intersection.inverse() * T_hip_axis_intersection;
    std::cout << "Ankle to hip:\n" << T_ankle_to_hip << "\n";

    // ankle-to-leg alignment (inverse of foot_to_leg subproblem)
    Vec3 leg_vec = T_ankle_to_hip.block<3,1>(0,3);
    Mat4 T_ankle_to_leg = foot_to_leg_from_vec_subproblem(leg_vec).inverse();

    // hip-to-leg alignment
    Mat4 T_hip_to_leg = T_ankle_to_hip.inverse() * T_ankle_to_leg;
    std::cout << "Hip to leg:\n" << T_hip_to_leg << "\n";

    // HipYaw and HipRoll from rzxy decomposition: M = Rz(yaw)*Rx(roll)*Ry(pitch)
    auto hip_euler = mat2euler_rzxy(T_hip_to_leg.block<3,3>(0,0));
    joint_angles["HipYaw"]  = hip_euler[0];
    joint_angles["HipRoll"] = hip_euler[1];
    // hip_euler[2] would be HipPitch, but we compute it properly below
    std::cout << "Hip angles (spherical): " << joint_angles["HipYaw"]
              << ", " << joint_angles["HipRoll"] << "\n";

    // Virtual leg length (distance from ankle axis intersection to hip axis intersection)
    double virtual_leg_length = leg_vec.norm();
    std::cout << "Leg length: " << virtual_leg_length << "\n";

    // Hip transform without pitch component
    Mat4 T_hip_without_pitch = Mat4::Identity();
    T_hip_without_pitch.block<3,3>(0,0) =
        euler2mat_rzxy(joint_angles["HipYaw"], joint_angles["HipRoll"], 0.0);

    // Hip pitch offset transform
    Mat4 T_hip_pitch_offset = Mat4::Identity();
    T_hip_pitch_offset.block<3,1>(0,3) = Vec3(HIP_PITCH_OFFSET, 0, 0);

    Mat4 T_hip_pitch_origin = T_hip_without_pitch * T_hip_pitch_offset;

    // Real leg: from ankle to hip-pitch origin
    Mat4 T_real_leg = T_ankle_axis_intersection.inverse()
                     * T_hip_axis_intersection
                     * T_hip_pitch_origin;
    std::cout << "Real leg transformation matrix:\n" << T_real_leg << "\n";

    double real_leg_length = T_real_leg.block<3,1>(0,3).norm();
    std::cout << "Real leg length: " << real_leg_length << "\n";

    // Ankle angles from real-leg direction
    Vec3 real_leg_vec = T_real_leg.block<3,1>(0,3);
    Mat4 T_ankle_to_real_leg = foot_to_leg_from_vec_subproblem(real_leg_vec).inverse();

    // mat2euler 'rxyz': M = Rx(a)*Ry(b)*Rz(c)
    auto ankle_euler = mat2euler_rxyz(T_ankle_to_real_leg.block<3,3>(0,0));
    joint_angles["AnkleRoll"]  = -ankle_euler[0];
    joint_angles["AnklePitch"] = -ankle_euler[1];
    std::cout << "Ankle to real leg: " << joint_angles["AnkleRoll"]
              << ", " << joint_angles["AnklePitch"] << "\n";

    // Knee from cosine rule
    double knee_arg = (real_leg_length*real_leg_length
                       - UPPER_LEG_LENGTH*UPPER_LEG_LENGTH
                       - LOWER_LEG_LENGTH*LOWER_LEG_LENGTH)
                      / (2.0 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
    joint_angles["Knee"] = std::acos(std::clamp(knee_arg, -1.0, 1.0));

    // HipPitch base from rzxy[2]
    Mat4 T_hip_to_leg_real = T_real_leg.inverse() * T_ankle_to_real_leg;
    auto hip_euler_real = mat2euler_rzxy(T_hip_to_leg_real.block<3,3>(0,0));
    double hip_pitch_base = hip_euler_real[2];

    // Hip pitch offset (cosine rule)
    double hip_pitch_offset_arg = (UPPER_LEG_LENGTH*UPPER_LEG_LENGTH
                                   + real_leg_length*real_leg_length
                                   - LOWER_LEG_LENGTH*LOWER_LEG_LENGTH)
                                   / (2.0 * UPPER_LEG_LENGTH * real_leg_length);
    double hip_pitch_offset_val = std::acos(std::clamp(hip_pitch_offset_arg, -1.0, 1.0));
    std::cout << "Hip pitch offset: " << hip_pitch_offset_val << "\n";

    // Ankle pitch offset (cosine rule)
    double ankle_pitch_offset_arg = (LOWER_LEG_LENGTH*LOWER_LEG_LENGTH
                                     + real_leg_length*real_leg_length
                                     - UPPER_LEG_LENGTH*UPPER_LEG_LENGTH)
                                     / (2.0 * LOWER_LEG_LENGTH * real_leg_length);
    double ankle_pitch_offset_val = std::acos(std::clamp(ankle_pitch_offset_arg, -1.0, 1.0));
    std::cout << "Ankle pitch offset: " << ankle_pitch_offset_val << "\n";

    // Solution i=0 (matches Python solutions[0])
    joint_angles["HipPitch"]   = hip_pitch_base - hip_pitch_offset_val;
    joint_angles["AnklePitch"] += -ankle_pitch_offset_val;  // i=0: subtract

    return joint_angles;
}
