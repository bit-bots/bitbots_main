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

#include <wolfgang_ik/ik.h>

// ── Main ───────────────────────────────────────────────────────────────────
int main() {
    const double PI = M_PI;

    // ── Test foot subproblem ───────────────────────────────────────────────
    {
        Vec3 leg_vector(1.0, 1.0, 0.0);
        Mat4 T = foot_to_leg_from_vec_subproblem(leg_vector);
        auto euler = mat2euler_sxyz(T.block<3,3>(0,0));
        std::cout << "Ankle Joint Angles from Foot Vector:\n";
        std::cout << "Roll:  " << euler[0] * 180.0/PI << "\n";
        std::cout << "Pitch: " << euler[1] * 180.0/PI << "\n";
        std::cout << "Yaw:   " << euler[2] * 180.0/PI << "\n\n";
    }

    // ── Zero configuration test ────────────────────────────────────────────
    Eigen::VectorXd q(6);
    q << 0.0, 0.0, 0.0, -0.5, 0.0, 0.0;
    q *= PI;

    JointAngles joint_angles = {
        {"HipYaw",    q(0)},
        {"HipRoll",   q(1)},
        {"HipPitch",  q(2)},
        {"Knee", q(3)},
        {"AnklePitch",q(4)},
        {"AnkleRoll", q(5)}
    };

    std::cout << "Joint Angles:\n";
    for (auto& [k,v] : joint_angles)
        std::cout << "  " << k << ": " << v << "\n";

    // Test goal leg length (knee-to-foot in leg frame)
    {
        Mat4 T_knee = Mat4::Identity();
        T_knee.block<3,3>(0,0) = axangle2mat({0,1,0}, q(3));
        T_knee.block<3,1>(0,3) = Vec3(0, 0, -UPPER_LEG_LENGTH);
        Mat4 T_lower = Mat4::Identity();
        T_lower.block<3,1>(0,3) = Vec3(0, 0, -LOWER_LEG_LENGTH);
        Mat4 T_combined = T_knee * T_lower;
        std::cout << "\nGoal leg length: " << T_combined.block<3,1>(0,3).norm() << "\n\n";
    }

    // FK
    Mat4 T_fk = calculate_fk(joint_angles);
    std::cout << "Forward Kinematics:\n" << T_fk << "\n";
    auto fk_euler = mat2euler_sxyz(T_fk.block<3,3>(0,0));
    std::cout << "Foot orientation (sxyz): "
              << fk_euler[0] << ", " << fk_euler[1] << ", " << fk_euler[2] << "\n";
    std::cout << "Goal distance: " << T_fk.block<3,1>(0,3).norm() << "\n\n";

    // IK
    JointAngles q_ik = calculate_ik(T_fk, true);
    std::cout << "\nInverse Kinematics Joint Angles:\n";
    for (auto& [k,v] : q_ik)
        std::cout << "  " << k << ": " << v << "\n";

    // Verify FK from IK result
    Mat4 T_fk_ik = calculate_fk(q_ik);
    std::cout << "\nForward Kinematics from IK:\n" << T_fk_ik << "\n";
    double pos_err = (T_fk.block<3,1>(0,3) - T_fk_ik.block<3,1>(0,3)).norm();
    double rot_err = (T_fk.block<3,3>(0,0) - T_fk_ik.block<3,3>(0,0)).norm();
    std::cout << "Position error:    " << pos_err << "\n";
    std::cout << "Orientation error: " << rot_err << "\n\n";

    // ── Random test (1000 iterations) ─────────────────────────────────────
    std::cout << "Running 1000 random IK tests...\n";
    std::srand(42);
    std::vector<double> errors;
    int skipped = 0;
    for (int iter = 0; iter < 1000; ++iter) {
        Eigen::VectorXd q_rand(6);
        for (int i = 0; i < 6; ++i)
            q_rand(i) = (static_cast<double>(std::rand()) / RAND_MAX * 2.0 - 1.0) * 0.4 * PI;

        JointAngles ja = {
            {"HipYaw",    q_rand(0)},
            {"HipRoll",   q_rand(1)},
            {"HipPitch",  q_rand(2)},
            {"Knee", q_rand(3)},
            {"AnklePitch",q_rand(4)},
            {"AnkleRoll", q_rand(5)}
        };

        Mat4 T_rand = calculate_fk(ja);

        JointAngles q_ik_rand;
        try {
            q_ik_rand = calculate_ik(T_rand, true);
        } catch (const SolverError& e) {
            ++skipped;
            continue;
        }

        Mat4 T_ik_rand = calculate_fk(q_ik_rand);
        double err = (T_rand.block<3,1>(0,3) - T_ik_rand.block<3,1>(0,3)).norm()
                   + (T_rand.block<3,3>(0,0) - T_ik_rand.block<3,3>(0,0)).norm();

        if (std::isnan(err)) {
            std::cout << "NaN error at iteration " << iter << ", skipping.\n";
            ++skipped;
            continue;
        }
        if (err > 0.001) {
            std::cout << "High error " << err << " at iteration " << iter
                      << " for q = " << q_rand.transpose() << "\n";
            break;
        }
        errors.push_back(err);
    }

    if (!errors.empty()) {
        double sum = 0.0, max_err = 0.0;
        for (double e : errors) { sum += e; max_err = std::max(max_err, e); }
        std::cout << "Mean error: " << sum / errors.size() << "\n";
        std::cout << "Max error:  " << max_err << "\n";
    }
    std::cout << "Skipped: " << skipped << " / 1000\n";

    return 0;
}
