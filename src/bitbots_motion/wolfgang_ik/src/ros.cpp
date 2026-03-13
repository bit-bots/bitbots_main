/**
 * leg_ik_node.cpp
 *
 * ROS2 node that:
 *   1. Looks up the current l_sole → base_link TF transform.
 *   2. Runs the analytic IK solver on that target.
 *   3. Subscribes to /joint_states and prints IK solution vs. actual joint
 *      angles side-by-side in a loop.
 *
 * Build (add to CMakeLists.txt):
 *   find_package(Eigen3 REQUIRED)
 *   find_package(rclcpp REQUIRED)
 *   find_package(tf2_ros REQUIRED)
 *   find_package(tf2_eigen REQUIRED)
 *   find_package(sensor_msgs REQUIRED)
 *
 *   add_executable(leg_ik_node leg_ik_node.cpp)
 *   ament_target_dependencies(leg_ik_node rclcpp tf2_ros tf2_eigen sensor_msgs)
 *   target_include_directories(leg_ik_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
 */

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <wolfgang_ik/ik.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

using JointAngles = std::map<std::string, double>;


// ═══════════════════════════════════════════════════════════════════════════
//  ROS2 node
// ═══════════════════════════════════════════════════════════════════════════

// Joint names in the order we want to print them.
// Adjust these to match your robot's URDF joint names.
static const std::vector<std::string> JOINT_NAMES = {
    "RHipYaw", "RHipRoll", "RHipPitch",
    "RKnee",
    "RAnklePitch", "RAnkleRoll"
};

class LegIKNode : public rclcpp::Node {
public:
    LegIKNode() : Node("leg_ik_node") {
        // TF
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Joint state subscriber
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SystemDefaultsQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                last_js_ = msg;
            });

        // Timer — runs the main loop at ~10 Hz
        timer_ = create_wall_timer(100ms, [this]() { timerCb(); });

        RCLCPP_INFO(get_logger(), "leg_ik_node started. Waiting for TF and joint states...");
    }

private:
    void timerCb() {
        // ── 1. Look up l_sole → base_link ──────────────────────────────────
        geometry_msgs::msg::TransformStamped ts;
        try {
            ts = tf_buffer_->lookupTransform(
                "base_link",  // target frame
                "r_sole",     // source frame
                tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        // {
        //     geometry_msgs::msg::TransformStamped ts1;
        //     try {
        //         ts1 = tf_buffer_->lookupTransform(
        //             "l_hip_1",  // target frame
        //             "l_foot",     // source frame
        //             tf2::TimePointZero);
        //     } catch (const tf2::TransformException& ex) {
        //         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        //             "TF lookup failed: %s", ex.what());
        //         return;
        //     }


        //     std::cout << "TF lookup successful: base_link → l_foot" << ts1.transform.translation.z << "\n";
        // }


        // Convert to Eigen 4×4
        Eigen::Isometry3d iso = tf2::transformToEigen(ts);
        Mat4 T_target = iso.matrix();

        // ── 2. Run IK ───────────────────────────────────────────────────────
        JointAngles ik_solution;
        try {
            bool left = true;


            ik_solution = calculate_ik(T_target, left);

            // Apply joint offsets to match actual robot configuration (if needed)
            if (left){
                ik_solution["HipYaw"]   *= -1;
                ik_solution["HipPitch"]   *= -1;
                ik_solution["AnkleRoll"]       *= -1;
                ik_solution["HipPitch"]   += HIP_PITCH_ANGLE_OFFSET;
                ik_solution["Knee"]       += KNEE_OFFSET;
            } else {
                ik_solution["HipYaw"]   *= -1;
                ik_solution["HipPitch"]   -= HIP_PITCH_ANGLE_OFFSET;
                ik_solution["Knee"]       += KNEE_OFFSET;
                ik_solution["Knee"]       *= -1;
                ik_solution["AnklePitch"]   *= -1;
                ik_solution["AnkleRoll"]   *= -1;
            }
        } catch (const SolverError& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "IK solver error: %s", e.what());
            return;
        }

        // ── 3. Extract actual joint angles from /joint_states ───────────────
        JointAngles actual;
        if (last_js_) {
            for (const auto& name : JOINT_NAMES) {
                auto it = std::find(last_js_->name.begin(), last_js_->name.end(), name);
                if (it != last_js_->name.end()) {
                    size_t idx = std::distance(last_js_->name.begin(), it);
                    if (idx < last_js_->position.size())
                        actual[name] = last_js_->position[idx];
                }
            }
        }

        // ── 4. Print comparison ─────────────────────────────────────────────
        const int W = 12;
        const double RAD2DEG = 180.0 / M_PI;

        std::cout << "\n══════════════════════════════════════════════════════\n";
        std::cout << "  TF  base_link → l_sole\n";
        std::cout << "  pos [m]:   x=" << std::fixed << std::setprecision(4)
                  << ts.transform.translation.x << "  y=" << ts.transform.translation.y
                  << "  z=" << ts.transform.translation.z << "\n";
        std::cout << "  quat:      x=" << ts.transform.rotation.x
                  << "  y=" << ts.transform.rotation.y
                  << "  z=" << ts.transform.rotation.z
                  << "  w=" << ts.transform.rotation.w << "\n";
        std::cout << "──────────────────────────────────────────────────────\n";
        std::cout << std::setw(16) << "Joint"
                  << std::setw(W) << "IK [deg]"
                  << std::setw(W) << "IK [rad]"
                  << std::setw(W) << "Act [deg]"
                  << std::setw(W) << "Act [rad]"
                  << std::setw(W) << "Δ [deg]" << "\n";
        std::cout << "──────────────────────────────────────────────────────\n";

        for (const auto& name : JOINT_NAMES) {

            //Remove first character (L) for printing

            double ik_rad  = ik_solution.count(name.substr(1))  ? ik_solution.at(name.substr(1))  : -100.0;
            double act_rad = actual.count(name)        ? actual.at(name)       : -100.0;
            double delta   = (ik_rad - act_rad) * RAD2DEG;

            std::cout << std::setw(16) << name
                      << std::setw(W) << std::fixed << std::setprecision(3) << ik_rad * RAD2DEG
                      << std::setw(W) << std::setprecision(4) << ik_rad
                      << std::setw(W) << std::setprecision(3) << act_rad * RAD2DEG
                      << std::setw(W) << std::setprecision(4) << act_rad
                      << std::setw(W) << std::setprecision(3) << delta << "\n";
        }
        std::cout << "══════════════════════════════════════════════════════\n";
    }

    // TF
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Joint states
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    sensor_msgs::msg::JointState::SharedPtr last_js_;

    // Loop timer
    rclcpp::TimerBase::SharedPtr timer_;
};

// ─── main ──────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegIKNode>());
    rclcpp::shutdown();
    return 0;
}
