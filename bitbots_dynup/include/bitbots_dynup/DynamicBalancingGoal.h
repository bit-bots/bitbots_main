// Walking, 2018, Philipp Ruppel

#ifndef BITBOTS_DYNUP_BALANCING_GOAL_H
#define BITBOTS_DYNUP_BALANCING_GOAL_H
#include <vector>
#include <string>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <moveit/robot_model/robot_model.h>
#include <bio_ik/goal.h>


class DynamicBalancingContext {
    std::vector<tf::Vector3> link_centers_;
    std::vector<double> link_masses_;
    std::vector<std::string> link_names_;
    double total_mass_ = 0.0;
    std::vector<std::vector<tf::Vector3>> history_;
    std::vector<std::vector<tf::Quaternion>> history_r_;
    double time_step_ = 1.0;
    std::vector<tf::Matrix3x3> inertials_;

public:
    explicit DynamicBalancingContext(const moveit::core::RobotModelConstPtr &robot_model) {
        for (auto &link_name : robot_model->getLinkModelNames()) {
            auto link_urdf = robot_model->getURDF()->getLink(link_name);
            if (!link_urdf)
                continue;
            if (!link_urdf->inertial)
                continue;
            const auto &center_urdf = link_urdf->inertial->origin.position;
            tf::Vector3 center(center_urdf.x, center_urdf.y, center_urdf.z);
            double mass = link_urdf->inertial->mass;
            if (!(mass > 0))
                continue;
            link_centers_.push_back(center);
            link_masses_.push_back(mass);
            link_names_.push_back(link_name);
            if (link_urdf->inertial) {
                inertials_.emplace_back(
                        link_urdf->inertial->ixx, link_urdf->inertial->ixy,
                        link_urdf->inertial->ixz, link_urdf->inertial->ixy,
                        link_urdf->inertial->iyy, link_urdf->inertial->iyz,
                        link_urdf->inertial->ixz, link_urdf->inertial->iyz,
                        link_urdf->inertial->izz);
            } else {
                inertials_.emplace_back(0, 0, 0, 0, 0, 0, 0, 0, 0);
            }
            total_mass_ += mass;
        }
    }
    const tf::Matrix3x3 &getInertial(size_t i) const { return inertials_[i]; }
    void setTimeStep(double dt) { time_step_ = dt; }
    double getTimeStep() const { return time_step_; }
    inline const tf::Vector3 &getLinkCenter(size_t index) const {
        return link_centers_[index];
    }
    inline double getLinkMass(size_t index) const { return link_masses_[index]; }
    inline const std::string &getLinkName(size_t index) const {
        return link_names_[index];
    }
    inline size_t getLinkCount() const { return link_names_.size(); }
    inline double getTotalMass() const { return total_mass_; }
    inline size_t getHistorySize() const { return history_.size(); }
    inline const tf::Vector3 &getLinkPosition(size_t link_index,
                                              size_t history_index = 0) const {
        return (
                *(history_.data() + history_.size() - 1 - history_index))[link_index];
    }
    inline const tf::Quaternion &getLinkOrientation(size_t link_index,
                                                    size_t history_index) const {
        return (*(history_r_.data() + history_r_.size() - 1 -
                  history_index))[link_index];
    }
    inline tf::Vector3 getCenterOfGravity(size_t history_index = 0) const {
        tf::Vector3 center_of_gravity(0, 0, 0);
        for (size_t i = 0; i < getLinkCount(); i++) {
            auto center = getLinkPosition(i);
            double mass = getLinkMass(i);
            center_of_gravity += center * mass;
        }
        center_of_gravity /= getTotalMass();
        return center_of_gravity;
    }
};

class DynamicBalancingGoal : public bio_ik::Goal {
    tf::Vector3 target_;
    const DynamicBalancingContext *balancing_context_;
    tf::Vector3 gravity_ = tf::Vector3(0, 0, -9.81);
    std::string reference_link_;

public:
    DynamicBalancingGoal(const DynamicBalancingContext *balancing_context,
                         const tf::Vector3 &target, double weight)
            : target_(target), balancing_context_(balancing_context) {
        weight_ = weight;
        reference_link_ = "base_link";
    }
    void setReferenceLink(std::string link) { reference_link_ = link; }
    virtual void describe(bio_ik::GoalContext &context) const {
        Goal::describe(context);
        for (size_t i = 0; i < balancing_context_->getLinkCount(); i++) {
            context.addLink(balancing_context_->getLinkName(i));
        }
        context.addLink(reference_link_);
    }
    static inline double sign(double v) {
        if (v < 0.0)
            return -1.0;
        if (v > 0.0)
            return +1.0;
        return 0.0;
    }
    virtual double evaluate(const bio_ik::GoalContext &context) const {

        tf::Vector3 torque_g = tf::Vector3(0, 0, 0);
        tf::Vector3 torque_r = tf::Vector3(0, 0, 0);
        tf::Vector3 torque_v = tf::Vector3(0, 0, 0);

        double dt_rcp = balancing_context_->getTimeStep();

        // Last element (after all of the regular links) is the reference link
        bio_ik::Frame reference_link = bio_ik::inverse(context.getLinkFrame(balancing_context_->getLinkCount()));

        // static torques from gravity
        for (size_t i = 0; i < balancing_context_->getLinkCount(); i++) {
            tf::Vector3 center = balancing_context_->getLinkCenter(i); // m
            double mass = balancing_context_->getLinkMass(i);   // kg
            const bio_ik::Frame& frame = reference_link * context.getLinkFrame(i);
            bio_ik::quat_mul_vec(frame.rot, center, center);
            center += frame.pos;
            torque_g += (center - target_).cross(gravity_ * mass); // m * N
        }

        // torque_sum *= 20;

        double friction = 0.1;

        // simplified inverted pendulum model
        if (balancing_context_->getHistorySize() >= 2) {
            tf::Vector3 p0(0, 0, 0);
            tf::Vector3 p1(0, 0, 0);
            tf::Vector3 p2(0, 0, 0);
            for (size_t i = 0; i < balancing_context_->getLinkCount(); i++) {

                auto center = balancing_context_->getLinkCenter(i); // m
                auto frame = reference_link * context.getLinkFrame(i);
                bio_ik::quat_mul_vec(frame.rot, center, center);
                center += frame.pos;

                p0 += balancing_context_->getLinkPosition(i, 1) *
                      balancing_context_->getLinkMass(i);
                p1 += balancing_context_->getLinkPosition(i, 0) *
                      balancing_context_->getLinkMass(i);
                // p2 += context.getLinkFrame(i).pos *
                // balancing_context_->getLinkMass(i);
                p2 += center * balancing_context_->getLinkMass(i);
            }
            double s = 1.0 / balancing_context_->getTotalMass();
            p0 *= s;
            p1 *= s;
            p2 *= s;

            tf::Vector3 v0 = (p1 - p0) * dt_rcp; // m / s
            tf::Vector3 v1 = (p2 - p1) * dt_rcp; // m / s

            v0 *= (1.0 - friction);

            tf::Vector3 a = (v1 - v0) * dt_rcp; // m / s²

            tf::Vector3 f =
                    balancing_context_->getTotalMass() * a; // kg * m / s² = N

            torque_v -= (p1 - target_).cross(f) * 0.5; // m * N
        }

        double m = balancing_context_->getTotalMass();
        return (torque_g.length2() + (torque_v + torque_r).length2()) / (m * m * gravity_.length2());
    }
};

#endif  // BITBOTS_DYNUP_BALANCING_GOAL_H
