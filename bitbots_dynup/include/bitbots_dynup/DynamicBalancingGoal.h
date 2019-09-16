// Walking, 2018, Philipp Ruppel

#ifndef BITBOTS_DYNUP_DYNAMIC_BALANCING_GOAL_H
#define BITBOTS_DYNUP_DYNAMIC_BALANCING_GOAL_H
#include <vector>
#include <string>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit/robot_model/robot_model.h>
#include <bio_ik/goal.h>


class DynamicBalancingContext {
    std::vector<tf2::Vector3> link_centers_;
    std::vector<double> link_masses_;
    std::vector<std::string> link_names_;
    double total_mass_ = 0.0;

public:
    explicit DynamicBalancingContext(const moveit::core::RobotModelConstPtr &robot_model) {
        for (auto &link_name : robot_model->getLinkModelNames()) {
            auto link_urdf = robot_model->getURDF()->getLink(link_name);
            if (!link_urdf)
                continue;
            if (!link_urdf->inertial)
                continue;
            const auto &center_urdf = link_urdf->inertial->origin.position;
            tf2::Vector3 center(center_urdf.x, center_urdf.y, center_urdf.z);
            double mass = link_urdf->inertial->mass;
            if (!(mass > 0))
                continue;
            link_centers_.push_back(center);
            link_masses_.push_back(mass);
            link_names_.push_back(link_name);
            total_mass_ += mass;
        }
    }
    inline const tf2::Vector3 &getLinkCenter(size_t index) const {
        return link_centers_[index];
    }
    inline double getLinkMass(size_t index) const { return link_masses_[index]; }
    inline const std::string &getLinkName(size_t index) const {
        return link_names_[index];
    }
    inline size_t getLinkCount() const { return link_names_.size(); }
    inline double getTotalMass() const { return total_mass_; }
};

class DynamicBalancingGoal : public bio_ik::Goal {
    tf2::Vector3 target_;
    const DynamicBalancingContext *balancing_context_;
    tf2::Vector3 gravity_ = tf2::Vector3(0, 0, -9.81);
    std::string reference_link_;

public:
    DynamicBalancingGoal(const DynamicBalancingContext *balancing_context,
                         const tf2::Vector3 &target, double weight)
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
    virtual double evaluate(const bio_ik::GoalContext &context) const {
        tf2::Vector3 torque_g = tf2::Vector3(0, 0, 0);

        // Last element (after all of the regular links) is the reference link
        bio_ik::Frame reference_link = bio_ik::inverse(context.getLinkFrame(balancing_context_->getLinkCount()));

        // static torques from gravity
        for (size_t i = 0; i < balancing_context_->getLinkCount(); i++) {
            tf2::Vector3 center = balancing_context_->getLinkCenter(i); // m
            double mass = balancing_context_->getLinkMass(i);   // kg
            const bio_ik::Frame& frame = reference_link * context.getLinkFrame(i);
            bio_ik::quat_mul_vec(frame.rot, center, center);
            center += frame.pos;
            torque_g += (center - target_).cross(gravity_ * mass); // m * N
        }

        double m = balancing_context_->getTotalMass();
        return torque_g.length2() / (m * m * gravity_.length2());
    }
};

#endif  // BITBOTS_DYNUP_DYNAMIC_BALANCING_GOAL_H
