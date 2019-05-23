//
// Created by timon on 5/22/19.
//

#ifndef BITBOTS_DYNAMIC_KICK_REFERENCEPOSEGOAL_H
#define BITBOTS_DYNAMIC_KICK_REFERENCEPOSEGOAL_H

#include <bio_ik/goal.h>
#include <ros/console.h>

class ReferencePoseGoal : public bio_ik::Goal
{
    std::string link_name_;
    std::string reference_link_name_;
    bio_ik::Frame frame_;  // Goal pose relative to reference link

public:
    ReferencePoseGoal()
    {
        weight_ = 1;
        link_name_ = "";
        reference_link_name_ = "";
    }
    ReferencePoseGoal(const std::string &link_name, const std::string &reference_link_name,
                          const tf::Vector3 &position, const tf::Quaternion &orientation,
                          double weight)
    {
        weight_ = weight;
        link_name_ = link_name;
        reference_link_name_ = reference_link_name;
    }
    void setLinkName(const std::string &link_name) { link_name_ = link_name; }
    void setReferenceLinkName(const std::string &reference_link_name) { reference_link_name_ = reference_link_name; }
    void setWeight(double weight) { weight_ = weight; }
    void setPosition(tf::Vector3 position) { frame_.setPosition(position); }
    void setOrientation(tf::Quaternion rotation) { frame_.setOrientation(rotation); }
    void describe(bio_ik::GoalContext& context) const
    {
        Goal::describe(context);
        context.addLink(link_name_);
        context.addLink(reference_link_name_);
    }
    double evaluate(const bio_ik::GoalContext& context) const
    {
        double e = 0.0;

        tf::Transform base_to_goal(context.getLinkFrame(0).getOrientation(), context.getLinkFrame(0).getPosition());
        tf::Transform base_to_reference(context.getLinkFrame(1).getOrientation(), context.getLinkFrame(1).getPosition());
        tf::Transform reference_to_base = base_to_reference.inverse();
        tf::Transform reference_to_goal = reference_to_base * base_to_goal;

        e += reference_to_goal.getOrigin().distance2(frame_.getPosition());
        e += (reference_to_goal.getRotation() - frame_.getOrientation()).length2();

        return e;
    }
};

#endif //BITBOTS_DYNAMIC_KICK_REFERENCEPOSEGOAL_H
