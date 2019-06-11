#ifndef BITBOTS_DYNAMIC_KICK_REFERENCEPOSEGOAL_H
#define BITBOTS_DYNAMIC_KICK_REFERENCEPOSEGOAL_H

#include <bio_ik/goal.h>

class ReferencePoseGoal : public bio_ik::Goal
{
    std::string link_name_;
    std::string reference_link_name_;
    bio_ik::Frame frame_;  // Goal pose relative to reference link

public:
    ReferencePoseGoal()
    {
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

        bio_ik::Frame reference_to_goal = bio_ik::inverse(context.getLinkFrame(1)) * context.getLinkFrame(0);

        e += reference_to_goal.getPosition().distance2(frame_.getPosition());
        e += (reference_to_goal.getOrientation() - frame_.getOrientation()).length2();

        return e;
    }
};

#endif //BITBOTS_DYNAMIC_KICK_REFERENCEPOSEGOAL_H
