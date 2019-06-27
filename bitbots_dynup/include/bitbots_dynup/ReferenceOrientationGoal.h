#ifndef BITBOTS_DYNUP_REFERENCEORIENTATIONGOAL_H
#define BITBOTS_DYNUP_REFERENCEORIENTATIONGOAL_H

#include <bio_ik/goal.h>

class ReferenceOrientationGoal : public bio_ik::Goal
{
    std::string link_name_;
    std::string reference_link_name_;
    tf::Quaternion orientation_;

public:
    ReferenceOrientationGoal()
        : orientation_(0, 0, 0, 1)
        , link_name_("")
        , reference_link_name_("")
    {
        weight_ = 1;
    }
    ReferenceOrientationGoal(const std::string& link_name, const std::string& reference_link_name,
                    const tf::Quaternion& orientation, double weight = 1.0)
        : link_name_(link_name)
        , reference_link_name_(reference_link_name)
        , orientation_(orientation.normalized())
    {
        weight_ = weight;
    }
    inline const tf::Quaternion& getOrientation() const { return orientation_; }
    inline void setOrientation(const tf::Quaternion& orientation) { orientation_ = orientation.normalized(); }
    void setReferenceLinkName(const std::string &reference_link_name) { reference_link_name_ = reference_link_name; }
    void setLinkName(const std::string &link_name) { link_name_ = link_name; }
    void describe(bio_ik::GoalContext& context) const
    {
        Goal::describe(context);
        context.addLink(link_name_);
        context.addLink(reference_link_name_);
    }
    virtual double evaluate(const bio_ik::GoalContext& context) const
    {
        bio_ik::Frame reference_to_goal = bio_ik::inverse(context.getLinkFrame(1)) * context.getLinkFrame(0);
        return fmin((getOrientation() - reference_to_goal.getOrientation()).length2(), (getOrientation() + reference_to_goal.getOrientation()).length2());
    }
};

#endif //BITBOTS_DYNAMIC_KICK_REFERENCEORIENTATIONGOAL_H
