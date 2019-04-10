#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine() {}

void KickEngine::reset() {}

void KickEngine::set_goal(const bitbots_msgs::KickGoalConstPtr &goal) {}

bitbots_msgs::KickFeedback KickEngine::tick() {
    return bitbots_msgs::KickFeedback();
}

void KickEngine::calc_splines() {}