#ifndef BITBOTS_DYNAMIC_KICK_KICKUTILS_H
#define BITBOTS_DYNAMIC_KICK_KICKUTILS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

class KickPositions {
public:
    bool is_left_kick = true;
    geometry_msgs::Point support_point;
    geometry_msgs::PoseStamped flying_foot_pose;
    bool cop_support_point = false;
};

class KickGoals {
public:
    std_msgs::Header header;
    geometry_msgs::Vector3 ball_position;
    geometry_msgs::Quaternion kick_direction;
    float kick_speed;
    geometry_msgs::Pose r_foot_pose;
    geometry_msgs::Pose l_foot_pose;
};


#endif //BITBOTS_DYNAMIC_KICK_KICKUTILS_H
