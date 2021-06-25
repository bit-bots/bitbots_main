#include <bitbots_local_planner/bitbots_local_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(bitbots_local_planner::BBPlanner, nav_core::BaseLocalPlanner)

namespace bitbots_local_planner
{

    BBPlanner::BBPlanner() : odom_helper_("odom")
    {
    }

    void BBPlanner::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap_ros)
    {
        // Init node handles
        ros::NodeHandle private_nh("~/" + name);

        // Register publishers
        local_plan_publisher_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

        // Link tf buffer
        tf_buffer_ = tf_buffer;
        // Link costmap
        costmap_ros_ = costmap_ros;

        // Set fields to default values
        goal_reached_ = false;

        //odom_helper_ = new base_local_planner::OdometryHelperRos("motion_odometry");

        // Setup dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<BBPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<BBPlannerConfig>::CallbackType cb = boost::bind(&BBPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_DEBUG("BBPlanner: Version 2 Init.");
    }

    void BBPlanner::motionOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
    {
        motion_odom_ = *msg;
        ROS_ERROR("GOT ODOM");
    }

    void BBPlanner::reconfigureCB(BBPlannerConfig &config, uint32_t level)
    {
        // Save new config values
        config_ = config;
    }

	bool BBPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
        // Save global plan
	    global_plan_ = plan;

        // set carrot distance to the config value or the end of the path if it is shorter
	    int carrot_distance = std::min((int)global_plan_.size() - 1, config_.carrot_distance);

        // Querys the pose of our carrot which we want to follow
        bitbots_local_planner::getXPose(
            *tf_buffer_,
            global_plan_,
            costmap_ros_->getGlobalFrameID(),
            goal_pose_,
            carrot_distance);

        // Query the final pose of our robot at the end of the global plan
        bitbots_local_planner::getXPose(
            *tf_buffer_, global_plan_,
            costmap_ros_->getGlobalFrameID(),
            end_pose_,
            global_plan_.size() - 1);

        // Set our old pose
        old_goal_pose_ = goal_pose_;

        // We obviously didn't reach that plan (yet)
        goal_reached_ = false;

        return true;
    }

    bool BBPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        // Save time for profiling
        ros::Time begin = ros::Time::now();

        // Query the current robot pose as a transform
        tf2::Stamped<tf2::Transform> current_pose;
        geometry_msgs::PoseStamped current_pose_gm;
        costmap_ros_->getRobotPose(current_pose_gm);
        tf2::fromMsg(current_pose_gm, current_pose);

        // Calculate the heading angle from our current position to the carrot
        double walk_angle = std::atan2(
            goal_pose_.getOrigin().y() - current_pose.getOrigin().y(),
            goal_pose_.getOrigin().x() - current_pose.getOrigin().x());

        // Calculate the heading angle from our current position to the final position of the global plan
        double final_walk_angle = std::atan2(
            end_pose_.getOrigin().y() - current_pose.getOrigin().y(),
            end_pose_.getOrigin().x() - current_pose.getOrigin().x());

        // Calculate the distance from our current position to the final position of the global plan
        double distance = std::hypot(
            end_pose_.getOrigin().x() - current_pose.getOrigin().x(),
            end_pose_.getOrigin().y() - current_pose.getOrigin().y());

        // Calculate the translational walk velocity. It considers the distance and breaks if we are close to the final position of the global plan
        double walk_vel = std::min(distance * config_.translation_slow_down_factor, config_.max_vel_x);

        // Check if we are so close to the final position of the global plan that we want to align us with its orientation and not the heading towards its position
        double diff = 0;
        if (distance > config_.orient_to_goal_distance)
        {
            // Calculate the difference between our current heading and the heading towards the final position of the global plan
            diff = final_walk_angle - tf2::getYaw(current_pose.getRotation());
        }
        else
        {
            // Calculate the difference between our current heading and the heading of the final position of the global plan
            diff = tf2::getYaw(end_pose_.getRotation()) - tf2::getYaw(current_pose.getRotation());
        }

        // Get the min angle of the difference
        double min_angle = std::remainder(diff, 2 * M_PI);
        // Calculate our desired rotation velocity based on the angle difference and our max velocity
        double rot_goal_vel = std::clamp(
            config_.rotation_slow_down_factor * min_angle,
            -config_.max_rotation_vel,
            config_.max_rotation_vel);

        // Check if we reached the goal. If we did so set the velocities to 0
        if (distance < config_.position_accuracy && abs(min_angle) < config_.rotation_accuracy)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            goal_reached_ = true;
        }
        else
        {
            // Get odometry velocity vector
            geometry_msgs::PoseStamped robot_vel;
            odom_helper_.getRobotVel(robot_vel);
            // Calc current velocity value
            double current_vel_ = std::hypot(robot_vel.pose.position.x, robot_vel.pose.position.y);

            // Calculate the x and y components of our linear velocity based on the desired heading and the desired translational velocity.
            cmd_vel.linear.x = std::cos(walk_angle - tf2::getYaw(current_pose.getRotation())) * walk_vel;
            cmd_vel.linear.y = std::sin(walk_angle - tf2::getYaw(current_pose.getRotation())) * walk_vel;

            // Scale command accordingly if a limit is acceded
            if (cmd_vel.linear.x > config_.max_vel_x) {
                ROS_DEBUG("X LIMIT reached: %f > %f, with y %f", cmd_vel.linear.x, config_.max_vel_x, cmd_vel.linear.y);
                cmd_vel.linear.y *= config_.max_vel_x / cmd_vel.linear.x;
                cmd_vel.linear.x = config_.max_vel_x;
                ROS_DEBUG("X LIMIT set y %f", cmd_vel.linear.y);
            }

            if (cmd_vel.linear.x < config_.min_vel_x) {
                ROS_DEBUG("X LIMIT reached: %f < %f, with y %f", cmd_vel.linear.x, config_.min_vel_x, cmd_vel.linear.y);
                cmd_vel.linear.y *= config_.min_vel_x / cmd_vel.linear.x;
                cmd_vel.linear.x = config_.min_vel_x;
                ROS_DEBUG("X LIMIT set y %f", cmd_vel.linear.y);
            }

            double max_y = config_.max_vel_y;

            if (std::abs(cmd_vel.linear.y) > max_y) {
                ROS_DEBUG("Y LIMIT reached: %f > %f, with x %f", cmd_vel.linear.y, max_y, cmd_vel.linear.x);
                cmd_vel.linear.x *= max_y / std::abs(cmd_vel.linear.y);
                cmd_vel.linear.y *= max_y / std::abs(cmd_vel.linear.y);
                ROS_DEBUG("Y LIMIT set x %f", cmd_vel.linear.x);
            }

            // Complementary Filter for smooting the outputs
            cmd_vel.linear.x = cmd_vel.linear.x * config_.smoothing_k + robot_vel.pose.position.x * (1.0 - config_.smoothing_k);
            cmd_vel.linear.y = cmd_vel.linear.y * config_.smoothing_k + robot_vel.pose.position.y * (1.0 - config_.smoothing_k);

            // Apply the desired rotational velocity
            cmd_vel.angular.z = rot_goal_vel;
            // We didn't reached our goal if we need this step
            goal_reached_ = false;

            ROS_DEBUG("End vel %f, %f\n", std::hypot(cmd_vel.linear.x, cmd_vel.linear.y), current_vel_);
        }
        // Publich our "local plan" for viz purposes
        publishPlan();

        // Print profiling result
        ros::Time end = ros::Time::now();
        ros::Duration duration = end - begin;
        ROS_DEBUG("BBPlanner: Calculation time: %f seconds", duration.toSec());
        return true;
    }

    bool BBPlanner::isGoalReached()
    {
        if (goal_reached_)
        {
            ROS_DEBUG("BBPlanner: Goal reached.");
        }
        return goal_reached_;
    }

    void BBPlanner::publishPlan()
    {
        // Create a path message
        nav_msgs::Path gui_path;
        //Get the robot pose
        geometry_msgs::PoseStamped robot_pose, carrot_pose;
        costmap_ros_->getRobotPose(robot_pose);
        // Set the path length to 2. Wow so long
        gui_path.poses.resize(2);
        // Set header stuff
        gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        gui_path.header.stamp = ros::Time::now();
        // Get the carrot pose
        tf2::toMsg(goal_pose_, carrot_pose);
        // The the positions of the two path elements (1. The robot position, 2. The carrot position)
        gui_path.poses[0].pose.position = robot_pose.pose.position;
        gui_path.poses[1].pose.position = carrot_pose.pose.position;
        // Publish
        local_plan_publisher_.publish(gui_path);
    }

    BBPlanner::~BBPlanner()
    {
    }
}
