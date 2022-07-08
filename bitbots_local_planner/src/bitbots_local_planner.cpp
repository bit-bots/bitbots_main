#include <bitbots_local_planner/bitbots_local_planner.h>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(bitbots_local_planner::BBPlanner, nav2_core::Controller)

namespace bitbots_local_planner
{

    BBPlanner::BBPlanner()
    {
    }

    void BBPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        parent_ = parent;
        auto node = parent_.lock();

        logger_ = std::make_shared<rclcpp::Logger>(node->get_logger());
        clock_ = node->get_clock();
        costmap_ros_ = costmap_ros;

        // Register publishers
        local_plan_publisher_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

        // Link tf buffer
        tf_buffer_ = tf_buffer;
        // Link costmap
        costmap_ros_ = costmap_ros;

        auto plugin_name = name;

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".carrot_distance",
            rclcpp::ParameterValue(10));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".max_rotation_vel",
            rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".max_vel_x",
            rclcpp::ParameterValue(0.05));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".max_vel_y",
            rclcpp::ParameterValue(0.05));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".min_vel_x",
            rclcpp::ParameterValue(0.02));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".orient_to_goal_distance",
            rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".rotation_slow_down_factor",
            rclcpp::ParameterValue(0.3));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".smoothing_k",
            rclcpp::ParameterValue(0.04));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name + ".translation_slow_down_factor",
            rclcpp::ParameterValue(0.5));

        node->get_parameter(plugin_name + ".carrot_distance", config_carrot_distance);
        node->get_parameter(plugin_name + ".max_rotation_vel", config_max_rotation_vel);
        node->get_parameter(plugin_name + ".max_vel_x", config_max_vel_x);
        node->get_parameter(plugin_name + ".max_vel_y", config_max_vel_y);
        node->get_parameter(plugin_name + ".min_vel_x", config_min_vel_x);
        node->get_parameter(plugin_name + ".orient_to_goal_distance", config_orient_to_goal_distance);
        node->get_parameter(plugin_name + ".rotation_slow_down_factor", config_rotation_slow_down_factor);
        node->get_parameter(plugin_name + ".smoothing_k", config_smoothing_k);
        node->get_parameter(plugin_name + ".translation_slow_down_factor", config_translation_slow_down_factor);

        RCLCPP_DEBUG(*logger_.get(), "BBPlanner: Version 2 Init.");
    }

    /**
     * @brief Method to cleanup resources.
     */
    void BBPlanner::cleanup()
    {

    }

    /**
     * @brief Method to active planner and any threads involved in execution.
     */
    void BBPlanner::activate()
    {

    }

    /**
     * @brief Method to deactive planner and any threads involved in execution.
     */
    void BBPlanner::deactivate()
    {

    }

    geometry_msgs::msg::TwistStamped BBPlanner::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        // Save time for profiling
        rclcpp::Time begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        // Create velocity msg
        auto cmd_vel = geometry_msgs::msg::Twist();

        // Query the current robot pose as a transform
        geometry_msgs::msg::Pose current_pose;
        geometry_msgs::msg::PoseStamped current_pose_stamped;
        costmap_ros_->getRobotPose(current_pose_stamped);
        current_pose = current_pose_stamped.pose;


        // Calculate the heading angle from our current position to the carrot
        double walk_angle = std::atan2(
            goal_pose_.position.y - current_pose.position.y,
            goal_pose_.position.x - current_pose.position.x);

        // Calculate the heading angle from our current position to the final position of the global plan
        double final_walk_angle = std::atan2(
            end_pose_.position.y - current_pose.position.y,
            end_pose_.position.x - current_pose.position.x);

        // Calculate the distance from our current position to the final position of the global plan
        double distance = std::hypot(
            end_pose_.position.x - current_pose.position.x,
            end_pose_.position.y - current_pose.position.y);

        // Calculate the translational walk velocity. It considers the distance and breaks if we are close to the final position of the global plan
        double walk_vel = std::min(distance * config_translation_slow_down_factor, config_max_vel_x);

        // Check if we are so close to the final position of the global plan that we want to align us with its orientation and not the heading towards its position
        double diff = 0;
        if (distance > config_orient_to_goal_distance)
        {
            // Calculate the difference between our current heading and the heading towards the final position of the global plan
            diff = final_walk_angle - tf2::getYaw(current_pose.orientation);
        }
        else
        {
            // Calculate the difference between our current heading and the heading of the final position of the global plan
            diff = tf2::getYaw(end_pose_.orientation) - tf2::getYaw(current_pose.orientation);
        }

        // Get the min angle of the difference
        double min_angle = std::remainder(diff, 2 * M_PI);
        // Calculate our desired rotation velocity based on the angle difference and our max velocity
        double rot_goal_vel = std::clamp(
            config_rotation_slow_down_factor * min_angle,
            -config_max_rotation_vel,
            config_max_rotation_vel);

        // Get odometry velocity vector
        geometry_msgs::PoseStamped robot_vel;
        //odom_helper_.getRobotVel(robot_vel); TODO implement
        // Calc current velocity value
        double current_vel_ = std::hypot(robot_vel.pose.position.x, robot_vel.pose.position.y);

        // Calculate the x and y components of our linear velocity based on the desired heading and the desired translational velocity.
        cmd_vel.linear.x = std::cos(walk_angle - tf2::getYaw(current_pose.orientation)) * walk_vel;
        cmd_vel.linear.y = std::sin(walk_angle - tf2::getYaw(current_pose.orientation)) * walk_vel;

        // Scale command accordingly if a limit is acceded
        if (cmd_vel.linear.x > config_max_vel_x) {
            RCLCPP_DEBUG(*logger_.get(), "X LIMIT reached: %f > %f, with y %f", cmd_vel.linear.x, config_max_vel_x, cmd_vel.linear.y);
            cmd_vel.linear.y *= config_max_vel_x / cmd_vel.linear.x;
            cmd_vel.linear.x = config_max_vel_x;
            RCLCPP_DEBUG(*logger_.get(), "X LIMIT set y %f", cmd_vel.linear.y);
        }

        if (cmd_vel.linear.x < config_min_vel_x) {
            RCLCPP_DEBUG(*logger_.get(), "X LIMIT reached: %f < %f, with y %f", cmd_vel.linear.x, config_min_vel_x, cmd_vel.linear.y);
            cmd_vel.linear.y *= config_min_vel_x / cmd_vel.linear.x;
            cmd_vel.linear.x = config_min_vel_x;
            RCLCPP_DEBUG(*logger_.get(), "X LIMIT set y %f", cmd_vel.linear.y);
        }

        double max_y = config_max_vel_y;

        if (std::abs(cmd_vel.linear.y) > max_y) {
            RCLCPP_DEBUG(*logger_.get(), "Y LIMIT reached: %f > %f, with x %f", cmd_vel.linear.y, max_y, cmd_vel.linear.x);
            cmd_vel.linear.x *= max_y / std::abs(cmd_vel.linear.y);
            cmd_vel.linear.y *= max_y / std::abs(cmd_vel.linear.y);
            RCLCPP_DEBUG(*logger_.get(), "Y LIMIT set x %f", cmd_vel.linear.x);
        }

        // Complementary Filter for smooting the outputs
        cmd_vel.linear.x = cmd_vel.linear.x * config_smoothing_k + robot_vel.pose.position.x * (1.0 - config_smoothing_k);
        cmd_vel.linear.y = cmd_vel.linear.y * config_smoothing_k + robot_vel.pose.position.y * (1.0 - config_smoothing_k);

        // Apply the desired rotational velocity
        cmd_vel.angular.z = rot_goal_vel;

        RCLCPP_DEBUG(*logger_.get(), "End vel %f, %f\n", std::hypot(cmd_vel.linear.x, cmd_vel.linear.y), current_vel_);

        // Publich our "local plan" for viz purposes
        publishPlan();

        // Print profiling result
        rclcpp::Time end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        rclcpp::Duration duration = end - begin;
        RCLCPP_DEBUG(*logger_.get(), "BBPlanner: Calculation time: %f seconds", duration.seconds());

        // Create stamped msg
        auto cmd_vel_stamped = geometry_msgs::msg::TwistStamped();
        cmd_vel_stamped.header.stamp = clock_->now();
        cmd_vel_stamped.header.frame_id = "base_footprint";
        cmd_vel_stamped.twist = cmd_vel;
        return cmd_vel_stamped;
    }

    void BBPlanner::setPlan(const nav_msgs::msg::Path & path)
    {
        // Save global plan
	    global_plan_ = path.poses;

        for (geometry_msgs::msg::PoseStamped & pose : global_plan_)
        {
            pose.header = path.header;
        }

        // set carrot distance to the config value or the end of the path if it is shorter
	    int carrot_distance = std::min((int)global_plan_.size() - 1, config_carrot_distance);

        // Querys the pose of our carrot which we want to follow
        goal_pose_ = this->getXPose(
            costmap_ros_->getGlobalFrameID(),
            carrot_distance);

        // Query the final pose of our robot at the end of the global plan
        end_pose_ = this->getXPose(
            costmap_ros_->getGlobalFrameID(),
            global_plan_.size() - 1);
    }


    geometry_msgs::msg::Pose BBPlanner::getXPose(
            const std::string &global_frame,
            int plan_point)
    {
        if (global_plan_.empty()) {
            RCLCPP_ERROR(*logger_.get(), "Received plan with zero length");
        }
        if (plan_point >= (int) global_plan_.size()) {
            RCLCPP_ERROR(*logger_.get(),"Goal_functions: Plan_point %d to big. Plan size: %lu", plan_point, global_plan_.size());
        }

        geometry_msgs::msg::PoseStamped &plan_goal_pose = global_plan_.at(plan_point);
        geometry_msgs::msg::PoseStamped result;
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                global_frame, plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp);
            tf2::doTransform(plan_goal_pose, result, transform);
        }
        catch (tf2::LookupException &ex) {
            RCLCPP_ERROR(*logger_.get(),"No Transform available Error: %s\n", ex.what());
        }
        catch (tf2::ConnectivityException &ex) {
            RCLCPP_ERROR(*logger_.get(),"Connectivity Error: %s\n", ex.what());
        }
        catch (tf2::ExtrapolationException &ex) {
            RCLCPP_ERROR(*logger_.get(),"Extrapolation Error: %s\n", ex.what());
            if (!global_plan_.empty()) {
            RCLCPP_ERROR(*logger_.get(),"Global Frame: %s Plan Frame size %d: %s\n",
                        global_frame.c_str(),
                        (unsigned int) global_plan_.size(),
                        global_plan_[0].header.frame_id.c_str());
            }
        }
        return result.pose;
    }

    void BBPlanner::motionOdomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        motion_odom_ = msg;
    }

    void BBPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
    {

    };


    void BBPlanner::publishPlan()
    {
        // Create a path message
        nav_msgs::msg::Path gui_path;
        //Get the robot pose
        geometry_msgs::msg::PoseStamped robot_pose;
        costmap_ros_->getRobotPose(robot_pose);
        // Set the path length to 2. Wow so long
        gui_path.poses.resize(2);
        // Set header stuff
        gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        gui_path.header.stamp = clock_->now();
        // The the positions of the two path elements (1. The robot position, 2. The carrot position)
        gui_path.poses[0].pose.position = robot_pose.pose.position;
        gui_path.poses[1].pose.position = goal_pose_.position;
        // Publish
        local_plan_publisher_->publish(gui_path);
    }

    BBPlanner::~BBPlanner()
    {
    }
}
