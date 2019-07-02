#include "bitbots_dynup/DynupEngine.h"

DynupEngine::DynupEngine() : m_listener(m_tf_buffer) {}

void DynupEngine::reset() {
    m_time = 0;
    m_hand_trajectories.reset();
    m_foot_trajectories.reset();
}

std::optional<JointGoals> DynupEngine::tick(double dt) {
    /* Only do an actual tick when splines are present */
    if (m_hand_trajectories && m_foot_trajectories) {
        /* Get should-be pose from planned splines (every axis) at current time */
        geometry_msgs::PoseStamped l_foot_pose = get_current_pose(m_foot_trajectories.value(), true);
        geometry_msgs::PoseStamped trunk_pose = get_current_pose(m_trunk_trajectories.value(), false);
        //geometry_msgs::PoseStamped l_hand_pose = get_current_pose(m_foot_trajectories.value(), "l_hand");
        //geometry_msgs::PoseStamped r_hand_pose = get_current_pose(m_foot_trajectories.value(), "r_hand");


        m_time += dt;
        //TODO support point between feet
        geometry_msgs::Point support_point;
        /* Stabilize and return result */
        return m_stabilizer.stabilize(support_point, l_foot_pose, trunk_pose);
    } else {
        return std::nullopt;
    }
}

geometry_msgs::PoseStamped DynupEngine::get_current_pose(Trajectories spline_container, bool foot) {
    geometry_msgs::PoseStamped pose;
    if(foot){
        pose.header.frame_id = "l_sole";
    }else{
        pose.header.frame_id = "torso";
    }
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = spline_container.get("pos_x").pos(m_time);
    pose.pose.position.y = spline_container.get("pos_y").pos(m_time);
    pose.pose.position.z = spline_container.get("pos_z").pos(m_time);
    tf2::Quaternion q;
    /* Apparently, the axis order is different than expected */
    q.setEuler(spline_container.get("pitch").pos(m_time),
               spline_container.get("roll").pos(m_time),
               spline_container.get("yaw").pos(m_time));
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

void DynupEngine::calc_front_splines(){

    //
    // TODO THIS IS CURRENTLY NOT USEEEEEEED
    //

    /*
    calculates splines for front up
    */

    /*
     * start spline point with current poses
     */

    double time_start = 0;

    // hand
    geometry_msgs::Pose hand_pose; //TODO read actual pose

    m_hand_trajectories->get("pos_x").addPoint(time_start, hand_pose.position.x);
    m_hand_trajectories->get("pos_y").addPoint(time_start, hand_pose.position.y);
    m_hand_trajectories->get("pos_z").addPoint(time_start, hand_pose.position.z);

    /* Construct a start_rotation as quaternion from Pose msg */
    tf::Quaternion hand_start_rotation(hand_pose.orientation.x, hand_pose.orientation.y,
                                  hand_pose.orientation.z, hand_pose.orientation.w);
    double hand_start_r, hand_start_p, hand_start_y;
    tf::Matrix3x3(hand_start_rotation).getRPY(hand_start_r, hand_start_p, hand_start_y);
    m_hand_trajectories->get("roll").addPoint(time_start, hand_start_r);
    m_hand_trajectories->get("pitch").addPoint(time_start, hand_start_p);
    m_hand_trajectories->get("yaw").addPoint(time_start, hand_start_y);

    // foot
    geometry_msgs::Pose foot_pose; //TODO read actual pose
    m_foot_trajectories->get("pos_x").addPoint(time_start, foot_pose.position.x);
    m_foot_trajectories->get("pos_y").addPoint(time_start, foot_pose.position.y);
    m_foot_trajectories->get("pos_z").addPoint(time_start, foot_pose.position.z);

    /* Construct a start_rotation as quaternion from Pose msg */
    tf::Quaternion foot_start_rotation(foot_pose.orientation.x, foot_pose.orientation.y,
                                  foot_pose.orientation.z, foot_pose.orientation.w);
    double foot_start_r, foot_start_p, foot_start_y;
    tf::Matrix3x3(foot_start_rotation).getRPY(foot_start_r, foot_start_p, foot_start_y);
    m_foot_trajectories->get("roll").addPoint(time_start, foot_start_r);
    m_foot_trajectories->get("pitch").addPoint(time_start, foot_start_p);
    m_foot_trajectories->get("yaw").addPoint(time_start, foot_start_y);


    //TODO spline in between to enable the hands to go to the front

    /*
     * pull legs to body
    */
    double time_foot_close = m_params.time_foot_close; // TODO
    m_foot_trajectories->get("pos_x").addPoint(time_foot_close, 0);
    m_foot_trajectories->get("pos_y").addPoint(time_foot_close, 0);
    m_foot_trajectories->get("pos_z").addPoint(time_foot_close, m_params.leg_min_length);
    m_foot_trajectories->get("roll").addPoint(time_foot_close, 0);
    m_foot_trajectories->get("pitch").addPoint(time_foot_close, 0);
    m_foot_trajectories->get("yaw").addPoint(time_foot_close, 0);


    /*
     * hands to the front
     */
    double time_hands_front = m_params.time_hands_front; //TODO parameter
    m_hand_trajectories->get("pos_x").addPoint(time_hands_front, 0);
    m_hand_trajectories->get("pos_y").addPoint(time_hands_front, 0);
    m_hand_trajectories->get("pos_z").addPoint(time_hands_front, m_params.arm_max_length);
    m_hand_trajectories->get("roll").addPoint(time_hands_front, 0);
    m_hand_trajectories->get("pitch").addPoint(time_hands_front, 3.14); //todo pi
    m_hand_trajectories->get("yaw").addPoint(time_hands_front, 0);

    /*
     * Foot under body
     */
    double time_foot_ground = m_params.time_foot_ground; //TODO
    m_foot_trajectories->get("pos_x").addPoint(time_foot_ground, 0);
    m_foot_trajectories->get("pos_y").addPoint(time_foot_ground, 0);
    m_foot_trajectories->get("pos_z").addPoint(time_foot_ground, m_params.leg_min_length);
    m_foot_trajectories->get("roll").addPoint(time_foot_ground, 0);
    m_foot_trajectories->get("pitch").addPoint(time_foot_ground, 3.14); //todo pi
    m_foot_trajectories->get("yaw").addPoint(time_foot_ground, 0);


    /*
     * Torso 45°
     */
    double time_torso_45 = m_params.time_torso_45; //TODO
    m_hand_trajectories->get("pos_x").addPoint(time_torso_45, m_params.arm_max_length);
    m_hand_trajectories->get("pos_y").addPoint(time_torso_45, 0);
    m_hand_trajectories->get("pos_z").addPoint(time_torso_45, 0);
    m_hand_trajectories->get("roll").addPoint(time_torso_45, 0);
    m_hand_trajectories->get("pitch").addPoint(time_torso_45, 0);
    m_hand_trajectories->get("yaw").addPoint(time_torso_45, 0);

    m_foot_trajectories->get("pos_x").addPoint(time_torso_45, 0);
    m_foot_trajectories->get("pos_y").addPoint(time_torso_45, 0);
    m_foot_trajectories->get("pos_z").addPoint(time_torso_45, m_params.leg_min_length);
    m_foot_trajectories->get("roll").addPoint(time_torso_45, 0);
    m_foot_trajectories->get("pitch").addPoint(time_torso_45, 3.14); //todo pi
    m_foot_trajectories->get("yaw").addPoint(time_torso_45, 0);

}

void DynupEngine::calc_back_splines(){

    //TODO from back to squat

}


void DynupEngine::calc_squat_splines(geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose){

    // current position as first spline point
    // all positions relative to right foot

    // m_foot_trajectories are for left foot
    m_foot_trajectories->get("pos_x").addPoint(0, l_foot_pose.position.x);
    m_foot_trajectories->get("pos_y").addPoint(0, l_foot_pose.position.y);
    m_foot_trajectories->get("pos_z").addPoint(0, l_foot_pose.position.z);
    double r, p, y;
    tf2::Quaternion q;
    tf2::convert(l_foot_pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    m_foot_trajectories->get("roll").addPoint(0, r);
    m_foot_trajectories->get("pitch").addPoint(0, p);
    m_foot_trajectories->get("yaw").addPoint(0, y);

    m_foot_trajectories->get("pos_x").addPoint(m_params.rise_time, 0);
    m_foot_trajectories->get("pos_y").addPoint(m_params.rise_time, m_params.foot_distance);
    m_foot_trajectories->get("pos_z").addPoint(m_params.rise_time, 0);
    m_foot_trajectories->get("roll").addPoint(m_params.rise_time, 0);
    m_foot_trajectories->get("pitch").addPoint(m_params.rise_time, 0);
    m_foot_trajectories->get("yaw").addPoint(m_params.rise_time, 0);

    // m_trunk_trajectories are for trunk (relative to right foot)
    m_trunk_trajectories->get("pos_x").addPoint(0, trunk_pose.position.x);
    m_trunk_trajectories->get("pos_y").addPoint(0, trunk_pose.position.y);
    m_trunk_trajectories->get("pos_z").addPoint(0, trunk_pose.position.z);
    tf2::convert(trunk_pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    m_trunk_trajectories->get("roll").addPoint(0, r);
    m_trunk_trajectories->get("pitch").addPoint(0, p);
    m_trunk_trajectories->get("yaw").addPoint(0, y);

    m_trunk_trajectories->get("pos_x").addPoint(m_params.rise_time, m_params.trunk_x);
    m_trunk_trajectories->get("pos_y").addPoint(m_params.rise_time, m_params.foot_distance / 2.0);
    m_trunk_trajectories->get("pos_z").addPoint(m_params.rise_time, m_params.trunk_height);
    m_trunk_trajectories->get("roll").addPoint(m_params.rise_time, 0);
    m_trunk_trajectories->get("pitch").addPoint(m_params.rise_time, m_params.trunk_pitch);
    m_trunk_trajectories->get("yaw").addPoint(m_params.rise_time, 0);
}


void DynupEngine::start(bool front, geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose) {
    /*
     * Add current position, target position and current position to splines so that they describe a smooth
     * curve to the ball and back
     */
    /* Splines:
     * - if front:
     *   - move arms to frint and pull legs
     *   - get torso into 45°, pull foot under legs
     *   - get into crouch position
     * - if back:
     *
     * - after both:
     *    - slowly stand up with stabilization
     *    - move arms in finish position
     */

    init_trajectories();

     /*if(front){
     //TODO decide on which side we are lying on
        calc_front_splines();
     }else{
        calc_back_splines();
     }*/
     calc_squat_splines(l_foot_pose, trunk_pose);
}

void DynupEngine::init_trajectories() {
    m_foot_trajectories = Trajectories();

    m_foot_trajectories->add("pos_x");
    m_foot_trajectories->add("pos_y");
    m_foot_trajectories->add("pos_z");

    m_foot_trajectories->add("roll");
    m_foot_trajectories->add("pitch");
    m_foot_trajectories->add("yaw");

    m_trunk_trajectories = Trajectories();

    m_trunk_trajectories->add("pos_x");
    m_trunk_trajectories->add("pos_y");
    m_trunk_trajectories->add("pos_z");

    m_trunk_trajectories->add("roll");
    m_trunk_trajectories->add("pitch");
    m_trunk_trajectories->add("yaw");

    m_hand_trajectories = Trajectories();

    m_hand_trajectories->add("pos_x");
    m_hand_trajectories->add("pos_y");
    m_hand_trajectories->add("pos_z");

    m_hand_trajectories->add("roll");
    m_hand_trajectories->add("pitch");
    m_hand_trajectories->add("yaw");
}

int DynupEngine::get_percent_done() const {
    double duration = m_params.rise_time;
    return int(m_time / duration * 100);
}

void DynupEngine::set_params(DynUpParams params) {
    m_params = params;
}
