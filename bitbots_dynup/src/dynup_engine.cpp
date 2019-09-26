#include "bitbots_dynup/dynup_engine.h"

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::reset() {
  time_ = 0;
  hand_trajectories_.reset();
  foot_trajectories_.reset();
}

std::optional<JointGoals> DynupEngine::tick(double dt) {
  /* Only do an actual tick when splines are present */
  if (hand_trajectories_ && foot_trajectories_) {
    /* Get should-be pose from planned splines (every axis) at current time */
    geometry_msgs::PoseStamped l_foot_pose = getCurrentPose(foot_trajectories_.value(), true);
    geometry_msgs::PoseStamped trunk_pose = getCurrentPose(trunk_trajectories_.value(), false);
    //geometry_msgs::PoseStamped l_hand_pose = get_current_pose(foot_trajectories_.value(), "l_hand");
    //geometry_msgs::PoseStamped r_hand_pose = get_current_pose(foot_trajectories_.value(), "r_hand");


    time_ += dt;
    //TODO support point between feet
    geometry_msgs::Point support_point;
    /* Stabilize and return result */
    return stabilizer.stabilize(support_point, l_foot_pose, trunk_pose);
  } else {
    return std::nullopt;
  }
}

geometry_msgs::PoseStamped DynupEngine::getCurrentPose(Trajectories spline_container, bool foot) {
  geometry_msgs::PoseStamped pose;
  if (foot) {
    pose.header.frame_id = "l_sole";
  } else {
    pose.header.frame_id = "torso";
  }
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = spline_container.get("pos_x").pos(time_);
  pose.pose.position.y = spline_container.get("pos_y").pos(time_);
  pose.pose.position.z = spline_container.get("pos_z").pos(time_);
  tf2::Quaternion q;
  /* Apparently, the axis order is different than expected */
  q.setEuler(spline_container.get("pitch").pos(time_),
             spline_container.get("roll").pos(time_),
             spline_container.get("yaw").pos(time_));
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

void DynupEngine::calcFrontSplines() {

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

  hand_trajectories_->get("pos_x").addPoint(time_start, hand_pose.position.x);
  hand_trajectories_->get("pos_y").addPoint(time_start, hand_pose.position.y);
  hand_trajectories_->get("pos_z").addPoint(time_start, hand_pose.position.z);

  /* Construct a start_rotation as quaternion from Pose msg */
  tf2::Quaternion hand_start_rotation(hand_pose.orientation.x, hand_pose.orientation.y,
                                      hand_pose.orientation.z, hand_pose.orientation.w);
  double hand_start_r, hand_start_p, hand_start_y;
  tf2::Matrix3x3(hand_start_rotation).getRPY(hand_start_r, hand_start_p, hand_start_y);
  hand_trajectories_->get("roll").addPoint(time_start, hand_start_r);
  hand_trajectories_->get("pitch").addPoint(time_start, hand_start_p);
  hand_trajectories_->get("yaw").addPoint(time_start, hand_start_y);

  // foot
  geometry_msgs::Pose foot_pose; //TODO read actual pose
  foot_trajectories_->get("pos_x").addPoint(time_start, foot_pose.position.x);
  foot_trajectories_->get("pos_y").addPoint(time_start, foot_pose.position.y);
  foot_trajectories_->get("pos_z").addPoint(time_start, foot_pose.position.z);

  /* Construct a start_rotation as quaternion from Pose msg */
  tf2::Quaternion foot_start_rotation(foot_pose.orientation.x, foot_pose.orientation.y,
                                      foot_pose.orientation.z, foot_pose.orientation.w);
  double foot_start_r, foot_start_p, foot_start_y;
  tf2::Matrix3x3(foot_start_rotation).getRPY(foot_start_r, foot_start_p, foot_start_y);
  foot_trajectories_->get("roll").addPoint(time_start, foot_start_r);
  foot_trajectories_->get("pitch").addPoint(time_start, foot_start_p);
  foot_trajectories_->get("yaw").addPoint(time_start, foot_start_y);


  //TODO spline in between to enable the hands to go to the front

  /*
   * pull legs to body
  */
  double time_foot_close = params_.time_foot_close; // TODO
  foot_trajectories_->get("pos_x").addPoint(time_foot_close, 0);
  foot_trajectories_->get("pos_y").addPoint(time_foot_close, 0);
  foot_trajectories_->get("pos_z").addPoint(time_foot_close, params_.leg_min_length);
  foot_trajectories_->get("roll").addPoint(time_foot_close, 0);
  foot_trajectories_->get("pitch").addPoint(time_foot_close, 0);
  foot_trajectories_->get("yaw").addPoint(time_foot_close, 0);


  /*
   * hands to the front
   */
  double time_hands_front = params_.time_hands_front; //TODO parameter
  hand_trajectories_->get("pos_x").addPoint(time_hands_front, 0);
  hand_trajectories_->get("pos_y").addPoint(time_hands_front, 0);
  hand_trajectories_->get("pos_z").addPoint(time_hands_front, params_.arm_max_length);
  hand_trajectories_->get("roll").addPoint(time_hands_front, 0);
  hand_trajectories_->get("pitch").addPoint(time_hands_front, 3.14); //todo pi
  hand_trajectories_->get("yaw").addPoint(time_hands_front, 0);

  /*
   * Foot under body
   */
  double time_foot_ground = params_.time_foot_ground; //TODO
  foot_trajectories_->get("pos_x").addPoint(time_foot_ground, 0);
  foot_trajectories_->get("pos_y").addPoint(time_foot_ground, 0);
  foot_trajectories_->get("pos_z").addPoint(time_foot_ground, params_.leg_min_length);
  foot_trajectories_->get("roll").addPoint(time_foot_ground, 0);
  foot_trajectories_->get("pitch").addPoint(time_foot_ground, 3.14); //todo pi
  foot_trajectories_->get("yaw").addPoint(time_foot_ground, 0);


  /*
   * Torso 45°
   */
  double time_torso_45 = params_.time_torso_45; //TODO
  hand_trajectories_->get("pos_x").addPoint(time_torso_45, params_.arm_max_length);
  hand_trajectories_->get("pos_y").addPoint(time_torso_45, 0);
  hand_trajectories_->get("pos_z").addPoint(time_torso_45, 0);
  hand_trajectories_->get("roll").addPoint(time_torso_45, 0);
  hand_trajectories_->get("pitch").addPoint(time_torso_45, 0);
  hand_trajectories_->get("yaw").addPoint(time_torso_45, 0);

  foot_trajectories_->get("pos_x").addPoint(time_torso_45, 0);
  foot_trajectories_->get("pos_y").addPoint(time_torso_45, 0);
  foot_trajectories_->get("pos_z").addPoint(time_torso_45, params_.leg_min_length);
  foot_trajectories_->get("roll").addPoint(time_torso_45, 0);
  foot_trajectories_->get("pitch").addPoint(time_torso_45, 3.14); //todo pi
  foot_trajectories_->get("yaw").addPoint(time_torso_45, 0);

}

void DynupEngine::calcBackSplines() {

  //TODO from back to squat

}

void DynupEngine::calcSquatSplines(geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose) {

  // current position as first spline point
  // all positions relative to right foot

  // foot_trajectories_ are for left foot
  foot_trajectories_->get("pos_x").addPoint(0, l_foot_pose.position.x);
  foot_trajectories_->get("pos_y").addPoint(0, l_foot_pose.position.y);
  foot_trajectories_->get("pos_z").addPoint(0, l_foot_pose.position.z);
  double r, p, y;
  tf2::Quaternion q;
  tf2::convert(l_foot_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  foot_trajectories_->get("roll").addPoint(0, r);
  foot_trajectories_->get("pitch").addPoint(0, p);
  foot_trajectories_->get("yaw").addPoint(0, y);

  foot_trajectories_->get("pos_x").addPoint(params_.rise_time, 0);
  foot_trajectories_->get("pos_y").addPoint(params_.rise_time, params_.foot_distance);
  foot_trajectories_->get("pos_z").addPoint(params_.rise_time, 0);
  foot_trajectories_->get("roll").addPoint(params_.rise_time, 0);
  foot_trajectories_->get("pitch").addPoint(params_.rise_time, 0);
  foot_trajectories_->get("yaw").addPoint(params_.rise_time, 0);

  // trunk_trajectories_ are for trunk (relative to right foot)
  trunk_trajectories_->get("pos_x").addPoint(0, trunk_pose.position.x);
  trunk_trajectories_->get("pos_y").addPoint(0, trunk_pose.position.y);
  trunk_trajectories_->get("pos_z").addPoint(0, trunk_pose.position.z);
  tf2::convert(trunk_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  trunk_trajectories_->get("roll").addPoint(0, r);
  trunk_trajectories_->get("pitch").addPoint(0, p);
  trunk_trajectories_->get("yaw").addPoint(0, y);

  trunk_trajectories_->get("pos_x").addPoint(params_.rise_time, params_.trunk_x);
  trunk_trajectories_->get("pos_y").addPoint(params_.rise_time, params_.foot_distance / 2.0);
  trunk_trajectories_->get("pos_z").addPoint(params_.rise_time, params_.trunk_height);
  trunk_trajectories_->get("roll").addPoint(params_.rise_time, 0);
  trunk_trajectories_->get("pitch").addPoint(params_.rise_time / 2.0, params_.trunk_pitch);
  trunk_trajectories_->get("pitch").addPoint(params_.rise_time, params_.trunk_pitch);
  trunk_trajectories_->get("yaw").addPoint(params_.rise_time, 0);
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

  stabilizer.reset();
  initTrajectories();

  /*if(front){
  //TODO decide on which side we are lying on
     calcFrontSplines();
  }else{
     calcBackSplines();
  }*/
  calcSquatSplines(l_foot_pose, trunk_pose);
}

void DynupEngine::initTrajectories() {
  foot_trajectories_ = Trajectories();

  foot_trajectories_->add("pos_x");
  foot_trajectories_->add("pos_y");
  foot_trajectories_->add("pos_z");

  foot_trajectories_->add("roll");
  foot_trajectories_->add("pitch");
  foot_trajectories_->add("yaw");

  trunk_trajectories_ = Trajectories();

  trunk_trajectories_->add("pos_x");
  trunk_trajectories_->add("pos_y");
  trunk_trajectories_->add("pos_z");

  trunk_trajectories_->add("roll");
  trunk_trajectories_->add("pitch");
  trunk_trajectories_->add("yaw");

  hand_trajectories_ = Trajectories();

  hand_trajectories_->add("pos_x");
  hand_trajectories_->add("pos_y");
  hand_trajectories_->add("pos_z");

  hand_trajectories_->add("roll");
  hand_trajectories_->add("pitch");
  hand_trajectories_->add("yaw");
}

int DynupEngine::getPercentDone() const {
  double duration = params_.rise_time;
  return int(time_ / duration * 100);
}

void DynupEngine::setParams(DynUpParams params) {
  params_ = params;
}

}
