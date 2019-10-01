#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H_

#include <optional>
#include <std_msgs/Header.h>
#include <bitbots_splines/pose_spline.h>
#include <bitbots_splines/position_spline.h>
#include <bitbots_splines/abstract_engine.h>
#include <bitbots_msgs/KickGoal.h>
#include <bitbots_msgs/KickFeedback.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include "stabilizer.h"
#include "visualizer.h"

namespace bitbots_dynamic_kick {

struct KickParams {
  double foot_rise;
  double foot_distance;
  double kick_windup_distance;
  double trunk_height;

  double move_trunk_time = 1;
  double raise_foot_time = 1;
  double move_to_ball_time = 1;
  double kick_time = 1;
  double move_back_time = 1;
  double lower_foot_time = 1;
  double move_trunk_back_time = 1;

  double stabilizing_point_x;
  double stabilizing_point_y;

  double choose_foot_corridor_width;
};

enum KickPhase {
  INITIAL,
  MOVE_TRUNK,
  RAISE_FOOT,
  WINDUP,
  KICK,
  MOVE_BACK,
  LOWER_FOOT,
  MOVE_TRUNK_BACK,
  DONE
};

/**
 * An instance of this class describes after which time a KickPhase is done
 */
class PhaseTimings {
 public:
  double move_trunk;
  double raise_foot;
  double windup;
  double kick;
  double move_back;
  double lower_foot;
  double move_trunk_back;
};

/**
 * The KickEngine takes care of choosing an optimal foot to reach a given goal,
 * planning that foots required movement (rotation and positioning)
 * and updating short-term MotorGoals to move (the foot) along that planned path.
 *
 * It is vital to call the engines tick() method repeatedly because that is where these short-term MotorGoals are
 * returned.
 *
 * The KickEngine utilizes a Stabilizer to balance the robot during foot movments.
 */
class KickEngine : public bitbots_splines::AbstractEngine<KickGoals, KickPositions> {
 public:
  KickEngine();

  /**
   * Set new goal which the engine tries to kick at. This will remove the old goal completely and plan new splines.
   * @param header Definition of frame and time in which the goals were published
   * @param ball_position Position of the ball
   * @param kick_direction Direction into which to kick the ball
   * @param kick_speed Speed with which to kick the ball
   * @param r_foot_pose Current pose of right foot in l_sole frame
   * @param l_foot_pose Current pose of left foot in r_sole frame
   *
   * @throws tf2::TransformException when goal cannot be converted into needed tf frames
   */
  void setGoals(const KickGoals &goals) override;

  /**
   * Reset this KickEngine completely, removing the goal, all splines and thereby stopping all output
   */
  void reset() override;

  /**
   * Do one iteration of spline-progress-updating. This means that whenever update() is called,
   *      new position goals are retrieved from previously calculated splines, stabilized and transformed into
   *      JointGoals
   * @param dt Passed delta-time between last call to update() and now. Measured in seconds
   * @return New motor goals only if a goal is currently set, position extractions from splines was possible and
   *      bio_ik was able to compute valid motor positions
   */
  KickPositions update(double dt) override;

  /**
   * Is the currently performed kick with the left foot or not
   */
  bool isLeftKick();

  int getPercentDone() const override;

  bitbots_splines::PoseSpline getFlyingSplines() const ;
  bitbots_splines::PoseSpline getTrunkSplines() const ;

  void setParams(KickParams params);

  KickPhase getPhase() const;

  tf2::Vector3 getWindupPoint();

 private:
  double time_;
  tf2::Vector3 ball_position_;
  tf2::Quaternion kick_direction_;
  float kick_speed_;
  bool is_left_kick_;
  bitbots_splines::PoseSpline flying_foot_spline_, trunk_spline_;
  KickParams params_;
  PhaseTimings phase_timings_;
  tf2::Vector3 windup_point_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  /**
   *  Calculate splines for a complete kick whereby is_left_kick_ should already be set correctly
   *
   *  @param flying_foot_pose Current pose of the foot which is supposed to be the flying/kicking one
   */
  void calcSplines(const geometry_msgs::Pose &flying_foot_pose, const geometry_msgs::Transform &trunk_pose);

  /**
   *  Calculate the point from which to perform the final kicking movement
   */
  tf2::Vector3 calcKickWindupPoint();

  /**
   * Choose with which foot the kick should be performed
   *
   * This is done by checking whether the ball is outside of a corridor ranging from base_footprint forward.
   * If it is, the foot on that side will be chosen as the kicking foot.
   * If not, a more fine grained angle based criterion is used.     *
   *
   * @param header Definition of frame and time in which the goals were published
   * @param ball_position Position where the ball is currently located
   * @param kick_direction Direction into which the ball should be kicked
   * @return Whether the resulting kick should be performed with the left foot
   *
   * @throws tf2::TransformException when ball_position and kick_direction cannot be converted into base_footprint frame
   */
  bool calcIsLeftFootKicking(const std_msgs::Header &header,
                             const geometry_msgs::Vector3 &ball_position,
                             const geometry_msgs::Quaternion &kick_direction);

  geometry_msgs::Transform getTrunkPose();


  /**
   * Calculate the yaw of the kicking foot, so that it is turned
   * in the direction of the kick
   */
  double calcKickFootYaw();

  /**
   * Transform then goal into our support_foots frame
   * @param support_foot_frame Name of the support foots frame, meaning where to transform to
   * @param header Frame and time in which the goals were published
   * @param ball_position Position of the ball
   * @param kick_direction Direction in which to kick the ball
   * @return pair of (transformed_pose, transformed_direction)
   *
   * @throws tf2::TransformException when goal cannot be transformed into support_foot_frame
   */
  std::pair<geometry_msgs::Point, geometry_msgs::Quaternion> transformGoal(
      const std::string &support_foot_frame,
      const std_msgs::Header &header,
      const geometry_msgs::Vector3 &ball_position,
      const geometry_msgs::Quaternion &kick_direction);
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H_
