/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/walk_engine.h"

namespace bitbots_quintic_walk {

QuinticWalk::QuinticWalk() :
    footstep_(0.14, true),
    phase_(0.0),
    last_phase_(0.0),
    pause_requested_(false),
    left_kick_requested_(false),
    right_kick_requested_(false),
    params_(),
    trunk_pos_at_last_(),
    trunk_vel_at_last_(),
    trunk_acc_at_last_(),
    trunk_axis_pos_at_last_(),
    trunk_axis_vel_at_last_(),
    trunk_axis_acc_at_last_(),
    trajs_(),
    time_paused_(0.0) {
  // make sure to call the reset method after having the parameters
  trajs_ = trajectoriesInit();
}

bool QuinticWalk::updateState(double dt, const Eigen::Vector3d &orders, bool walkable_state) {
  bool orders_zero = orders[0]==0.0 && orders[1]==0.0 && orders[2]==0.0;

  // First check if we are currently in pause state or idle, since we don't want to update the phase in this case
  if (engine_state_=="paused") {
    if (time_paused_ > params_.pause_duration) {
      // our pause is finished, see if we can continue walking
      if (pause_requested_) {
        // not yet, wait another pause duration
        pause_requested_ = false;
        time_paused_ = 0.0;
        return false;
      } else {
        // we can continue
        engine_state_ = "walking";
        time_paused_ = 0.0;
        /*buildNormalTrajectories(orders);
        updatePhase(dt);
        return true;*/
      }
    } else {
      time_paused_ += dt;
      return false;
    }
    // we don't have to update anything more
  } else if (engine_state_=="idle") {
    if (orders_zero || !walkable_state) {
      // we are in idle and are not supposed to walk. current state is fine, just do nothing
      return false;
    }
  }

  // update the current phase
  updatePhase(dt);

  // check if we will finish a half step with this update
  bool half_step_finished = (last_phase_ < 0.5 && phase_ >= 0.5) || (last_phase_ > 0.5 && phase_ < 0.5);

  // small state machine
  if (engine_state_=="idle") {
    // state is idle and orders are not zero, we can start walking
    buildStartTrajectories(orders);
    engine_state_ = "startMovement";
  } else if (engine_state_=="startMovement") {
    // in this state we do a single "step" where we only move the trunk
    if (half_step_finished) {
      if (orders_zero) {
        engine_state_ = "stopMovement";
        buildStopMovementTrajectories(orders);
      } else {
        //start step is finished, go to next state
        buildTrajectories(orders, false, true, false);
        engine_state_ = "startStep";
      }
    }
  } else if (engine_state_=="startStep") {
    if (half_step_finished) {
      if (orders_zero) {
        // we have zero command vel -> we should stop
        engine_state_ = "stopStep";
        //phase_ = 0.0;
        buildStopStepTrajectories(orders);
      } else {
        //start step is finished, go to next state
        buildNormalTrajectories(orders);
        engine_state_ = "walking";
      }
    }
  } else if (engine_state_=="walking") {
    // check if a half step was finished and we are unstable
    if (half_step_finished && pause_requested_) {
      // go into pause
      engine_state_ = "paused";
      pause_requested_ = false;
      return false;
    } else if (half_step_finished &&
        ((left_kick_requested_ && !footstep_.isLeftSupport()) || (right_kick_requested_ && footstep_.isLeftSupport()))) {
      // lets do a kick
      buildKickTrajectories(orders);
      engine_state_ = "kick";
      left_kick_requested_ = false;
      right_kick_requested_ = false;
    } else if (half_step_finished) {
      // current step is finished, lets see if we have to change state
      if (orders_zero) {
        // we have zero command vel -> we should stop
        engine_state_ = "stopStep";
        //phase_ = 0.0;
        buildStopStepTrajectories(orders);
      } else {
        // we can keep on walking
        buildNormalTrajectories(orders);
      }
    }
  } else if (engine_state_=="kick") {
    // in this state we do a kick while doing a step
    if (half_step_finished) {
      //kick step is finished, go on walking
      engine_state_ = "walking";
      buildNormalTrajectories(orders);
    }
  } else if (engine_state_=="stopStep") {
    // in this state we do a step back to get feet into idle pose
    if (half_step_finished) {
      //stop step is finished, go to stop movement state
      engine_state_ = "stopMovement";
      buildStopMovementTrajectories(orders);
    }
  } else if (engine_state_=="stopMovement") {
    // in this state we do a "step" where we move the trunk back to idle position
    if (half_step_finished) {
      //stop movement is finished, go to idle state
      engine_state_ = "idle";
      return false;
    }
  } else {
    ROS_ERROR("Somethings wrong with the walking engine state");
  }

  //Sanity check support foot state
  if ((phase_ < 0.5 && !footstep_.isLeftSupport()) ||
      (phase_ >= 0.5 && footstep_.isLeftSupport())) {
    ROS_ERROR_THROTTLE(1,
                       "QuinticWalk exception invalid state phase= %f support= %d dt= %f",
                       phase_,
                       footstep_.isLeftSupport(),
                       dt);
    return false;
  }
  last_phase_ = phase_;

  return true;
}

void QuinticWalk::updatePhase(double dt) {
  //Check for negative time step
  if (dt <= 0.0) {
    if (dt==0.0) { //sometimes happens due to rounding
      dt = 0.0001;
    } else {
      ROS_ERROR_THROTTLE(1, "QuinticWalk exception negative dt phase= %f dt= %f", phase_, dt);
      return;
    }
  }
  //Check for too long dt
  if (dt > 0.25/params_.freq) {
    ROS_ERROR_THROTTLE(1, "QuinticWalk error too long dt phase= %f dt= %f", phase_, dt);
    return;
  }

  //Update the phase
  phase_ += dt*params_.freq;

  // reset to 0 if step complete
  if (phase_ > 1.0) {
    phase_ = 0.0;
  }
}

void QuinticWalk::endStep() {
  // ends the step earlier, e.g. when foot has already contact to ground
  if (phase_ < 0.5) {
    phase_ = 0.5;
  } else {
    phase_ = 0.0;
  }
}

void QuinticWalk::reset() {
  engine_state_ = "idle";
  phase_ = 0.0;
  time_paused_ = 0.0;

  //Initialize the footstep
  footstep_.setFootDistance(params_.foot_distance);
  footstep_.reset(false);
  //Reset the trunk saved state
  resetTrunkLastState();
}

void QuinticWalk::saveCurrentTrunkState() {
  //Evaluate current trunk state
  //(position, velocity, acceleration)
  //in next support foot frame

  // compute current point in time to save state
  // by multiplying the half_period time with the advancement of period time
  double half_period = 1.0/(2.0*params_.freq);
  double factor;
  if (last_phase_ < 0.5) {
    factor = last_phase_/0.5;
  } else {
    factor = last_phase_;
  }
  double period_time = half_period*factor;

  Eigen::Vector2d trunk_pos(
      trajs_.get("trunk_pos_x").pos(period_time),
      trajs_.get("trunk_pos_y").pos(period_time));
  Eigen::Vector2d trunk_vel(
      trajs_.get("trunk_pos_x").vel(period_time),
      trajs_.get("trunk_pos_y").vel(period_time));
  Eigen::Vector2d trunk_acc(
      trajs_.get("trunk_pos_x").acc(period_time),
      trajs_.get("trunk_pos_y").acc(period_time));
  //Convert in next support foot frame
  trunk_pos.x() -= footstep_.getNext().x();
  trunk_pos.y() -= footstep_.getNext().y();
  trunk_pos = Eigen::Rotation2Dd(
      -footstep_.getNext().z()).toRotationMatrix()
      *trunk_pos;
  trunk_vel = Eigen::Rotation2Dd(
      -footstep_.getNext().z()).toRotationMatrix()
      *trunk_vel;
  trunk_acc = Eigen::Rotation2Dd(
      -footstep_.getNext().z()).toRotationMatrix()
      *trunk_acc;
  //Save state
  trunk_pos_at_last_.x() = trunk_pos.x();
  trunk_pos_at_last_.y() = trunk_pos.y();
  trunk_vel_at_last_.x() = trunk_vel.x();
  trunk_vel_at_last_.y() = trunk_vel.y();
  trunk_acc_at_last_.x() = trunk_acc.x();
  trunk_acc_at_last_.y() = trunk_acc.y();
  //No transformation for height
  trunk_pos_at_last_.z() = trajs_.get("trunk_pos_z").pos(period_time);
  trunk_vel_at_last_.z() = trajs_.get("trunk_pos_z").vel(period_time);
  trunk_acc_at_last_.z() = trajs_.get("trunk_pos_z").acc(period_time);
  //Evaluate and save trunk orientation
  //in next support foot frame
  Eigen::Vector3d trunk_axis(
      trajs_.get("trunk_axis_x").pos(period_time),
      trajs_.get("trunk_axis_y").pos(period_time),
      trajs_.get("trunk_axis_z").pos(period_time));
  //Convert in intrinsic euler angle
  Eigen::Matrix3d trunk_mat = bitbots_splines::AxisToMatrix(trunk_axis);
  Eigen::Vector3d trunk_euler = bitbots_splines::MatrixToEulerIntrinsic(trunk_mat);
  //Transform to next support foot
  trunk_euler.z() -= footstep_.getNext().z();
  //Reconvert to axis and save it
  trunk_mat = bitbots_splines::EulerIntrinsicToMatrix(trunk_euler);
  trunk_axis = bitbots_splines::MatrixToAxis(trunk_mat);
  trunk_axis_pos_at_last_ = trunk_axis;
  //Evaluate trunk orientation velocity
  //and acceleration without frame
  //transformation
  trunk_axis_vel_at_last_.x() = trajs_.get("trunk_axis_x").vel(period_time);
  trunk_axis_vel_at_last_.y() = trajs_.get("trunk_axis_y").vel(period_time);
  trunk_axis_vel_at_last_.z() = trajs_.get("trunk_axis_z").vel(period_time);
  trunk_axis_acc_at_last_.x() = trajs_.get("trunk_axis_x").acc(period_time);
  trunk_axis_acc_at_last_.y() = trajs_.get("trunk_axis_y").acc(period_time);
  trunk_axis_acc_at_last_.z() = trajs_.get("trunk_axis_z").acc(period_time);
}

void QuinticWalk::buildNormalTrajectories(const Eigen::Vector3d &orders) {
  buildTrajectories(orders, false, false, false);
}

void QuinticWalk::buildKickTrajectories(const Eigen::Vector3d &orders) {
  buildTrajectories(orders, false, false, true);
}

void QuinticWalk::buildStartTrajectories(const Eigen::Vector3d &orders) {
  buildTrajectories(orders, true, false, false);
}

void QuinticWalk::buildStopStepTrajectories(const Eigen::Vector3d &orders) {
  buildWalkDisableTrajectories(orders, false);
}

void QuinticWalk::buildStopMovementTrajectories(const Eigen::Vector3d &orders) {
  buildWalkDisableTrajectories(orders, true);
}

void QuinticWalk::buildTrajectories(const Eigen::Vector3d &orders, bool start_movement, bool start_step, bool kick_step) {
  // save the current trunk state to use it later
  if (!start_movement) {
    saveCurrentTrunkState();
  } else {
    trunk_pos_at_last_.y() -= footstep_.getNext().y();
    //trunkPos = Eigen::Rotation2Dd(-_footstep.getNext().z()).toRotationMatrix() * trunkPos;
  }

  if (start_movement) {
    // update support foot and compute odometry
    footstep_.stepFromOrders(Eigen::Vector3d::Zero());
  } else {
    footstep_.stepFromOrders(orders);
  }

  //Reset the trajectories
  trajs_ = trajectoriesInit();
  //Set up the trajectories for the half cycle (single step)
  double half_period = 1.0/(2.0*params_.freq);
  // full period (double step) is needed for trunk splines
  double period = 2.0*half_period;

  //Time length of double and single support phase during the half cycle
  double double_support_length = params_.double_support_ratio*half_period;
  double single_support_length = half_period - double_support_length;

  //Sign of support foot with respect to lateral
  double support_sign = (footstep_.isLeftSupport() ? 1.0 : -1.0);

  //The trunk trajectory is defined for a
  //complete cycle to handle trunk phase shift
  //Trunk phase shift is done due to the length of the double
  //support phase and can be adjusted optionally by a parameter
  // 0.5halfPeriod to be acyclic to the feet, 0.5doubleSupportLength to keep the double support phase centered between feet
  double time_shift = -0.5*half_period + 0.5*double_support_length + params_.trunk_phase*half_period;


  //Only move the trunk on the first half cycle after a walk enable
  if (start_movement) {
    double_support_length = half_period;
    single_support_length = 0.0;
  }
  //Set double support phase
  point("is_double_support", 0.0, 1.0);
  point("is_double_support", double_support_length, 1.0);
  point("is_double_support", double_support_length, 0.0);
  point("is_double_support", half_period, 0.0);

  //Set support foot
  point("is_left_support_foot", 0.0, footstep_.isLeftSupport());
  point("is_left_support_foot", half_period, footstep_.isLeftSupport());

  //Flying foot position
  point("foot_pos_x", 0.0, footstep_.getLast().x());
  point("foot_pos_x", double_support_length, footstep_.getLast().x());
  if (kick_step) {
    point("foot_pos_x", double_support_length + single_support_length*params_.kick_phase,
          footstep_.getNext().x() + params_.kick_length,
          params_.kick_vel);
  } else {
    point("foot_pos_x",
          double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
          footstep_.getNext().x() +
              footstep_.getNext().x()*params_.foot_overshoot_ratio);
  }
  point("foot_pos_x", double_support_length + single_support_length*params_.foot_put_down_phase,
        footstep_.getNext().x());
  point("foot_pos_x", half_period, footstep_.getNext().x());

  point("foot_pos_y", 0.0, footstep_.getLast().y());
  point("foot_pos_y", double_support_length, footstep_.getLast().y());
  point("foot_pos_y", double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
        footstep_.getNext().y() + (footstep_.getNext().y() - footstep_.getLast().y())*params_.foot_overshoot_ratio);
  point("foot_pos_y", double_support_length + single_support_length*params_.foot_put_down_phase,
        footstep_.getNext().y());
  point("foot_pos_y", half_period, footstep_.getNext().y());

  point("foot_pos_z", 0.0, 0.0);
  point("foot_pos_z", double_support_length, 0.0);
  point("foot_pos_z",
        double_support_length + single_support_length*params_.foot_apex_phase - 0.5*params_.foot_z_pause*single_support_length,
        params_.foot_rise);
  point("foot_pos_z",
        double_support_length + single_support_length*params_.foot_apex_phase + 0.5*params_.foot_z_pause*single_support_length,
        params_.foot_rise);
  point("foot_pos_z", double_support_length + single_support_length*params_.foot_put_down_phase,
        params_.foot_put_down_z_offset);
  point("foot_pos_z", half_period, 0.0);

  //Flying foot orientation
  point("foot_axis_x", 0.0, 0.0);
  point("foot_axis_x", double_support_length + 0.1*single_support_length,
        0.0);
  point("foot_axis_x", double_support_length + single_support_length*params_.foot_put_down_phase,
        params_.foot_put_down_roll_offset*support_sign);
  point("foot_axis_x", half_period, 0.0);

  point("foot_axis_y", 0.0, 0.0);
  point("foot_axis_y", half_period, 0.0);

  point("foot_axis_z", 0.0, footstep_.getLast().z());
  point("foot_axis_z", double_support_length, footstep_.getLast().z());
  point("foot_axis_z", double_support_length + single_support_length*params_.foot_put_down_phase,
        footstep_.getNext().z());
  point("foot_axis_z", half_period, footstep_.getNext().z());


  //Half pause length of trunk swing
  //lateral oscillation
  double pause_length = 0.5*params_.trunk_pause*half_period;

  //Trunk support foot and next
  //support foot external
  //oscillating position
  Eigen::Vector2d trunk_point_support(
      params_.trunk_x_offset
          + params_.trunk_x_offset_p_coef_forward*footstep_.getNext().x()
          + params_.trunk_x_offset_p_coef_turn*fabs(footstep_.getNext().z()),
      params_.trunk_y_offset);
  Eigen::Vector2d trunk_point_next(
      footstep_.getNext().x() + params_.trunk_x_offset
          + params_.trunk_x_offset_p_coef_forward*footstep_.getNext().x()
          + params_.trunk_x_offset_p_coef_turn*fabs(footstep_.getNext().z()),
      footstep_.getNext().y() + params_.trunk_y_offset);
  //Trunk middle neutral (no swing) position
  Eigen::Vector2d trunk_point_middle =
      0.5*trunk_point_support + 0.5*trunk_point_next;
  //Trunk vector from middle to support apex
  Eigen::Vector2d trunk_vect =
      trunk_point_support - trunk_point_middle;
  //Apply swing amplitude ratio
  trunk_vect.y() *= params_.trunkSwing;
  //Trunk support and next apex position
  Eigen::Vector2d trunk_apex_support =
      trunk_point_middle + trunk_vect;
  Eigen::Vector2d trunk_apex_next =
      trunk_point_middle - trunk_vect;
  //Trunk forward velocity
  double trunk_vel_support =
      (footstep_.getNext().x() - footstep_.getLast().x())/period;
  double trunk_vel_next =
      footstep_.getNext().x()/half_period;

  //Trunk position
  if (start_step) {
    point("trunk_pos_x", 0.0,
          0.0,
          0.0,
          0.0);
  } else {
    point("trunk_pos_x", 0.0,
          trunk_pos_at_last_.x(),
          trunk_vel_at_last_.x(),
          trunk_acc_at_last_.x());
    point("trunk_pos_x", half_period + time_shift,
          trunk_apex_support.x(),
          trunk_vel_support);
  }
  point("trunk_pos_x", period + time_shift,
        trunk_apex_next.x(),
        trunk_vel_next);

  point("trunk_pos_y", 0.0,
        trunk_pos_at_last_.y(),
        trunk_vel_at_last_.y(),
        trunk_acc_at_last_.y());
  if (start_step || start_movement) {
    point("trunk_pos_y", half_period + time_shift - pause_length,
          trunk_point_middle.y() + trunk_vect.y()*params_.first_step_swing_factor);
    point("trunk_pos_y", half_period + time_shift + pause_length,
          trunk_point_middle.y() + trunk_vect.y()*params_.first_step_swing_factor);
    point("trunk_pos_y", period + time_shift - pause_length,
          trunk_point_middle.y() - trunk_vect.y()*params_.first_step_swing_factor);
    point("trunk_pos_y", period + time_shift + pause_length,
          trunk_point_middle.y() - trunk_vect.y()*params_.first_step_swing_factor);
  } else {
    point("trunk_pos_y", half_period + time_shift - pause_length,
          trunk_apex_support.y());
    point("trunk_pos_y", half_period + time_shift + pause_length,
          trunk_apex_support.y());
    point("trunk_pos_y", period + time_shift - pause_length,
          trunk_apex_next.y());
    point("trunk_pos_y", period + time_shift + pause_length,
          trunk_apex_next.y());
  }

  point("trunk_pos_z", 0.0,
        trunk_pos_at_last_.z(),
        trunk_vel_at_last_.z(),
        trunk_acc_at_last_.z());
  point("trunk_pos_z", half_period + time_shift,
        params_.trunk_height);
  point("trunk_pos_z", period + time_shift,
        params_.trunk_height);

  //Define trunk yaw target
  //orientation position and velocity
  //in euler angle and conversion
  //to axis vector
  Eigen::Vector3d euler_at_support(
      0.0,
      params_.trunk_pitch
          + params_.trunk_pitch_p_coef_forward*footstep_.getNext().x()
          + params_.trunk_pitch_p_coef_turn*fabs(footstep_.getNext().z()),
      0.5*footstep_.getLast().z() + 0.5*footstep_.getNext().z());
  Eigen::Vector3d euler_at_next(
      0.0,
      params_.trunk_pitch
          + params_.trunk_pitch_p_coef_forward*footstep_.getNext().x()
          + params_.trunk_pitch_p_coef_turn*fabs(footstep_.getNext().z()),
      footstep_.getNext().z());
  Eigen::Matrix3d mat_at_support = bitbots_splines::EulerIntrinsicToMatrix(euler_at_support);
  Eigen::Matrix3d mat_at_next = bitbots_splines::EulerIntrinsicToMatrix(euler_at_next);
  Eigen::Vector3d axis_at_support = bitbots_splines::MatrixToAxis(mat_at_support);
  Eigen::Vector3d axis_at_next = bitbots_splines::MatrixToAxis(mat_at_next);
  Eigen::Vector3d axis_vel(
      0.0, 0.0,
      bitbots_splines::AngleDistance(
          footstep_.getLast().z(),
          footstep_.getNext().z())/period);

  //Trunk orientation
  point("trunk_axis_x", 0.0,
        trunk_axis_pos_at_last_.x(),
        trunk_axis_vel_at_last_.x(),
        trunk_axis_acc_at_last_.x());
  point("trunk_axis_x", half_period + time_shift,
        axis_at_support.x(),
        axis_vel.x());
  point("trunk_axis_x", period + time_shift,
        axis_at_next.x(),
        axis_vel.x());

  point("trunk_axis_y", 0.0,
        trunk_axis_pos_at_last_.y(),
        trunk_axis_vel_at_last_.y(),
        trunk_axis_acc_at_last_.y());
  point("trunk_axis_y", half_period + time_shift,
        axis_at_support.y(),
        axis_vel.y());
  point("trunk_axis_y", period + time_shift,
        axis_at_next.y(),
        axis_vel.y());

  point("trunk_axis_z", 0.0,
        trunk_axis_pos_at_last_.z(),
        trunk_axis_vel_at_last_.z(),
        trunk_axis_acc_at_last_.z());
  point("trunk_axis_z", half_period + time_shift,
        axis_at_support.z(),
        axis_vel.z());
  point("trunk_axis_z", period + time_shift,
        axis_at_next.z(),
        axis_vel.z());
}

void QuinticWalk::buildWalkDisableTrajectories(const Eigen::Vector3d &orders, bool foot_in_idle_position) {
  // save the current trunk state to use it later
  saveCurrentTrunkState();
  // update support foot and compute odometry
  footstep_.stepFromOrders(orders);

  //Reset the trajectories
  trajs_ = trajectoriesInit();

  //Set up the trajectories
  //for the half cycle
  double half_period = 1.0/(2.0*params_.freq);

  //Time length of double and single
  //support phase during the half cycle
  double double_support_length = params_.double_support_ratio*half_period;
  double single_support_length = half_period - double_support_length;

  //Sign of support foot with
  //respect to lateral
  double support_sign = (footstep_.isLeftSupport() ? 1.0 : -1.0);

  //Set double support phase
  double is_double_support = (foot_in_idle_position ? 1.0 : 0.0);
  point("is_double_support", 0.0, is_double_support);
  point("is_double_support", half_period, is_double_support);

  //Set support foot
  point("is_left_support_foot", 0.0, footstep_.isLeftSupport());
  point("is_left_support_foot", half_period, footstep_.isLeftSupport());

  //Flying foot position
  point("foot_pos_x", 0.0,
        footstep_.getLast().x());
  point("foot_pos_x", double_support_length,
        footstep_.getLast().x());
  point("foot_pos_x", double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
        0.0 + (0.0 - footstep_.getLast().x())*params_.foot_overshoot_ratio);
  point("foot_pos_x", double_support_length + single_support_length*params_.foot_put_down_phase,
        0.0);
  point("foot_pos_x", half_period,
        0.0);

  point("foot_pos_y", 0.0,
        footstep_.getLast().y());
  point("foot_pos_y", double_support_length,
        footstep_.getLast().y());
  point("foot_pos_y", double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
        -support_sign*params_.foot_distance
            + (-support_sign*params_.foot_distance - footstep_.getLast().y())*params_.foot_overshoot_ratio);
  point("foot_pos_x", double_support_length + single_support_length*params_.foot_put_down_phase,
        -support_sign*params_.foot_distance);
  point("foot_pos_y", half_period,
        -support_sign*params_.foot_distance);

  //If the walk has just been disabled,
  //make one single step to neutral pose
  if (!foot_in_idle_position) {
    point("foot_pos_z", 0.0, 0.0);
    point("foot_pos_z", double_support_length, 0.0);
    point("foot_pos_z",
          double_support_length + single_support_length*params_.foot_apex_phase - 0.5*params_.foot_z_pause*single_support_length,
          params_.foot_rise);
    point("foot_pos_z",
          double_support_length + single_support_length*params_.foot_apex_phase + 0.5*params_.foot_z_pause*single_support_length,
          params_.foot_rise);
    point("foot_pos_z", double_support_length + single_support_length*params_.foot_put_down_phase,
          params_.foot_put_down_z_offset);
    point("foot_pos_z", half_period,
          0.0);
  } else {
    //dont move the foot in last single step before stop since we only move the trunk back to the center
    point("foot_pos_z", 0.0, 0.0);
    point("foot_pos_z", half_period, 0.0);
  }
  //Flying foot orientation
  point("foot_axis_x", 0.0, 0.0);
  point("foot_axis_x", half_period, 0.0);

  point("foot_axis_y", 0.0, 0.0);
  point("foot_axis_y", half_period, 0.0);

  point("foot_axis_z", 0.0, footstep_.getLast().z());
  point("foot_axis_z", double_support_length,
        footstep_.getLast().z());
  point("foot_axis_z", double_support_length + single_support_length*params_.foot_put_down_phase,
        0.0);
  point("foot_axis_z", half_period, 0.0);

  //Trunk position
  point("trunk_pos_x", 0.0,
        trunk_pos_at_last_.x(),
        trunk_vel_at_last_.x(),
        trunk_acc_at_last_.x());
  point("trunk_pos_x", half_period,
        params_.trunk_x_offset);

  point("trunk_pos_y", 0.0,
        trunk_pos_at_last_.y(),
        trunk_vel_at_last_.y(),
        trunk_acc_at_last_.y());
  point("trunk_pos_y", half_period,
        -support_sign*0.5*params_.foot_distance + params_.trunk_y_offset);

  point("trunk_pos_z", 0.0,
        trunk_pos_at_last_.z(),
        trunk_vel_at_last_.z(),
        trunk_acc_at_last_.z());
  point("trunk_pos_z", half_period,
        params_.trunk_height);
  //Trunk orientation
  point("trunk_axis_x", 0.0,
        trunk_axis_pos_at_last_.x(),
        trunk_axis_vel_at_last_.x(),
        trunk_axis_acc_at_last_.x());
  point("trunk_axis_x", half_period, 0.0);
  point("trunk_axis_y", 0.0,
        trunk_axis_pos_at_last_.y(),
        trunk_axis_vel_at_last_.y(),
        trunk_axis_acc_at_last_.y());
  point("trunk_axis_y", half_period,
        params_.trunk_pitch);
  point("trunk_axis_z", 0.0,
        trunk_axis_pos_at_last_.z(),
        trunk_axis_vel_at_last_.z(),
        trunk_axis_acc_at_last_.z());
  point("trunk_axis_z", half_period,
        0.0);
}

void QuinticWalk::resetTrunkLastState() {
  if (footstep_.isLeftSupport()) {
    trunk_pos_at_last_ <<
                       params_.trunk_x_offset,
        -params_.foot_distance/2.0 + params_.trunk_y_offset,
        params_.trunk_height;
  } else {
    trunk_pos_at_last_ <<
                       params_.trunk_x_offset,
        params_.foot_distance/2.0 + params_.trunk_y_offset,
        params_.trunk_height;
  }
  trunk_vel_at_last_.setZero();
  trunk_acc_at_last_.setZero();
  trunk_axis_pos_at_last_ << 0.0, params_.trunk_pitch, 0.0;
  trunk_axis_vel_at_last_.setZero();
  trunk_axis_acc_at_last_.setZero();
}

void QuinticWalk::computeCartesianPosition(Eigen::Vector3d &trunk_pos,
                                           Eigen::Vector3d &trunk_axis,
                                           Eigen::Vector3d &foot_pos,
                                           Eigen::Vector3d &foot_axis,
                                           bool &is_leftsupport_foot) {
  //Compute trajectories time
  double time = getTrajsTime();

  computeCartesianPositionAtTime(trunk_pos, trunk_axis, foot_pos, foot_axis, is_leftsupport_foot, time);

}

void QuinticWalk::computeCartesianPositionAtTime(Eigen::Vector3d &trunk_pos,
                                                 Eigen::Vector3d &trunk_axis,
                                                 Eigen::Vector3d &foot_pos,
                                                 Eigen::Vector3d &foot_axis,
                                                 bool &is_leftsupport_foot,
                                                 double time) {
  //Evaluate target cartesian
  //state from trajectories
  bool is_double_support;
  trajectoriesTrunkFootPos(time, trajs_, trunk_pos, trunk_axis, foot_pos, foot_axis);
  trajectoriesSupportFootState(time, trajs_, is_double_support, is_leftsupport_foot);
}

void QuinticWalk::point(const std::string& spline, double t, double pos, double vel, double acc) {
  trajs_.get(spline).addPoint(t, pos, vel, acc);
}

double QuinticWalk::getPhase() const {
  return phase_;
}

double QuinticWalk::getTrajsTime() const {
  double t;
  if (phase_ < 0.5) {
    t = phase_/params_.freq;
  } else {
    t = (phase_ - 0.5)/params_.freq;
  }

  return t;
}

Footstep QuinticWalk::getFootstep() {
  return footstep_;
}

bool QuinticWalk::isLeftSupport() {
  return footstep_.isLeftSupport();
}

bool QuinticWalk::isDoubleSupport() {
  // returns true if the value of the "is_double_support" spline is currently higher than 0.5
  // the spline should only have values of 0 or 1
  return trajs_.get("is_double_support").pos(getTrajsTime()) >= 0.5;
}

void QuinticWalk::reconfCallback(const bitbots_quintic_walk_paramsConfig &params) {
  params_ = params;
  footstep_.setFootDistance(params_.foot_distance);
}

void QuinticWalk::requestKick(bool left) {
  if (left) {
    left_kick_requested_ = true;
  } else {
    right_kick_requested_ = true;
  }
}

void QuinticWalk::requestPause() {
  pause_requested_ = true;
}

std::string QuinticWalk::getState() {
  return engine_state_;
}

}

