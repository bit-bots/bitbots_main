/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/walk_engine.h"

namespace bitbots_quintic_walk {

WalkEngine::WalkEngine() :
    phase_(0.0),
    last_phase_(0.0),
    pause_requested_(false),
    left_kick_requested_(false),
    right_kick_requested_(false),
    time_paused_(0.0) {
  left_in_world_.setIdentity();
  right_in_world_.setIdentity();
  reset();

  // init dynamic reconfigure
  dyn_reconf_server_ =
      new dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_engine_paramsConfig>(ros::NodeHandle(
          "~/engine"));
  dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_engine_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_quintic_walk::WalkEngine::reconfCallback, this, _1, _2);
  dyn_reconf_server_->setCallback(f);
}

void WalkEngine::setGoals(const WalkRequest &goals) {
  request_ = goals;
}

WalkResponse WalkEngine::update(double dt) {
  // check if orders are zero, since we don't want to walk on the spot
  bool orders_zero = request_.orders.x() == 0 && request_.orders.y() == 0 && request_.orders.z() == 0;

  // First check if we are currently in pause state or idle, since we don't want to update the phase in this case
  if (engine_state_ == WalkState::PAUSED) {
    if (time_paused_ > pause_duration_) {
      // our pause is finished, see if we can continue walking
      if (pause_requested_) {
        // not yet, wait another pause duration
        pause_requested_ = false;
        time_paused_ = 0.0;
        return createResponse();
      } else {
        // we can continue
        engine_state_ = WalkState::WALKING;
        time_paused_ = 0.0;
      }
    } else {
      time_paused_ += dt;
      return createResponse();
    }
    // we don't have to update anything more
  } else if (engine_state_ == WalkState::IDLE) {
    if (orders_zero || !request_.walkable_state) {
      // we are in idle and are not supposed to walk. current state is fine, just do nothing
      return createResponse();
    }
  }

  // update the current phase
  updatePhase(dt);

  // check if we will finish a half step with this update
  bool half_step_finished = (last_phase_ < 0.5 && phase_ >= 0.5) || (last_phase_ > 0.5 && phase_ < 0.5);

  // small state machine
  if (engine_state_ == WalkState::IDLE) {
    // state is idle and orders are not zero, we can start walking
    buildStartMovementTrajectories();
    engine_state_ = WalkState::START_MOVEMENT;
  } else if (engine_state_ == WalkState::START_MOVEMENT) {
    // in this state we do a single "step" where we only move the trunk
    if (half_step_finished) {
      if (orders_zero) {
        engine_state_ = WalkState::STOP_MOVEMENT;
        buildStopMovementTrajectories();
      } else {
        //start movement is finished, go to next state
        buildStartStepTrajectories();
        engine_state_ = WalkState::START_STEP;
      }
    }
  } else if (engine_state_ == WalkState::START_STEP) {
    if (half_step_finished) {
      if (orders_zero) {
        // we have zero command vel -> we should stop
        engine_state_ = WalkState::STOP_STEP;
        //phase_ = 0.0;
        buildStopStepTrajectories();
      } else {
        //start step is finished, go to next state
        buildNormalTrajectories();
        engine_state_ = WalkState::WALKING;
      }
    }
  } else if (engine_state_ == WalkState::WALKING) {
    // check if a half step was finished and we are unstable
    if (half_step_finished && pause_requested_) {
      // go into pause
      engine_state_ = WalkState::PAUSED;
      pause_requested_ = false;
      return createResponse();
    } else if (half_step_finished &&
        ((left_kick_requested_ && !is_left_support_foot_)
            || (right_kick_requested_ && is_left_support_foot_))) {
      // lets do a kick
      buildKickTrajectories();
      engine_state_ = WalkState::KICK;
      left_kick_requested_ = false;
      right_kick_requested_ = false;
    } else if (half_step_finished) {
      // current step is finished, lets see if we have to change state
      if (orders_zero) {
        // we have zero command vel -> we should stop
        engine_state_ = WalkState::STOP_STEP;
        //phase_ = 0.0;
        buildStopStepTrajectories();
      } else {
        // we can keep on walking
        buildNormalTrajectories();
      }
    }
  } else if (engine_state_ == WalkState::KICK) {
    // in this state we do a kick while doing a step
    if (half_step_finished) {
      //kick step is finished, go on walking
      engine_state_ = WalkState::WALKING;
      buildNormalTrajectories();
    }
  } else if (engine_state_ == WalkState::STOP_STEP) {
    // in this state we do a step back to get feet into idle pose
    if (half_step_finished) {
      //stop step is finished, go to stop movement state
      engine_state_ = WalkState::STOP_MOVEMENT;
      buildStopMovementTrajectories();
    }
  } else if (engine_state_ == WalkState::STOP_MOVEMENT) {
    // in this state we do a "step" where we move the trunk back to idle position
    if (half_step_finished) {
      //stop movement is finished, go to idle state
      engine_state_ = WalkState::IDLE;
      return createResponse();
    }
  } else {
    ROS_ERROR("Something is wrong with the walking engine state");
  }

  //Sanity check support foot state
  if ((phase_ < 0.5 && !is_left_support_foot_) ||
      (phase_ >= 0.5 && is_left_support_foot_)) {
    ROS_ERROR_THROTTLE(1,
                       "WalkEngine exception invalid state phase= %f support= %d dt= %f",
                       phase_,
                       is_left_support_foot_,
                       dt);
    return createResponse();
  }
  last_phase_ = phase_;

  return createResponse();
}

void WalkEngine::updatePhase(double dt) {
  //Check for negative time step
  if (dt <= 0.0) {
    if (dt == 0.0) { //sometimes happens due to rounding
      dt = 0.0001;
    } else {
      ROS_ERROR_THROTTLE(1, "WalkEngine exception negative dt phase= %f dt= %f", phase_, dt);
      return;
    }
  }
  //Check for too long dt
  if (dt > 0.25 / params_.freq) {
    ROS_ERROR_THROTTLE(1, "WalkEngine error too long dt phase= %f dt= %f", phase_, dt);
    return;
  }

  //Update the phase
  phase_ += dt * params_.freq;

  // reset to 0 if step complete
  if (phase_ > 1.0) {
    phase_ = 0.0;
  }
}

void WalkEngine::endStep() {
  // ends the step earlier, e.g. when foot has already contact to ground
  if (phase_ < 0.5) {
    phase_ = 0.5;
  } else {
    phase_ = 0.0;
  }
}

void WalkEngine::reset() {
  request_.orders = {0, 0, 0};
  request_.walkable_state = false;
  engine_state_ = WalkState::IDLE;
  phase_ = 0.0;
  time_paused_ = 0.0;

  is_left_support_foot_ = false;
  support_to_last_.setIdentity();
  if (is_left_support_foot_) {
    support_to_last_.getOrigin()[1] = -params_.foot_distance;
  } else {
    support_to_last_.getOrigin()[1] = params_.foot_distance;
  }
  support_to_next_ = support_to_last_;

  //Reset the trunk saved state
  if (is_left_support_foot_) {
    trunk_pos_at_last_ = tf2::Vector3(
        params_.trunk_x_offset,
        -params_.foot_distance / 2.0 + params_.trunk_y_offset,
        params_.trunk_height);
  } else {
    trunk_pos_at_last_ = tf2::Vector3(
        params_.trunk_x_offset,
        params_.foot_distance / 2.0 + params_.trunk_y_offset,
        params_.trunk_height);
  }
  trunk_pos_vel_at_last_.setZero();
  trunk_pos_acc_at_last_.setZero();
  trunk_axis_pos_at_last_ = tf2::Vector3(0.0, params_.trunk_pitch, 0.0);
  trunk_axis_vel_at_last_.setZero();
  trunk_axis_acc_at_last_.setZero();
}

void WalkEngine::saveCurrentTrunkState() {
  //Evaluate current trunk state (position, velocity, acceleration) in next support foot frame

  // compute current point in time to save state
  // by multiplying the half_period time with the advancement of period time
  double half_period = 1.0 / (2.0 * params_.freq);
  double factor;
  if (last_phase_ < 0.5) {
    factor = last_phase_ / 0.5;
  } else {
    factor = last_phase_;
  }
  double period_time = half_period * factor;

  // get last values of trunk pose and its velocities and accelerations
  tf2::Vector3 trunk_pos = trunk_spline_.getPositionPos(period_time);
  tf2::Vector3 trunk_pos_vel = trunk_spline_.getPositionVel(period_time);
  tf2::Vector3 trunk_pos_acc = trunk_spline_.getPositionAcc(period_time);
  tf2::Vector3 trunk_axis_pos = trunk_spline_.getEulerAngles(period_time);
  tf2::Vector3 trunk_axis_vel = trunk_spline_.getEulerVel(period_time);
  tf2::Vector3 trunk_axis_acc = trunk_spline_.getEulerAcc(period_time);

  //Convert in next support foot frame and save
  trunk_pos_at_last_ = support_to_next_ * trunk_pos;
  trunk_pos_vel_at_last_ = support_to_next_ * trunk_pos_vel;
  trunk_pos_acc_at_last_ = support_to_next_ * trunk_pos_acc;
  trunk_axis_pos_at_last_ = support_to_next_ * trunk_axis_pos;
  trunk_axis_vel_at_last_ = support_to_next_ * trunk_axis_vel;
  trunk_axis_acc_at_last_ = support_to_next_ * trunk_axis_acc;
}

void WalkEngine::buildNormalTrajectories() {
  buildTrajectories(false, false, false);
}

void WalkEngine::buildKickTrajectories() {
  buildTrajectories(false, false, true);
}

void WalkEngine::buildStartMovementTrajectories() {
  buildTrajectories(true, false, false);
}

void WalkEngine::buildStartStepTrajectories() {
  buildTrajectories(false, true, false);
}

void WalkEngine::buildStopStepTrajectories() {
  buildWalkDisableTrajectories(false);
}

void WalkEngine::buildStopMovementTrajectories() {
  buildWalkDisableTrajectories(true);
}

void WalkEngine::buildTrajectories(bool start_movement, bool start_step, bool kick_step) {
  // save the current trunk state to use it later and compute the next step position
  if (!start_movement) {
    saveCurrentTrunkState();
    stepFromOrders(request_.orders);
  } else {
    // when we do start step, only transform the y coordinate since we stand still and only move trunk sideward
    trunk_pos_at_last_[1] = trunk_pos_at_last_.y() - support_to_next_.getOrigin().y();
    stepFromOrders({0, 0, 0});
  }

  //Reset the trajectories
  is_double_support_spline_ = bitbots_splines::SmoothSpline();
  is_left_support_foot_spline_ = bitbots_splines::SmoothSpline();
  trunk_spline_ = bitbots_splines::PoseSpline();
  foot_spline_ = bitbots_splines::PoseSpline();

  //Set up the trajectories for the half cycle (single step)
  double half_period = 1.0 / (2.0 * params_.freq);
  // full period (double step) is needed for trunk splines
  double period = 2.0 * half_period;

  // Time length of double and single support phase during the half cycle
  double double_support_length = params_.double_support_ratio * half_period;
  double single_support_length = half_period - double_support_length;

  // Sign of support foot with respect to lateral
  double support_sign = (is_left_support_foot_ ? 1.0 : -1.0);

  // The trunk trajectory is defined for a complete cycle to handle trunk phase shift Trunk phase shift is done due to
  // the length of the double support phase and can be adjusted optionally by a parameter 0.5 * half_period to be
  // acyclic to the feet, 0.5 * double_support_length to keep the double support phase centered between feet
  double time_shift = -0.5 * half_period + 0.5 * double_support_length + params_.trunk_phase * half_period;


  //Only move the trunk on the first half cycle after a walk enable
  if (start_movement) {
    double_support_length = half_period;
    single_support_length = 0.0;
  }
  //Set double support phase
  is_double_support_spline_.addPoint(0.0, 1.0);
  is_double_support_spline_.addPoint(double_support_length, 1.0);
  is_double_support_spline_.addPoint(double_support_length, 0.0);
  is_double_support_spline_.addPoint(half_period, 0.0);

  //Set support foot
  is_left_support_foot_spline_.addPoint(0.0, is_left_support_foot_);
  is_left_support_foot_spline_.addPoint(half_period, is_left_support_foot_);

  //Flying foot position
  foot_spline_.x()->addPoint(0.0, support_to_last_.getOrigin().x());
  foot_spline_.x()->addPoint(double_support_length, support_to_last_.getOrigin().x());
  if (kick_step) {
    foot_spline_.x()->addPoint(double_support_length + single_support_length * params_.kick_phase,
                               support_to_next_.getOrigin().x() + params_.kick_length,
                               params_.kick_vel);
  } else {
    foot_spline_.x()->addPoint(
        double_support_length + single_support_length * params_.foot_put_down_phase * params_.foot_overshoot_phase,
        support_to_next_.getOrigin().x() + support_to_next_.getOrigin().x() * params_.foot_overshoot_ratio);
  }
  foot_spline_.x()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             support_to_next_.getOrigin().x());
  foot_spline_.x()->addPoint(half_period, support_to_next_.getOrigin().x());

  foot_spline_.y()->addPoint(0.0, support_to_last_.getOrigin().y());
  foot_spline_.y()->addPoint(double_support_length, support_to_last_.getOrigin().y());
  foot_spline_.y()->addPoint(
      double_support_length + single_support_length * params_.foot_put_down_phase * params_.foot_overshoot_phase,
      support_to_next_.getOrigin().y()
          + (support_to_next_.getOrigin().y() - support_to_last_.getOrigin().y()) * params_.foot_overshoot_ratio);
  foot_spline_.y()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             support_to_next_.getOrigin().y());
  foot_spline_.y()->addPoint(half_period, support_to_next_.getOrigin().y());

  foot_spline_.z()->addPoint(0.0, 0.0);
  foot_spline_.z()->addPoint(double_support_length, 0.0);
  foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_apex_phase
                                 - 0.5 * params_.foot_z_pause * single_support_length,
                             params_.foot_rise);
  foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_apex_phase
                                 + 0.5 * params_.foot_z_pause * single_support_length,
                             params_.foot_rise);
  foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             params_.foot_put_down_z_offset);
  foot_spline_.z()->addPoint(half_period, 0.0);

  //Flying foot orientation
  foot_spline_.roll()->addPoint(0.0, 0.0);
  foot_spline_.roll()->addPoint(double_support_length + 0.1 * single_support_length, 0.0);
  foot_spline_.roll()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                                params_.foot_put_down_roll_offset * support_sign);
  foot_spline_.roll()->addPoint(half_period, 0.0);

  foot_spline_.pitch()->addPoint(0.0, 0.0);
  foot_spline_.pitch()->addPoint(half_period, 0.0);

  foot_spline_.yaw()->addPoint(0.0, getLastEuler().z());
  foot_spline_.yaw()->addPoint(double_support_length, getLastEuler().z());
  foot_spline_.yaw()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                               getNextEuler().z());
  foot_spline_.yaw()->addPoint(half_period, getNextEuler().z());


  //Half pause length of trunk swing
  //lateral oscillation
  double pause_length = 0.5 * params_.trunk_pause * half_period;

  //Trunk support foot and next
  //support foot external
  //oscillating position
  tf2::Vector3 trunk_point_support(
      params_.trunk_x_offset
          + params_.trunk_x_offset_p_coef_forward * support_to_next_.getOrigin().x()
          + params_.trunk_x_offset_p_coef_turn * fabs(support_to_next_.getOrigin().z()),
      params_.trunk_y_offset,
      0);
  tf2::Vector3 trunk_point_next(
      support_to_next_.getOrigin().x() + params_.trunk_x_offset
          + params_.trunk_x_offset_p_coef_forward * support_to_next_.getOrigin().x()
          + params_.trunk_x_offset_p_coef_turn * fabs(getNextEuler().z()),
      support_to_next_.getOrigin().y() + params_.trunk_y_offset,
      0);
  //Trunk middle neutral (no swing) position
  tf2::Vector3 trunk_point_middle =
      0.5 * trunk_point_support + 0.5 * trunk_point_next;
  //Trunk vector from middle to support apex
  tf2::Vector3 trunk_vect =
      trunk_point_support - trunk_point_middle;
  //Apply swing amplitude ratio
  trunk_vect[1] *= params_.trunkSwing;
  //Trunk support and next apex position
  tf2::Vector3 trunk_apex_support =
      trunk_point_middle + trunk_vect;
  tf2::Vector3 trunk_apex_next =
      trunk_point_middle - trunk_vect;
  //Trunk forward velocity
  double trunk_vel_support =
      (support_to_next_.getOrigin().x() - support_to_last_.getOrigin().x()) / period;
  double trunk_vel_next =
      support_to_next_.getOrigin().x() / half_period;

  //Trunk position
  if (start_step) {
    trunk_spline_.x()->addPoint(0.0, 0.0, 0.0, 0.0);
  } else {
    trunk_spline_.x()->addPoint(0.0,
                                trunk_pos_at_last_.x(),
                                trunk_pos_vel_at_last_.x(),
                                trunk_pos_acc_at_last_.x());
    trunk_spline_.x()->addPoint(half_period + time_shift,
                                trunk_apex_support.x(),
                                trunk_vel_support);
  }
  trunk_spline_.x()->addPoint(period + time_shift,
                              trunk_apex_next.x(),
                              trunk_vel_next);

  trunk_spline_.y()->addPoint(0.0,
                              trunk_pos_at_last_.y(),
                              trunk_pos_vel_at_last_.y(),
                              trunk_pos_acc_at_last_.y());
  if (start_step || start_movement) {
    trunk_spline_.y()->addPoint(half_period + time_shift - pause_length,
                                trunk_point_middle.y() + trunk_vect.y() * params_.first_step_swing_factor);
    trunk_spline_.y()->addPoint(half_period + time_shift + pause_length,
                                trunk_point_middle.y() + trunk_vect.y() * params_.first_step_swing_factor);
    trunk_spline_.y()->addPoint(period + time_shift - pause_length,
                                trunk_point_middle.y() - trunk_vect.y() * params_.first_step_swing_factor);
    trunk_spline_.y()->addPoint(period + time_shift + pause_length,
                                trunk_point_middle.y() - trunk_vect.y() * params_.first_step_swing_factor);
  } else {
    trunk_spline_.y()->addPoint(half_period + time_shift - pause_length,
                                trunk_apex_support.y());
    trunk_spline_.y()->addPoint(half_period + time_shift + pause_length,
                                trunk_apex_support.y());
    trunk_spline_.y()->addPoint(period + time_shift - pause_length,
                                trunk_apex_next.y());
    trunk_spline_.y()->addPoint(period + time_shift + pause_length,
                                trunk_apex_next.y());
  }

  trunk_spline_.z()->addPoint(0.0,
                              trunk_pos_at_last_.z(),
                              trunk_pos_vel_at_last_.z(),
                              trunk_pos_acc_at_last_.z());
  trunk_spline_.z()->addPoint(half_period + time_shift,
                              params_.trunk_height);
  trunk_spline_.z()->addPoint(period + time_shift,
                              params_.trunk_height);

  //Define trunk rotation as rool pitch yaw
  tf2::Vector3 euler_at_support = tf2::Vector3(
      0.0,
      params_.trunk_pitch
          + params_.trunk_pitch_p_coef_forward * support_to_next_.getOrigin().x()
          + params_.trunk_pitch_p_coef_turn * fabs(getNextEuler().z()),
      0.5 * getLastEuler().z() + 0.5 * getNextEuler().z());
  tf2::Vector3 euler_at_next = tf2::Vector3(
      0.0,
      params_.trunk_pitch
          + params_.trunk_pitch_p_coef_forward * support_to_next_.getOrigin().x()
          + params_.trunk_pitch_p_coef_turn * fabs(getNextEuler().z()),
      getNextEuler().z());

  // we set a velocity for the points in yaw since we want to keep the speed in turning direction for next step
  // in roll and pitch, no velocity is set since changes are only minor when speed changes
  tf2::Vector3 axis_vel(
      0.0, 0.0,
      bitbots_quintic_walk::AngleDistance(
          getLastEuler().z(),
          getNextEuler().z()) / period);

  //Trunk orientation
  trunk_spline_.roll()->addPoint(0.0,
                                 trunk_axis_pos_at_last_.x(),
                                 trunk_axis_vel_at_last_.x(),
                                 trunk_axis_acc_at_last_.x());
  trunk_spline_.roll()->addPoint(half_period + time_shift,
                                 euler_at_support.x(),
                                 axis_vel.x());
  trunk_spline_.roll()->addPoint(period + time_shift,
                                 euler_at_next.x(),
                                 axis_vel.x());

  trunk_spline_.pitch()->addPoint(0.0,
                                  trunk_axis_pos_at_last_.y(),
                                  trunk_axis_vel_at_last_.y(),
                                  trunk_axis_acc_at_last_.y());
  trunk_spline_.pitch()->addPoint(half_period + time_shift,
                                  euler_at_support.y(),
                                  axis_vel.y());
  trunk_spline_.pitch()->addPoint(period + time_shift,
                                  euler_at_next.y(),
                                  axis_vel.y());

  trunk_spline_.yaw()->addPoint(0.0,
                                trunk_axis_pos_at_last_.z(),
                                trunk_axis_vel_at_last_.z(),
                                trunk_axis_acc_at_last_.z());
  trunk_spline_.yaw()->addPoint(half_period + time_shift,
                                euler_at_support.z(),
                                axis_vel.z());
  trunk_spline_.yaw()->addPoint(period + time_shift,
                                euler_at_next.z(),
                                axis_vel.z());
}

void WalkEngine::buildWalkDisableTrajectories(bool foot_in_idle_position) {
  // save the current trunk state to use it later
  saveCurrentTrunkState();
  // update support foot and compute odometry
  stepFromOrders(request_.orders);

  //Reset the trajectories
  is_double_support_spline_ = bitbots_splines::SmoothSpline();
  is_left_support_foot_spline_ = bitbots_splines::SmoothSpline();
  trunk_spline_ = bitbots_splines::PoseSpline();
  foot_spline_ = bitbots_splines::PoseSpline();

  //Set up the trajectories
  //for the half cycle
  double half_period = 1.0 / (2.0 * params_.freq);

  //Time length of double and single
  //support phase during the half cycle
  double double_support_length = params_.double_support_ratio * half_period;
  double single_support_length = half_period - double_support_length;

  //Sign of support foot with
  //respect to lateral
  double support_sign = (is_left_support_foot_ ? 1.0 : -1.0);

  //Set double support phase
  double is_double_support = (foot_in_idle_position ? 1.0 : 0.0);
  is_double_support_spline_.addPoint(0.0, is_double_support);
  is_double_support_spline_.addPoint(half_period, is_double_support);

  //Set support foot
  is_left_support_foot_spline_.addPoint(0.0, is_left_support_foot_);
  is_left_support_foot_spline_.addPoint(half_period, is_left_support_foot_);

  //Flying foot position
  foot_spline_.x()->addPoint(0.0,
                             support_to_last_.getOrigin().x());
  foot_spline_.x()->addPoint(double_support_length,
                             support_to_last_.getOrigin().x());
  foot_spline_.x()->addPoint(
      double_support_length + single_support_length * params_.foot_put_down_phase * params_.foot_overshoot_phase,
      0.0 + (0.0 - support_to_last_.getOrigin().x()) * params_.foot_overshoot_ratio);
  foot_spline_.x()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             0.0);
  foot_spline_.x()->addPoint(half_period,
                             0.0);

  foot_spline_.y()->addPoint(0.0,
                             support_to_last_.getOrigin().y());
  foot_spline_.y()->addPoint(double_support_length,
                             support_to_last_.getOrigin().y());
  foot_spline_.y()->addPoint(
      double_support_length + single_support_length * params_.foot_put_down_phase * params_.foot_overshoot_phase,
      -support_sign * params_.foot_distance
          + (-support_sign * params_.foot_distance - support_to_last_.getOrigin().y())
              * params_.foot_overshoot_ratio);
  foot_spline_.y()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             -support_sign * params_.foot_distance);
  foot_spline_.y()->addPoint(half_period,
                             -support_sign * params_.foot_distance);

  //If the walk has just been disabled,
  //make one single step to neutral pose
  if (!foot_in_idle_position) {
    foot_spline_.z()->addPoint(0.0, 0.0);
    foot_spline_.z()->addPoint(double_support_length, 0.0);
    foot_spline_.z()->addPoint(
        double_support_length + single_support_length * params_.foot_apex_phase
            - 0.5 * params_.foot_z_pause * single_support_length,
        params_.foot_rise);
    foot_spline_.z()->addPoint(
        double_support_length + single_support_length * params_.foot_apex_phase
            + 0.5 * params_.foot_z_pause * single_support_length,
        params_.foot_rise);
    foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                               params_.foot_put_down_z_offset);
    foot_spline_.z()->addPoint(half_period,
                               0.0);
  } else {
    //dont move the foot in last single step before stop since we only move the trunk back to the center
    foot_spline_.z()->addPoint(0.0, 0.0);
    foot_spline_.z()->addPoint(half_period, 0.0);
  }
  //Flying foot orientation
  foot_spline_.roll()->addPoint(0.0, 0.0);
  foot_spline_.roll()->addPoint(half_period, 0.0);

  foot_spline_.pitch()->addPoint(0.0, 0.0);
  foot_spline_.pitch()->addPoint(half_period, 0.0);

  foot_spline_.yaw()->addPoint(0.0, getLastEuler().z());
  foot_spline_.yaw()->addPoint(double_support_length,
                               getLastEuler().z());
  foot_spline_.yaw()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                               0.0);
  foot_spline_.yaw()->addPoint(half_period, 0.0);

  //Trunk position
  trunk_spline_.x()->addPoint(0.0,
                              trunk_pos_at_last_.x(),
                              trunk_pos_vel_at_last_.x(),
                              trunk_pos_acc_at_last_.x());
  trunk_spline_.x()->addPoint(half_period,
                              params_.trunk_x_offset);

  trunk_spline_.y()->addPoint(0.0,
                              trunk_pos_at_last_.y(),
                              trunk_pos_vel_at_last_.y(),
                              trunk_pos_acc_at_last_.y());
  trunk_spline_.y()->addPoint(half_period,
                              -support_sign * 0.5 * params_.foot_distance + params_.trunk_y_offset);

  trunk_spline_.z()->addPoint(0.0,
                              trunk_pos_at_last_.z(),
                              trunk_pos_vel_at_last_.z(),
                              trunk_pos_acc_at_last_.z());
  trunk_spline_.z()->addPoint(half_period,
                              params_.trunk_height);

  //Trunk orientation
  trunk_spline_.roll()->addPoint(0.0,
                                 trunk_axis_pos_at_last_.x(),
                                 trunk_axis_vel_at_last_.x(),
                                 trunk_axis_acc_at_last_.x());
  trunk_spline_.roll()->addPoint(half_period, 0.0);
  trunk_spline_.pitch()->addPoint(0.0,
                                  trunk_axis_pos_at_last_.y(),
                                  trunk_axis_vel_at_last_.y(),
                                  trunk_axis_acc_at_last_.y());
  trunk_spline_.pitch()->addPoint(half_period,
                                  params_.trunk_pitch);
  trunk_spline_.yaw()->addPoint(0.0,
                                trunk_axis_pos_at_last_.z(),
                                trunk_axis_vel_at_last_.z(),
                                trunk_axis_acc_at_last_.z());
  trunk_spline_.yaw()->addPoint(half_period,
                                0.0);
}

WalkResponse WalkEngine::createResponse() {
  //Evaluate target cartesian state from trajectories at current trajectory time
  double time = getTrajsTime();
  WalkResponse response;
  response.is_double_support = is_double_support_spline_.pos(time) >= 0.5;
  response.is_left_support_foot = is_left_support_foot_spline_.pos(time) >= 0.5;
  response.support_foot_to_flying_foot = foot_spline_.getTfTransform(time);
  response.support_foot_to_trunk = trunk_spline_.getTfTransform(time);

  //add additional information to response, mainly for debug purposes
  response.phase = phase_;
  response.traj_time = getTrajsTime();
  response.foot_distance = params_.foot_distance;
  response.state = engine_state_;
  response.support_to_last = support_to_last_;
  response.support_to_next = support_to_next_;

  return response;
}

double WalkEngine::getPhase() const {
  return phase_;
}

double WalkEngine::getTrajsTime() const {
  double t;
  if (phase_ < 0.5) {
    t = phase_ / params_.freq;
  } else {
    t = (phase_ - 0.5) / params_.freq;
  }

  return t;
}

bool WalkEngine::isLeftSupport() {
  return is_left_support_foot_;
}

bool WalkEngine::isDoubleSupport() {
  // returns true if the value of the "is_double_support" spline is currently higher than 0.5
  // the spline should only have values of 0 or 1
  return is_double_support_spline_.pos(getTrajsTime()) >= 0.5;
}

void WalkEngine::reconfCallback(bitbots_quintic_walk_engine_paramsConfig &params, uint32_t level) {
  params_ = params;
}

void WalkEngine::requestKick(bool left) {
  if (left) {
    left_kick_requested_ = true;
  } else {
    right_kick_requested_ = true;
  }
}

void WalkEngine::requestPause() {
  pause_requested_ = true;
}

WalkState WalkEngine::getState() {
  return engine_state_;
}

int WalkEngine::getPercentDone() const {
  return (int) (getTrajsTime() * 100);
}

void WalkEngine::setPauseDuration(double duration) {
  pause_duration_ = duration;
}

double WalkEngine::getFreq() {
  return params_.freq;
}

double WalkEngine::getWantedTrunkPitch() {
  return params_.trunk_pitch + params_.trunk_pitch_p_coef_forward * support_to_next_.getOrigin().x()
      + params_.trunk_pitch_p_coef_turn * fabs(support_to_next_.getOrigin().z());
}

void WalkEngine::stepFromSupport(const tf2::Transform &diff) {
  //Update relative diff from support foot
  support_to_last_ = support_to_next_.inverse();
  support_to_next_ = diff;
  //Update world integrated position
  if (is_left_support_foot_) {
    left_in_world_ = right_in_world_ * diff;
  } else {
    right_in_world_ = left_in_world_ * diff;
  }
  //Update current support foot
  is_left_support_foot_ = !is_left_support_foot_;
}

void WalkEngine::stepFromOrders(const tf2::Vector3 &orders) {
  //Compute step diff in next support foot frame
  tf2::Transform tmp_diff = tf2::Transform();
  tmp_diff.setIdentity();
  //No change in forward step
  tmp_diff.getOrigin()[0] = orders.x();
  //Add lateral foot offset
  if (is_left_support_foot_) {
    tmp_diff.getOrigin()[1] += params_.foot_distance;
  } else {
    tmp_diff.getOrigin()[1] -= params_.foot_distance;
  }
  //Allow lateral step only on external foot
  //(internal foot will return to zero pose)
  if (
      (is_left_support_foot_ && orders.y() > 0.0) ||
          (!is_left_support_foot_ && orders.y() < 0.0)
      ) {
    tmp_diff.getOrigin()[1] += orders.y();
  }
  //No change in turn (in order to
  //rotate around trunk center)
  tf2::Quaternion quat;
  quat.setRPY(0, 0, orders.z());
  tmp_diff.setRotation(quat);

  //Make the step
  stepFromSupport(tmp_diff);
}

tf2::Vector3 WalkEngine::getNextEuler() {
  double roll, pitch, yaw;
  tf2::Matrix3x3(support_to_next_.getRotation()).getRPY(roll, pitch, yaw);
  return tf2::Vector3(roll, pitch, yaw);
}

tf2::Vector3 WalkEngine::getLastEuler() {
  double roll, pitch, yaw;
  tf2::Matrix3x3(support_to_last_.getRotation()).getRPY(roll, pitch, yaw);
  return tf2::Vector3(roll, pitch, yaw);
}

tf2::Transform WalkEngine::getLeft() {
  return left_in_world_;
}

tf2::Transform WalkEngine::getRight() {
  return right_in_world_;
}

}

