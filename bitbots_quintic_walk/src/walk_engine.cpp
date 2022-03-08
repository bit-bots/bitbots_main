/*
node_ code is partly based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/walk_engine.h"

namespace bitbots_quintic_walk {

WalkEngine::WalkEngine(rclcpp::Node::SharedPtr node) :
    params_(),
    node_(node),
    engine_state_(WalkState::IDLE),
    phase_(0.0),
    last_phase_(0.0),
    time_paused_(0.0),
    pause_duration_(0.0),
    pause_requested_(false),
    phase_rest_active_(false),
    left_kick_requested_(false),
    right_kick_requested_(false),
    is_left_support_foot_(false),
    foot_pos_vel_at_foot_change_({0.0, 0.0, 0.0}),
    foot_pos_acc_at_foot_change_({0.0, 0.0, 0.0}),
    foot_orientation_pos_at_last_foot_change_({0, 0, 0}),
    foot_orientation_vel_at_last_foot_change_({0, 0, 0}),
    foot_orientation_acc_at_foot_change_({0, 0, 0}) {
  left_in_world_.setIdentity();
  right_in_world_.setIdentity();
  reset();

  node_->get_parameter("engine.freq", params_.freq);
  node_->get_parameter("engine.double_support_ratio", params_.double_support_ratio);
  node_->get_parameter("engine.foot_distance", params_.foot_distance);
  node_->get_parameter("engine.foot_rise", params_.foot_rise);
  node_->get_parameter("engine.trunk_swing", params_.trunk_swing);
  node_->get_parameter("engine.trunk_height", params_.trunk_height);
  node_->get_parameter("engine.trunk_pitch", params_.trunk_pitch);
  node_->get_parameter("engine.trunk_pitch_p_coef_forward", params_.trunk_pitch_p_coef_forward);
  node_->get_parameter("engine.trunk_phase", params_.trunk_phase);
  node_->get_parameter("engine.foot_z_pause", params_.foot_z_pause);
  node_->get_parameter("engine.foot_put_down_z_offset", params_.foot_put_down_z_offset);
  node_->get_parameter("engine.foot_put_down_phase", params_.foot_put_down_phase);
  node_->get_parameter("engine.foot_apex_phase", params_.foot_apex_phase);
  node_->get_parameter("engine.foot_overshoot_ratio", params_.foot_overshoot_ratio);
  node_->get_parameter("engine.foot_overshoot_phase", params_.foot_overshoot_phase);
  node_->get_parameter("engine.trunk_x_offset", params_.trunk_x_offset);
  node_->get_parameter("engine.trunk_y_offset", params_.trunk_y_offset);
  node_->get_parameter("engine.trunk_pause", params_.trunk_pause);
  node_->get_parameter("engine.trunk_x_offset_p_coef_forward", params_.trunk_x_offset_p_coef_forward);
  node_->get_parameter("engine.trunk_x_offset_p_coef_turn", params_.trunk_x_offset_p_coef_turn);
  node_->get_parameter("engine.trunk_pitch_p_coef_turn", params_.trunk_pitch_p_coef_turn);
  node_->get_parameter("engine.kick_length", params_.kick_length);
  node_->get_parameter("engine.kick_vel", params_.kick_vel);
  node_->get_parameter("engine.kick_phase", params_.kick_phase);
  node_->get_parameter("engine.foot_put_down_roll_offset", params_.foot_put_down_roll_offset);
  node_->get_parameter("engine.first_step_swing_factor", params_.first_step_swing_factor);
  node_->get_parameter("engine.first_step_trunk_phase", params_.first_step_trunk_phase);
  node_->get_parameter("engine.trunk_z_movement", params_.trunk_z_movement);

  // move left and right in world by foot distance for correct initialization
  left_in_world_.setOrigin(tf2::Vector3{0, params_.foot_distance / 2, 0});
  right_in_world_.setOrigin(tf2::Vector3{0, -1 * params_.foot_distance / 2, 0});
  // create splines one time to have no empty splines during first idle phase
  buildStartMovementTrajectories();

  if (params_.foot_distance == 0) {
    RCLCPP_WARN(node_->get_logger(), "Parameters are probably not loaded correctly (unless you are running optimization)");
  }

}

void WalkEngine::setGoals(const WalkRequest &goals) {
  request_ = goals;
}

WalkResponse WalkEngine::update(double dt) {
  // First check cases where we do not want to update the phase: pausing, idle and phase rest
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
    if ((request_.stop_walk && !request_.single_step) || !request_.walkable_state) {
      // we are in idle and are not supposed to walk. current state is fine, just do nothing
      return createResponse();
    }
  } else if (engine_state_ == WalkState::WALKING) {
    // check if the step would finish with node_ update of the phase
    bool step_will_finish = (phase_ < 0.5 && phase_ + dt * params_.freq > 0.5) || phase_ + dt * params_.freq > 1.0;
    // check if we should rest the phase because the flying foot didn't make contact to the ground during step
    if (step_will_finish && phase_rest_active_) {
      // dont update the phase (do a phase rest) till it gets updated by a phase reset
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1, "PHASE REST");
      return createResponse();
    }
  }

  if (engine_state_ == WalkState::IDLE && request_.single_step) {
    float start_phase;
    if (request_.linear_orders[1] < 0) {
      start_phase = 0.5;
    } else {
      start_phase = 0.0;
    }
    // when we want to perform a single step to the right, we need to force using the correct leg
    reset(WalkState::IDLE,
          start_phase,
          {request_.linear_orders[0], request_.linear_orders[1], request_.linear_orders[2], request_.angular_z},
          false,
          true,
          false);
    request_.stop_walk = true;
  }

  // update the current phase
  updatePhase(dt);

  // check if we will finish a half step with node_ update
  bool half_step_finished = (last_phase_ < 0.5 && phase_ >= 0.5) || (last_phase_ > 0.5 && phase_ < 0.5);

  // small state machine
  if (engine_state_ == WalkState::IDLE) {
    // state is idle and orders are not zero, we can start walking
    buildStartMovementTrajectories();
    engine_state_ = WalkState::START_MOVEMENT;
  } else if (engine_state_ == WalkState::START_MOVEMENT) {
    // in node_ state we do a single "step" where we only move the trunk
    if (half_step_finished) {
      if (request_.stop_walk && !request_.single_step) {
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
      if (request_.single_step) {
        request_.single_step = false;
        engine_state_ = WalkState::STOP_STEP;
        buildStopStepTrajectories();
      } else if (request_.stop_walk) {
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
      if (request_.single_step) {
        request_.single_step = false;
        engine_state_ = WalkState::STOP_STEP;
        buildStopStepTrajectories();
      } else if (request_.stop_walk) {
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
    // in node_ state we do a kick while doing a step
    if (half_step_finished) {
      //kick step is finished, go on walking
      engine_state_ = WalkState::WALKING;
      buildNormalTrajectories();
    }
  } else if (engine_state_ == WalkState::STOP_STEP) {
    // in node_ state we do a step back to get feet into idle pose
    if (half_step_finished) {
      //stop step is finished, go to stop movement state
      engine_state_ = WalkState::STOP_MOVEMENT;
      buildStopMovementTrajectories();
    }
  } else if (engine_state_ == WalkState::STOP_MOVEMENT) {
    // in node_ state we do a "step" where we move the trunk back to idle position
    if (half_step_finished) {
      //stop movement is finished, go to idle state
      engine_state_ = WalkState::IDLE;
      return createResponse();
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Something is wrong with the walking engine state");
  }

  //Sanity check support foot state
  if ((phase_ < 0.5 && !is_left_support_foot_) ||
      (phase_ >= 0.5 && is_left_support_foot_)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1,
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
      RCLCPP_ERROR_THROTTLE(node_->get_logger(),
                            *node_->get_clock(),
                            1,
                            "WalkEngine exception negative dt phase= %f dt= %f",
                            phase_,
                            dt);
      return;
    }
  }
  //Check for too long dt
  if (dt > 0.5001 / params_.freq) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(),
                          *node_->get_clock(),
                          1,
                          "WalkEngine error too long dt phase= %f dt= %f",
                          phase_,
                          dt);
    return;
  }

  //Update the phase
  phase_ += dt * params_.freq;

  // reset if step complete
  if (phase_ > 1.0) {
    phase_ = phase_ - 1;
  }
}

void WalkEngine::endStep() {
  // ends the step earlier, e.g. when foot has already contact to ground
  // last_phase will still held the information about the time point where the reset occurred
  if (phase_ < 0.5) {
    phase_ = 0.5;
  } else {
    phase_ = 0.0;
  }
}

void WalkEngine::setPhaseRest(bool active) {
  phase_rest_active_ = active;
}

void WalkEngine::reset() {
  reset(WalkState::IDLE, 0.0, {0, 0, 0, 0}, true, false, false);
}

void WalkEngine::reset(WalkState state,
                       double phase,
                       std::vector<double> step,
                       bool stop_walk,
                       bool walkable_state,
                       bool reset_odometry) {
  request_.linear_orders[0] = step[0];
  request_.linear_orders[1] = step[1];
  request_.linear_orders[2] = step[2];
  request_.angular_z = step[3];
  request_.stop_walk = stop_walk;
  request_.walkable_state = walkable_state;
  engine_state_ = state;
  time_paused_ = 0.0;
  pause_requested_ = false;

  if (state == WalkState::IDLE) {
    // we dont need to build trajectories in idle, just reset everything
    if (phase < 0.5) {
      is_left_support_foot_ = false;
      last_phase_ = 0.49999;
    } else {
      is_left_support_foot_ = true;
      last_phase_ = 1.0;
    }
    phase_ = phase;

    support_to_last_.setIdentity();
    support_to_next_.setIdentity();
    if (is_left_support_foot_) {
      support_to_next_.getOrigin()[1] = -params_.foot_distance;
    } else {
      support_to_next_.getOrigin()[1] = params_.foot_distance;
    }

    //Reset the trunk saved state
    if (is_left_support_foot_) {
      trunk_pos_at_foot_change_ = tf2::Vector3(
          params_.trunk_x_offset,
          -params_.foot_distance / 2.0 + params_.trunk_y_offset,
          params_.trunk_height);
    } else {
      trunk_pos_at_foot_change_ = tf2::Vector3(
          params_.trunk_x_offset,
          params_.foot_distance / 2.0 + params_.trunk_y_offset,
          params_.trunk_height);
    }

    trunk_pos_vel_at_foot_change_.setZero();
    trunk_pos_acc_at_foot_change_.setZero();
    trunk_orientation_pos_at_last_foot_change_ = tf2::Vector3(0.0, params_.trunk_pitch, 0.0);
    trunk_orientation_vel_at_last_foot_change_.setZero();
    trunk_orientation_acc_at_foot_change_.setZero();

  } else {

    if (phase >= 0.5) {
      is_left_support_foot_ = false;
      last_phase_ = 0.49999;
    } else {
      is_left_support_foot_ = true;
      last_phase_ = 1.0;
    }
    phase_ = phase;

    // build trajectories for node_ state once to get correct start point for new trajectory
    if (state == WalkState::WALKING) {
      buildNormalTrajectories();
    } else if (state == WalkState::START_MOVEMENT) {
      buildStartMovementTrajectories();
    } else if (state == WalkState::START_STEP) {
      buildStartStepTrajectories();
    } else if (state == WalkState::STOP_MOVEMENT) {
      buildStopMovementTrajectories();
    } else if (state == WalkState::START_STEP) {
      buildStopStepTrajectories();
    } else if (state == WalkState::KICK) {
      buildKickTrajectories();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "walk reset state not known");
    }

    // now switch them again
    if (phase >= 0.5) {
      is_left_support_foot_ = true;
      last_phase_ = 1.0;
    } else {
      is_left_support_foot_ = false;
      last_phase_ = 0.49999;
    }

    if (reset_odometry) {
      // move left and right in world by foot distance for correct initialization
      left_in_world_.setIdentity();
      right_in_world_.setIdentity();
      left_in_world_.setOrigin(tf2::Vector3{0, params_.foot_distance / 2, 0});
      right_in_world_.setOrigin(tf2::Vector3{0, -1 * params_.foot_distance / 2, 0});
    }

    // build trajectories one more time with end state of previously build trajectories as a start
    if (state == WalkState::WALKING) {
      buildNormalTrajectories();
    } else if (state == WalkState::START_MOVEMENT) {
      buildStartMovementTrajectories();
    } else if (state == WalkState::START_STEP) {
      buildStartStepTrajectories();
    } else if (state == WalkState::STOP_MOVEMENT) {
      buildStopMovementTrajectories();
    } else if (state == WalkState::START_STEP) {
      buildStopStepTrajectories();
    } else if (state == WalkState::KICK) {
      buildKickTrajectories();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "walk reset state not known");
    }
  }
  last_phase_ = phase_;
}

void WalkEngine::saveCurrentRobotState() {
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

  // get the current transform to the next foot instead of the planned one (in case of phase reset)
  tf2::Transform current_support_to_next_ = foot_spline_.getTfTransform(period_time);
  // we use a transformation with a 0 origin, since we only want to do rotation
  tf2::Transform rotation_to_next(current_support_to_next_);
  rotation_to_next.setOrigin({0, 0, 0});

  // get last values of trunk pose and its velocities and accelerations
  tf2::Transform trunk_pose = trunk_spline_.getTfTransform(period_time);
  tf2::Vector3 trunk_pos_vel = trunk_spline_.getPositionVel(period_time);
  tf2::Vector3 trunk_pos_acc = trunk_spline_.getPositionAcc(period_time);
  tf2::Vector3 trunk_axis_vel = trunk_spline_.getEulerVel(period_time);
  tf2::Vector3 trunk_axis_acc = trunk_spline_.getEulerAcc(period_time);

  // Convert the pose in next support foot frame and save
  tf2::Transform trunk_pose_at_last = current_support_to_next_.inverse() * trunk_pose;
  trunk_pos_at_foot_change_ = trunk_pose_at_last.getOrigin();
  double roll, pitch, yaw;
  tf2::Matrix3x3(trunk_pose_at_last.getRotation()).getRPY(roll, pitch, yaw);
  trunk_orientation_pos_at_last_foot_change_ = tf2::Vector3(roll, pitch, yaw);

  // convert the velocities and accelerations in next support foot frame and save
  trunk_pos_vel_at_foot_change_ = rotation_to_next * trunk_pos_vel;
  trunk_pos_acc_at_foot_change_ = rotation_to_next * trunk_pos_acc;
  trunk_orientation_vel_at_last_foot_change_ = rotation_to_next * trunk_axis_vel;
  trunk_orientation_acc_at_foot_change_ = rotation_to_next * trunk_axis_acc;

  // get last values of foot pose and velocities and accelerations
  tf2::Transform foot_pose = foot_spline_.getTfTransform(period_time);
  tf2::Vector3 foot_pos_vel = foot_spline_.getPositionVel(period_time);
  tf2::Vector3 foot_pos_acc = foot_spline_.getPositionAcc(period_time);
  tf2::Vector3 foot_axis_vel = foot_spline_.getEulerVel(period_time);
  tf2::Vector3 foot_axis_acc = foot_spline_.getEulerAcc(period_time);

  // Convert the pose in next support foot frame and save
  tf2::Transform foot_pose_at_last = current_support_to_next_.inverse();
  foot_pos_at_foot_change_ = foot_pose_at_last.getOrigin();
  tf2::Matrix3x3(foot_pose_at_last.getRotation()).getRPY(roll, pitch, yaw);
  foot_orientation_pos_at_last_foot_change_ = tf2::Vector3(roll, pitch, yaw);

  // convert the velocities and accelerations in next support foot frame and save
  foot_pos_vel_at_foot_change_ = rotation_to_next * foot_pos_vel;
  foot_pos_acc_at_foot_change_ = rotation_to_next * foot_pos_acc;
  foot_orientation_vel_at_last_foot_change_ = rotation_to_next * foot_axis_vel;
  foot_orientation_acc_at_foot_change_ = rotation_to_next * foot_axis_acc;
}

void WalkEngine::buildNormalTrajectories() {
  buildTrajectories(false, false, false, false);
}

void WalkEngine::buildKickTrajectories() {
  buildTrajectories(false, false, true, false);
}

void WalkEngine::buildStartMovementTrajectories() {
  buildTrajectories(true, false, false, false);
}

void WalkEngine::buildStartStepTrajectories() {
  buildTrajectories(false, true, false, false);
}

void WalkEngine::buildStopStepTrajectories() {
  buildTrajectories(false, false, false, true);
}

void WalkEngine::buildStopMovementTrajectories() {
  buildWalkDisableTrajectories(true);
}

void WalkEngine::buildTrajectories(bool start_movement, bool start_step, bool kick_step, bool stop_step) {
  // save the current trunk state to use it later and compute the next step position
  if (!start_movement) {
    saveCurrentRobotState();
    if (!stop_step) {
      stepFromOrders(request_.linear_orders, request_.angular_z);
    } else {
      stepFromOrders({0, 0, 0}, 0);
    }
  } else {
    // reset all foot change parameters
    // when we do start step, only transform the y coordinate since we stand still and only move trunk sideward
    trunk_pos_at_foot_change_ =
        {0.0, trunk_pos_at_foot_change_.y() - support_to_next_.getOrigin().y(), params_.trunk_height};
    trunk_pos_vel_at_foot_change_.setZero();
    trunk_pos_acc_at_foot_change_.setZero();
    trunk_orientation_pos_at_last_foot_change_ = {0.0, params_.trunk_pitch, 0.0};
    trunk_orientation_vel_at_last_foot_change_.setZero();
    trunk_orientation_acc_at_foot_change_.setZero();
    foot_pos_at_foot_change_ = {0.0, support_to_next_.getOrigin().y() * -1, 0.0};
    foot_pos_vel_at_foot_change_.setZero();
    foot_pos_acc_at_foot_change_.setZero();
    foot_orientation_pos_at_last_foot_change_.setZero();
    foot_orientation_vel_at_last_foot_change_.setZero();
    foot_orientation_acc_at_foot_change_.setZero();
    support_to_last_.setIdentity();
    support_to_next_.setIdentity();
    if (is_left_support_foot_) {
      support_to_next_.getOrigin()[1] = -params_.foot_distance;
    } else {
      support_to_next_.getOrigin()[1] = params_.foot_distance;
    }
    stepFromOrders({0, 0, 0}, 0);
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
  double trunk_phase;
  if (start_movement || start_step) {
    trunk_phase = params_.first_step_trunk_phase;
  } else {
    trunk_phase = params_.trunk_phase;
  }

  double time_shift = -0.5 * half_period + 0.5 * double_support_length + trunk_phase * half_period;


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
  foot_spline_.x()->addPoint(0.0, foot_pos_at_foot_change_.x(),
                             foot_pos_vel_at_foot_change_.x(),
                             foot_pos_acc_at_foot_change_.x());
  foot_spline_.x()->addPoint(double_support_length, foot_pos_at_foot_change_.x());
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

  foot_spline_.y()->addPoint(0.0, foot_pos_at_foot_change_.y(),
                             foot_pos_vel_at_foot_change_.y(),
                             foot_pos_acc_at_foot_change_.y());
  foot_spline_.y()->addPoint(double_support_length, foot_pos_at_foot_change_.y());
  foot_spline_.y()->addPoint(
      double_support_length + single_support_length * params_.foot_put_down_phase * params_.foot_overshoot_phase,
      support_to_next_.getOrigin().y()
          + (support_to_next_.getOrigin().y() - support_to_last_.getOrigin().y()) * params_.foot_overshoot_ratio);
  foot_spline_.y()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             support_to_next_.getOrigin().y());
  foot_spline_.y()->addPoint(half_period, support_to_next_.getOrigin().y());

  foot_spline_.z()->addPoint(0.0, foot_pos_at_foot_change_.z(),
                             foot_pos_vel_at_foot_change_.z(),
                             foot_pos_acc_at_foot_change_.z());
  foot_spline_.z()->addPoint(double_support_length, foot_pos_at_foot_change_.z());
  foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_apex_phase
                                 - 0.5 * params_.foot_z_pause * single_support_length,
                             params_.foot_rise);
  foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_apex_phase
                                 + 0.5 * params_.foot_z_pause * single_support_length,
                             params_.foot_rise);
  foot_spline_.z()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             params_.foot_put_down_z_offset + support_to_next_.getOrigin().z());
  foot_spline_.z()->addPoint(half_period, support_to_next_.getOrigin().z());

  //Flying foot orientation
  foot_spline_.roll()->addPoint(0.0, foot_orientation_pos_at_last_foot_change_.x(),
                                foot_orientation_vel_at_last_foot_change_.x(),
                                foot_orientation_acc_at_foot_change_.x());
  foot_spline_.roll()->addPoint(double_support_length + 0.1 * single_support_length, 0.0);
  foot_spline_.roll()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                                params_.foot_put_down_roll_offset * support_sign);
  foot_spline_.roll()->addPoint(half_period, 0.0);

  foot_spline_.pitch()->addPoint(0.0, foot_orientation_pos_at_last_foot_change_.y(),
                                 foot_orientation_vel_at_last_foot_change_.y(),
                                 foot_orientation_acc_at_foot_change_.y());
  foot_spline_.pitch()->addPoint(half_period, 0.0);

  foot_spline_.yaw()->addPoint(0.0, foot_orientation_pos_at_last_foot_change_.z(),
                               foot_orientation_vel_at_last_foot_change_.z(),
                               foot_orientation_acc_at_foot_change_.z());
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
  trunk_vect[1] *= params_.trunk_swing;
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
  if (start_movement || start_step) {
    trunk_spline_.x()->addPoint(0.0, 0.0, 0.0, 0.0);
  } else {
    trunk_spline_.x()->addPoint(0.0,
                                trunk_pos_at_foot_change_.x(),
                                trunk_pos_vel_at_foot_change_.x(),
                                trunk_pos_acc_at_foot_change_.x());
    trunk_spline_.x()->addPoint(half_period + time_shift,
                                trunk_apex_support.x(),
                                trunk_vel_support);
  }
  trunk_spline_.x()->addPoint(period + time_shift,
                              trunk_apex_next.x(),
                              trunk_vel_next);

  if (start_movement) {
    trunk_spline_.y()->addPoint(0.0, trunk_point_middle.y(), 0.0, 0.0);
  } else {
    trunk_spline_.y()->addPoint(0.0,
                                trunk_pos_at_foot_change_.y(),
                                trunk_pos_vel_at_foot_change_.y(),
                                trunk_pos_acc_at_foot_change_.y());
  }
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

  // When walking downwards, the correct trunk height is the one relative to the flying
  // foot goal position, node_ results in lowering the trunk correctly during the step
  double
      trunk_height_including_foot_z_movement = params_.trunk_height + std::min(0.0, support_to_next_.getOrigin().z());
  // Periodic z movement of trunk is at lowest point at double support center, highest at single support center
  trunk_spline_.z()->addPoint(0.0,
                              trunk_pos_at_foot_change_.z(),
                              trunk_pos_vel_at_foot_change_.z(),
                              trunk_pos_acc_at_foot_change_.z());
  trunk_spline_.z()->addPoint(double_support_length / 2,
                              trunk_height_including_foot_z_movement);
  if (!start_movement) {
    trunk_spline_.z()->addPoint(double_support_length + single_support_length / 2,
                                trunk_height_including_foot_z_movement + params_.trunk_z_movement);
    trunk_spline_.z()->addPoint(half_period + double_support_length / 2,
                                trunk_height_including_foot_z_movement);
    trunk_spline_.z()->addPoint(half_period + double_support_length + single_support_length / 2,
                                trunk_height_including_foot_z_movement + params_.trunk_z_movement);
  }
  trunk_spline_.z()->addPoint(period + double_support_length / 2,
                              trunk_height_including_foot_z_movement);

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
      bitbots_quintic_walk::angleDistance(
          getLastEuler().z(),
          getNextEuler().z()) / period);

  //Trunk orientation
  trunk_spline_.roll()->addPoint(0.0,
                                 trunk_orientation_pos_at_last_foot_change_.x(),
                                 trunk_orientation_vel_at_last_foot_change_.x(),
                                 trunk_orientation_acc_at_foot_change_.x());
  trunk_spline_.roll()->addPoint(half_period + time_shift,
                                 euler_at_support.x(),
                                 axis_vel.x());
  trunk_spline_.roll()->addPoint(period + time_shift,
                                 euler_at_next.x(),
                                 axis_vel.x());

  trunk_spline_.pitch()->addPoint(0.0,
                                  trunk_orientation_pos_at_last_foot_change_.y(),
                                  trunk_orientation_vel_at_last_foot_change_.y(),
                                  trunk_orientation_acc_at_foot_change_.y());
  trunk_spline_.pitch()->addPoint(half_period + time_shift,
                                  euler_at_support.y(),
                                  axis_vel.y());
  trunk_spline_.pitch()->addPoint(period + time_shift,
                                  euler_at_next.y(),
                                  axis_vel.y());

  trunk_spline_.yaw()->addPoint(0.0,
                                trunk_orientation_pos_at_last_foot_change_.z(),
                                trunk_orientation_vel_at_last_foot_change_.z(),
                                trunk_orientation_acc_at_foot_change_.z());
  trunk_spline_.yaw()->addPoint(half_period + time_shift,
                                euler_at_support.z(),
                                axis_vel.z());
  trunk_spline_.yaw()->addPoint(period + time_shift,
                                euler_at_next.z(),
                                axis_vel.z());
}

void WalkEngine::buildWalkDisableTrajectories(bool foot_in_idle_position) {
  // save the current trunk state to use it later
  saveCurrentRobotState();
  // update support foot and compute odometry
  stepFromOrders(request_.linear_orders, request_.angular_z);

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
  is_double_support_spline_.addPoint(0.0, 1.0);
  if (foot_in_idle_position) {
    is_double_support_spline_.addPoint(half_period, 1.0);
  } else {
    is_double_support_spline_.addPoint(double_support_length, 1.0);
    is_double_support_spline_.addPoint(double_support_length, 0.0);
    is_double_support_spline_.addPoint(half_period, 0.0);
  }

  //Set support foot
  is_left_support_foot_spline_.addPoint(0.0, is_left_support_foot_);
  is_left_support_foot_spline_.addPoint(half_period, is_left_support_foot_);

  //Flying foot position
  foot_spline_.x()->addPoint(0.0,
                             foot_pos_at_foot_change_.x(),
                             foot_pos_vel_at_foot_change_.x(),
                             foot_pos_acc_at_foot_change_.x());
  foot_spline_.x()->addPoint(double_support_length,
                             foot_pos_at_foot_change_.x());
  foot_spline_.x()->addPoint(
      double_support_length + single_support_length * params_.foot_put_down_phase * params_.foot_overshoot_phase,
      0.0 + (0.0 - support_to_last_.getOrigin().x()) * params_.foot_overshoot_ratio);
  foot_spline_.x()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                             0.0);
  foot_spline_.x()->addPoint(half_period,
                             0.0);

  foot_spline_.y()->addPoint(0.0,
                             foot_pos_at_foot_change_.y(),
                             foot_pos_vel_at_foot_change_.y(),
                             foot_pos_acc_at_foot_change_.y());
  foot_spline_.y()->addPoint(double_support_length,
                             foot_pos_at_foot_change_.y());
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
    foot_spline_.z()->addPoint(0.0,
                               foot_pos_at_foot_change_.z());
    foot_spline_.z()->addPoint(half_period, 0.0);
  }
  //Flying foot orientation
  foot_spline_.roll()->addPoint(0.0, foot_orientation_pos_at_last_foot_change_.x(),
                                foot_orientation_vel_at_last_foot_change_.x(),
                                foot_orientation_acc_at_foot_change_.x());
  foot_spline_.roll()->addPoint(half_period, 0.0);

  foot_spline_.pitch()->addPoint(0.0, foot_orientation_pos_at_last_foot_change_.y(),
                                 foot_orientation_vel_at_last_foot_change_.y(),
                                 foot_orientation_acc_at_foot_change_.y());
  foot_spline_.pitch()->addPoint(half_period, 0.0);

  foot_spline_.yaw()->addPoint(0.0, foot_orientation_pos_at_last_foot_change_.z(),
                               foot_orientation_vel_at_last_foot_change_.z(),
                               foot_orientation_acc_at_foot_change_.z());
  foot_spline_.yaw()->addPoint(double_support_length,
                               getLastEuler().z());
  foot_spline_.yaw()->addPoint(double_support_length + single_support_length * params_.foot_put_down_phase,
                               0.0);
  foot_spline_.yaw()->addPoint(half_period, 0.0);

  //Trunk position
  trunk_spline_.x()->addPoint(0.0,
                              trunk_pos_at_foot_change_.x(),
                              trunk_pos_vel_at_foot_change_.x(),
                              trunk_pos_acc_at_foot_change_.x());
  trunk_spline_.x()->addPoint(half_period,
                              params_.trunk_x_offset);

  trunk_spline_.y()->addPoint(0.0,
                              trunk_pos_at_foot_change_.y(),
                              trunk_pos_vel_at_foot_change_.y(),
                              trunk_pos_acc_at_foot_change_.y());
  // move trunk in the center
  trunk_spline_.y()->addPoint(half_period,
                              -support_sign * 0.5 * params_.foot_distance + params_.trunk_y_offset);

  trunk_spline_.z()->addPoint(0.0,
                              trunk_pos_at_foot_change_.z(),
                              trunk_pos_vel_at_foot_change_.z(),
                              trunk_pos_acc_at_foot_change_.z());
  trunk_spline_.z()->addPoint(half_period,
                              params_.trunk_height);

  //Trunk orientation
  trunk_spline_.roll()->addPoint(0.0,
                                 trunk_orientation_pos_at_last_foot_change_.x(),
                                 trunk_orientation_vel_at_last_foot_change_.x(),
                                 trunk_orientation_acc_at_foot_change_.x());
  trunk_spline_.roll()->addPoint(half_period, 0.0);
  trunk_spline_.pitch()->addPoint(0.0,
                                  trunk_orientation_pos_at_last_foot_change_.y(),
                                  trunk_orientation_vel_at_last_foot_change_.y(),
                                  trunk_orientation_acc_at_foot_change_.y());
  trunk_spline_.pitch()->addPoint(half_period,
                                  params_.trunk_pitch);
  trunk_spline_.yaw()->addPoint(0.0,
                                trunk_orientation_pos_at_last_foot_change_.z(),
                                trunk_orientation_vel_at_last_foot_change_.z(),
                                trunk_orientation_acc_at_foot_change_.z());
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

double WalkEngine::getPhaseResetPhase() const {
  // returning the phase when the foot is on apex in step phase (between 0 and 0.5)
  return (params_.double_support_ratio + params_.foot_apex_phase * (1 - params_.double_support_ratio)) / 2;
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

bool WalkEngine::isLeftSupport() const {
  return is_left_support_foot_;
}

bool WalkEngine::isDoubleSupport() {
  // returns true if the value of the "is_double_support" spline is currently higher than 0.5
  // the spline should only have values of 0 or 1
  return is_double_support_spline_.pos(getTrajsTime()) >= 0.5;
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

double WalkEngine::getFreq() const {
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

void WalkEngine::stepFromOrders(const std::vector<double> &linear_orders, double angular_z) {
  //Compute step diff in next support foot frame
  tf2::Transform tmp_diff = tf2::Transform();
  tmp_diff.setIdentity();
  //No change in forward step and upward step
  tmp_diff.getOrigin()[0] = linear_orders[0];
  tmp_diff.getOrigin()[2] = linear_orders[1];
  //Add lateral foot offset
  if (is_left_support_foot_) {
    tmp_diff.getOrigin()[1] = params_.foot_distance;
  } else {
    tmp_diff.getOrigin()[1] = -1 * params_.foot_distance;
  }
  //Allow lateral step only on external foot
  //(internal foot will return to zero pose)
  if (
      (is_left_support_foot_ && linear_orders[1] > 0.0) ||
          (!is_left_support_foot_ && linear_orders[1] < 0.0)
      ) {
    tmp_diff.getOrigin()[1] += linear_orders[1];
  }
  //No change in turn (in order to rotate around trunk center)
  tf2::Quaternion quat;
  quat.setRPY(0, 0, angular_z);
  tmp_diff.setRotation(quat);

  //Make the step
  stepFromSupport(tmp_diff);
}

tf2::Vector3 WalkEngine::getNextEuler() {
  double roll, pitch, yaw;
  tf2::Matrix3x3(support_to_next_.getRotation()).getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

tf2::Vector3 WalkEngine::getLastEuler() {
  double roll, pitch, yaw;
  tf2::Matrix3x3(support_to_last_.getRotation()).getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

tf2::Transform WalkEngine::getLeft() {
  return left_in_world_;
}

tf2::Transform WalkEngine::getRight() {
  return right_in_world_;
}

bool WalkEngine::onSetParameters(const rclcpp::Parameter &parameter) {
  if (parameter.get_name() == "engine.freq") {
    params_.freq = parameter.as_double();
  } else if (parameter.get_name() == "engine.double_support_ratio") {
    params_.double_support_ratio = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_distance") {
    params_.foot_distance = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_rise") {
    params_.foot_rise = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_swing") {
    params_.trunk_swing = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_height") {
    params_.trunk_height = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_pitch") {
    params_.trunk_pitch = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_pitch_p_coef_forward") {
    params_.trunk_pitch_p_coef_forward = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_phase") {
    params_.trunk_phase = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_z_pause") {
    params_.foot_z_pause = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_put_down_z_offset") {
    params_.foot_put_down_z_offset = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_put_down_phase") {
    params_.foot_put_down_phase = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_apex_phase") {
    params_.foot_apex_phase = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_overshoot_ratio") {
    params_.foot_overshoot_ratio = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_overshoot_phase") {
    params_.foot_overshoot_phase = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_x_offset") {
    params_.trunk_x_offset = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_y_offset") {
    params_.trunk_y_offset = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_pause") {
    params_.trunk_pause = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_x_offset_p_coef_forward") {
    params_.trunk_x_offset_p_coef_forward = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_x_offset_p_coef_turn") {
    params_.trunk_x_offset_p_coef_turn = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_pitch_p_coef_turn") {
    params_.trunk_pitch_p_coef_turn = parameter.as_double();
  } else if (parameter.get_name() == "engine.kick_length") {
    params_.kick_length = parameter.as_double();
  } else if (parameter.get_name() == "engine.kick_vel") {
    params_.kick_vel = parameter.as_double();
  } else if (parameter.get_name() == "engine.kick_phase") {
    params_.kick_phase = parameter.as_double();
  } else if (parameter.get_name() == "engine.foot_put_down_roll_offset") {
    params_.foot_put_down_roll_offset = parameter.as_double();
  } else if (parameter.get_name() == "engine.first_step_swing_factor") {
    params_.first_step_swing_factor = parameter.as_double();
  } else if (parameter.get_name() == "engine.first_step_trunk_phase") {
    params_.first_step_trunk_phase = parameter.as_double();
  } else if (parameter.get_name() == "engine.trunk_z_movement") {
    params_.trunk_z_movement = parameter.as_double();
  } else {
    return false;
  }
  return true;
}

} // namespace bitbots_quintic_walk