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
    trunk_pos_vel_at_last_(),
    trunk_pos_acc_at_last_(),
    trunk_axis_pos_at_last_(),
    trunk_axis_vel_at_last_(),
    trunk_axis_acc_at_last_(),
    time_paused_(0.0) {
  // make sure to call the reset method after having the parameters
  is_double_support_ = bitbots_splines::SmoothSpline();
  is_left_support_foot_ = bitbots_splines::SmoothSpline();
  trunk_ = bitbots_splines::PoseSpline();
  foot_ = bitbots_splines::PoseSpline();

  // init dynamic reconfigure
  dyn_reconf_server_ = new dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_engine_paramsConfig>(ros::NodeHandle("~/engine"));
  dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_engine_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_quintic_walk::QuinticWalk::reconfCallback,this, _1, _2);
  dyn_reconf_server_->setCallback(f);

}

void QuinticWalk::setGoals(const WalkRequest &goals) {
  request_ = goals;
}

WalkResponse QuinticWalk::update(double dt) {
  bool orders_zero = request_.orders==tf2::Transform();

  // First check if we are currently in pause state or idle, since we don't want to update the phase in this case
  if (engine_state_=="paused") {
    if (time_paused_ > pause_duration_) {
      // our pause is finished, see if we can continue walking
      if (pause_requested_) {
        // not yet, wait another pause duration
        pause_requested_ = false;
        time_paused_ = 0.0;
        return computeCartesianPositionAtTime(getTrajsTime());
      } else {
        // we can continue
        engine_state_ = "walking";
        time_paused_ = 0.0;
      }
    } else {
      time_paused_ += dt;
      return computeCartesianPositionAtTime(getTrajsTime());
    }
    // we don't have to update anything more
  } else if (engine_state_=="idle") {
    if (orders_zero || !request_.walkable_state) {
      // we are in idle and are not supposed to walk. current state is fine, just do nothing
      return computeCartesianPositionAtTime(getTrajsTime());
    }
  }

  // update the current phase
  updatePhase(dt);

  // check if we will finish a half step with this update
  bool half_step_finished = (last_phase_ < 0.5 && phase_ >= 0.5) || (last_phase_ > 0.5 && phase_ < 0.5);

  // small state machine
  if (engine_state_=="idle") {
    // state is idle and orders are not zero, we can start walking
    buildStartTrajectories();
    engine_state_ = "startMovement";
  } else if (engine_state_=="startMovement") {
    // in this state we do a single "step" where we only move the trunk
    if (half_step_finished) {
      if (orders_zero) {
        engine_state_ = "stopMovement";
        buildStopMovementTrajectories();
      } else {
        //start step is finished, go to next state
        buildTrajectories(false, true, false);
        engine_state_ = "startStep";
      }
    }
  } else if (engine_state_=="startStep") {
    if (half_step_finished) {
      if (orders_zero) {
        // we have zero command vel -> we should stop
        engine_state_ = "stopStep";
        //phase_ = 0.0;
        buildStopStepTrajectories();
      } else {
        //start step is finished, go to next state
        buildNormalTrajectories();
        engine_state_ = "walking";
      }
    }
  } else if (engine_state_=="walking") {
    // check if a half step was finished and we are unstable
    if (half_step_finished && pause_requested_) {
      // go into pause
      engine_state_ = "paused";
      pause_requested_ = false;
      return computeCartesianPositionAtTime(getTrajsTime());
    } else if (half_step_finished &&
        ((left_kick_requested_ && !footstep_.isLeftSupport())
            || (right_kick_requested_ && footstep_.isLeftSupport()))) {
      // lets do a kick
      buildKickTrajectories();
      engine_state_ = "kick";
      left_kick_requested_ = false;
      right_kick_requested_ = false;
    } else if (half_step_finished) {
      // current step is finished, lets see if we have to change state
      if (orders_zero) {
        // we have zero command vel -> we should stop
        engine_state_ = "stopStep";
        //phase_ = 0.0;
        buildStopStepTrajectories();
      } else {
        // we can keep on walking
        buildNormalTrajectories();
      }
    }
  } else if (engine_state_=="kick") {
    // in this state we do a kick while doing a step
    if (half_step_finished) {
      //kick step is finished, go on walking
      engine_state_ = "walking";
      buildNormalTrajectories();
    }
  } else if (engine_state_=="stopStep") {
    // in this state we do a step back to get feet into idle pose
    if (half_step_finished) {
      //stop step is finished, go to stop movement state
      engine_state_ = "stopMovement";
      buildStopMovementTrajectories();
    }
  } else if (engine_state_=="stopMovement") {
    // in this state we do a "step" where we move the trunk back to idle position
    if (half_step_finished) {
      //stop movement is finished, go to idle state
      engine_state_ = "idle";
      return computeCartesianPositionAtTime(getTrajsTime());
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
    return computeCartesianPositionAtTime(getTrajsTime());
  }
  last_phase_ = phase_;

  return computeCartesianPositionAtTime(getTrajsTime());
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

  // get next support frame
  tf2::Transform next_support = footstep_.getNext();

  // get last values of trunk pose and its velocities and accelerations
  tf2::Vector3 trunk_pos = trunk_.getPositionPos(period_time);
  tf2::Vector3 trunk_pos_vel = trunk_.getPositionVel(period_time);
  tf2::Vector3 trunk_pos_acc = trunk_.getPositionAcc(period_time);
  tf2::Vector3 trunk_axis_pos = trunk_.getEulerAngles(period_time);
  tf2::Vector3 trunk_axis_vel = trunk_.getEulerVel(period_time);
  tf2::Vector3 trunk_axis_acc = trunk_.getEulerAcc(period_time);

  //Convert in next support foot frame and save
  trunk_pos_at_last_ = next_support*trunk_pos;
  trunk_pos_vel_at_last_ = next_support*trunk_pos_vel;
  trunk_pos_acc_at_last_ = next_support*trunk_pos_acc;
  trunk_axis_pos_at_last_ = next_support*trunk_axis_pos;
  trunk_axis_vel_at_last_ = next_support*trunk_axis_vel;
  trunk_axis_acc_at_last_ = next_support*trunk_axis_acc;
}

void QuinticWalk::buildNormalTrajectories() {
  buildTrajectories(false, false, false);
}

void QuinticWalk::buildKickTrajectories() {
  buildTrajectories(false, false, true);
}

void QuinticWalk::buildStartTrajectories() {
  buildTrajectories(true, false, false);
}

void QuinticWalk::buildStopStepTrajectories() {
  buildWalkDisableTrajectories(false);
}

void QuinticWalk::buildStopMovementTrajectories() {
  buildWalkDisableTrajectories(true);
}

void QuinticWalk::buildTrajectories(bool start_movement, bool start_step, bool kick_step) {
  // save the current trunk state to use it later
  if (!start_movement) {
    saveCurrentTrunkState();
  } else {
    // when we do start step, only transform the y coordinate since we stand still and only move trunk sideward
    trunk_pos_at_last_[1] = trunk_pos_at_last_.y() - footstep_.getNextPos().y();
  }

  if (start_movement) {
    // update support foot and compute odometry
    footstep_.stepFromOrders(tf2::Transform());
  } else {
    footstep_.stepFromOrders(request_.orders);
  }

  //Reset the trajectories
  is_double_support_ = bitbots_splines::SmoothSpline();
  is_left_support_foot_ = bitbots_splines::SmoothSpline();
  trunk_ = bitbots_splines::PoseSpline();
  foot_ = bitbots_splines::PoseSpline();

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
  point(is_double_support_, 0.0, 1.0);
  point(is_double_support_, double_support_length, 1.0);
  point(is_double_support_, double_support_length, 0.0);
  point(is_double_support_, half_period, 0.0);

  //Set support foot
  point(is_left_support_foot_, 0.0, footstep_.isLeftSupport());
  point(is_left_support_foot_, half_period, footstep_.isLeftSupport());

  //Flying foot position
  point(foot_.x(), 0.0, footstep_.getLastPos().x());
  point(foot_.x(), double_support_length, footstep_.getLastPos().x());
  if (kick_step) {
    point(foot_.x(), double_support_length + single_support_length*params_.kick_phase,
          footstep_.getNextPos().x() + params_.kick_length,
          params_.kick_vel);
  } else {
    point(foot_.x(),
          double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
          footstep_.getNextPos().x() +
              footstep_.getNextPos().x()*params_.foot_overshoot_ratio);
  }
  point(foot_.x(), double_support_length + single_support_length*params_.foot_put_down_phase,
        footstep_.getNextPos().x());
  point(foot_.x(), half_period, footstep_.getNextPos().x());

  point(foot_.y(), 0.0, footstep_.getLastPos().y());
  point(foot_.y(), double_support_length, footstep_.getLastPos().y());
  point(foot_.y(),
        double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
        footstep_.getNextPos().y()
            + (footstep_.getNextPos().y() - footstep_.getLastPos().y())*params_.foot_overshoot_ratio);
  point(foot_.y(), double_support_length + single_support_length*params_.foot_put_down_phase,
        footstep_.getNextPos().y());
  point(foot_.y(), half_period, footstep_.getNextPos().y());

  point(foot_.z(), 0.0, 0.0);
  point(foot_.z(), double_support_length, 0.0);
  point(foot_.z(),
        double_support_length + single_support_length*params_.foot_apex_phase
            - 0.5*params_.foot_z_pause*single_support_length,
        params_.foot_rise);
  point(foot_.z(),
        double_support_length + single_support_length*params_.foot_apex_phase
            + 0.5*params_.foot_z_pause*single_support_length,
        params_.foot_rise);
  point(foot_.z(), double_support_length + single_support_length*params_.foot_put_down_phase,
        params_.foot_put_down_z_offset);
  point(foot_.z(), half_period, 0.0);

  //Flying foot orientation
  point(foot_.roll(), 0.0, 0.0);
  point(foot_.roll(), double_support_length + 0.1*single_support_length,
        0.0);
  point(foot_.roll(), double_support_length + single_support_length*params_.foot_put_down_phase,
        params_.foot_put_down_roll_offset*support_sign);
  point(foot_.roll(), half_period, 0.0);

  point(foot_.pitch(), 0.0, 0.0);
  point(foot_.pitch(), half_period, 0.0);

  point(foot_.yaw(), 0.0, footstep_.getLastEuler().z());
  point(foot_.yaw(), double_support_length, footstep_.getLastEuler().z());
  point(foot_.yaw(), double_support_length + single_support_length*params_.foot_put_down_phase,
        footstep_.getNextEuler().z());
  point(foot_.yaw(), half_period, footstep_.getNextEuler().z());


  //Half pause length of trunk swing
  //lateral oscillation
  double pause_length = 0.5*params_.trunk_pause*half_period;

  //Trunk support foot and next
  //support foot external
  //oscillating position
  tf2::Vector3 trunk_point_support(
      params_.trunk_x_offset
          + params_.trunk_x_offset_p_coef_forward*footstep_.getNextPos().x()
          + params_.trunk_x_offset_p_coef_turn*fabs(footstep_.getNextPos().z()),
      params_.trunk_y_offset,
      0);
  tf2::Vector3 trunk_point_next(
      footstep_.getNextPos().x() + params_.trunk_x_offset
          + params_.trunk_x_offset_p_coef_forward*footstep_.getNextPos().x()
          + params_.trunk_x_offset_p_coef_turn*fabs(footstep_.getNextEuler().z()),
      footstep_.getNextPos().y() + params_.trunk_y_offset,
      0);
  //Trunk middle neutral (no swing) position
  tf2::Vector3 trunk_point_middle =
      0.5*trunk_point_support + 0.5*trunk_point_next;
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
      (footstep_.getNextPos().x() - footstep_.getLastPos().x())/period;
  double trunk_vel_next =
      footstep_.getNextPos().x()/half_period;

  //Trunk position
  if (start_step) {
    point(trunk_.x(), 0.0,
          0.0,
          0.0,
          0.0);
  } else {
    point(trunk_.x(), 0.0,
          trunk_pos_at_last_.x(),
          trunk_pos_vel_at_last_.x(),
          trunk_pos_acc_at_last_.x());
    point(trunk_.x(), half_period + time_shift,
          trunk_apex_support.x(),
          trunk_vel_support);
  }
  point(trunk_.x(), period + time_shift,
        trunk_apex_next.x(),
        trunk_vel_next);

  point(trunk_.y(), 0.0,
        trunk_pos_at_last_.y(),
        trunk_pos_vel_at_last_.y(),
        trunk_pos_acc_at_last_.y());
  if (start_step || start_movement) {
    point(trunk_.y(), half_period + time_shift - pause_length,
          trunk_point_middle.y() + trunk_vect.y()*params_.first_step_swing_factor);
    point(trunk_.y(), half_period + time_shift + pause_length,
          trunk_point_middle.y() + trunk_vect.y()*params_.first_step_swing_factor);
    point(trunk_.y(), period + time_shift - pause_length,
          trunk_point_middle.y() - trunk_vect.y()*params_.first_step_swing_factor);
    point(trunk_.y(), period + time_shift + pause_length,
          trunk_point_middle.y() - trunk_vect.y()*params_.first_step_swing_factor);
  } else {
    point(trunk_.y(), half_period + time_shift - pause_length,
          trunk_apex_support.y());
    point(trunk_.y(), half_period + time_shift + pause_length,
          trunk_apex_support.y());
    point(trunk_.y(), period + time_shift - pause_length,
          trunk_apex_next.y());
    point(trunk_.y(), period + time_shift + pause_length,
          trunk_apex_next.y());
  }

  point(trunk_.z(), 0.0,
        trunk_pos_at_last_.z(),
        trunk_pos_vel_at_last_.z(),
        trunk_pos_acc_at_last_.z());
  point(trunk_.z(), half_period + time_shift,
        params_.trunk_height);
  point(trunk_.z(), period + time_shift,
        params_.trunk_height);

  //Define trunk rotation as rool pitch yaw
  tf2::Vector3 euler_at_support = tf2::Vector3(
      0.0,
      params_.trunk_pitch
          + params_.trunk_pitch_p_coef_forward*footstep_.getNextPos().x()
          + params_.trunk_pitch_p_coef_turn*fabs(footstep_.getNextEuler().z()),
      0.5*footstep_.getLastEuler().z() + 0.5*footstep_.getNextEuler().z());
  tf2::Vector3 euler_at_next = tf2::Vector3(
      0.0,
      params_.trunk_pitch
          + params_.trunk_pitch_p_coef_forward*footstep_.getNextPos().x()
          + params_.trunk_pitch_p_coef_turn*fabs(footstep_.getNextEuler().z()),
      footstep_.getNextEuler().z());

  // we set a velocity for the points in yaw since we want to keep the speed in turning direction for next step
  // in roll and pitch, no velocity is set since changes are only minor when speed changes
  tf2::Vector3 axis_vel(
      0.0, 0.0,
      bitbots_splines::AngleDistance(
          footstep_.getLastEuler().z(),
          footstep_.getNextEuler().z())/period);

  //Trunk orientation
  point(trunk_.roll(), 0.0,
        trunk_axis_pos_at_last_.x(),
        trunk_axis_vel_at_last_.x(),
        trunk_axis_acc_at_last_.x());
  point(trunk_.roll(), half_period + time_shift,
        euler_at_support.x(),
        axis_vel.x());
  point(trunk_.roll(), period + time_shift,
        euler_at_next.x(),
        axis_vel.x());

  point(trunk_.pitch(), 0.0,
        trunk_axis_pos_at_last_.y(),
        trunk_axis_vel_at_last_.y(),
        trunk_axis_acc_at_last_.y());
  point(trunk_.pitch(), half_period + time_shift,
        euler_at_support.y(),
        axis_vel.y());
  point(trunk_.pitch(), period + time_shift,
        euler_at_next.y(),
        axis_vel.y());

  point(trunk_.yaw(), 0.0,
        trunk_axis_pos_at_last_.z(),
        trunk_axis_vel_at_last_.z(),
        trunk_axis_acc_at_last_.z());
  point(trunk_.yaw(), half_period + time_shift,
        euler_at_support.z(),
        axis_vel.z());
  point(trunk_.yaw(), period + time_shift,
        euler_at_next.z(),
        axis_vel.z());
}

void QuinticWalk::buildWalkDisableTrajectories(bool foot_in_idle_position) {
  // save the current trunk state to use it later
  saveCurrentTrunkState();
  // update support foot and compute odometry
  footstep_.stepFromOrders(request_.orders);

  //Reset the trajectories
  is_double_support_ = bitbots_splines::SmoothSpline();
  is_left_support_foot_ = bitbots_splines::SmoothSpline();
  trunk_ = bitbots_splines::PoseSpline();
  foot_ = bitbots_splines::PoseSpline();

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
  point(is_double_support_, 0.0, is_double_support);
  point(is_double_support_, half_period, is_double_support);

  //Set support foot
  point(is_left_support_foot_, 0.0, footstep_.isLeftSupport());
  point(is_left_support_foot_, half_period, footstep_.isLeftSupport());

  //Flying foot position
  point(foot_.x(), 0.0,
        footstep_.getLastPos().x());
  point(foot_.x(), double_support_length,
        footstep_.getLastPos().x());
  point(foot_.x(),
        double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
        0.0 + (0.0 - footstep_.getLastPos().x())*params_.foot_overshoot_ratio);
  point(foot_.x(), double_support_length + single_support_length*params_.foot_put_down_phase,
        0.0);
  point(foot_.x(), half_period,
        0.0);

  point(foot_.y(), 0.0,
        footstep_.getLastPos().y());
  point(foot_.y(), double_support_length,
        footstep_.getLastPos().y());
  point(foot_.y(),
        double_support_length + single_support_length*params_.foot_put_down_phase*params_.foot_overshoot_phase,
        -support_sign*params_.foot_distance
            + (-support_sign*params_.foot_distance - footstep_.getLastPos().y())*params_.foot_overshoot_ratio);
  point(foot_.x(), double_support_length + single_support_length*params_.foot_put_down_phase,
        -support_sign*params_.foot_distance);
  point(foot_.y(), half_period,
        -support_sign*params_.foot_distance);

  //If the walk has just been disabled,
  //make one single step to neutral pose
  if (!foot_in_idle_position) {
    point(foot_.z(), 0.0, 0.0);
    point(foot_.z(), double_support_length, 0.0);
    point(foot_.z(),
          double_support_length + single_support_length*params_.foot_apex_phase
              - 0.5*params_.foot_z_pause*single_support_length,
          params_.foot_rise);
    point(foot_.z(),
          double_support_length + single_support_length*params_.foot_apex_phase
              + 0.5*params_.foot_z_pause*single_support_length,
          params_.foot_rise);
    point(foot_.z(), double_support_length + single_support_length*params_.foot_put_down_phase,
          params_.foot_put_down_z_offset);
    point(foot_.z(), half_period,
          0.0);
  } else {
    //dont move the foot in last single step before stop since we only move the trunk back to the center
    point(foot_.z(), 0.0, 0.0);
    point(foot_.z(), half_period, 0.0);
  }
  //Flying foot orientation
  point(foot_.roll(), 0.0, 0.0);
  point(foot_.roll(), half_period, 0.0);

  point(foot_.pitch(), 0.0, 0.0);
  point(foot_.pitch(), half_period, 0.0);

  point(foot_.yaw(), 0.0, footstep_.getLastEuler().z());
  point(foot_.yaw(), double_support_length,
        footstep_.getLastEuler().z());
  point(foot_.yaw(), double_support_length + single_support_length*params_.foot_put_down_phase,
        0.0);
  point(foot_.yaw(), half_period, 0.0);

  //Trunk position
  point(trunk_.x(), 0.0,
        trunk_pos_at_last_.x(),
        trunk_pos_vel_at_last_.x(),
        trunk_pos_acc_at_last_.x());
  point(trunk_.x(), half_period,
        params_.trunk_x_offset);

  point(trunk_.y(), 0.0,
        trunk_pos_at_last_.y(),
        trunk_pos_vel_at_last_.y(),
        trunk_pos_acc_at_last_.y());
  point(trunk_.y(), half_period,
        -support_sign*0.5*params_.foot_distance + params_.trunk_y_offset);

  point(trunk_.z(), 0.0,
        trunk_pos_at_last_.z(),
        trunk_pos_vel_at_last_.z(),
        trunk_pos_acc_at_last_.z());
  point(trunk_.z(), half_period,
        params_.trunk_height);
  //Trunk orientation
  point(trunk_.roll(), 0.0,
        trunk_axis_pos_at_last_.x(),
        trunk_axis_vel_at_last_.x(),
        trunk_axis_acc_at_last_.x());
  point(trunk_.roll(), half_period, 0.0);
  point(trunk_.pitch(), 0.0,
        trunk_axis_pos_at_last_.y(),
        trunk_axis_vel_at_last_.y(),
        trunk_axis_acc_at_last_.y());
  point(trunk_.pitch(), half_period,
        params_.trunk_pitch);
  point(trunk_.yaw(), 0.0,
        trunk_axis_pos_at_last_.z(),
        trunk_axis_vel_at_last_.z(),
        trunk_axis_acc_at_last_.z());
  point(trunk_.yaw(), half_period,
        0.0);
}

void QuinticWalk::resetTrunkLastState() {
  if (footstep_.isLeftSupport()) {
    trunk_pos_at_last_ = tf2::Vector3(
        params_.trunk_x_offset,
        -params_.foot_distance/2.0 + params_.trunk_y_offset,
        params_.trunk_height);
  } else {
    trunk_pos_at_last_ = tf2::Vector3(
        params_.trunk_x_offset,
        params_.foot_distance/2.0 + params_.trunk_y_offset,
        params_.trunk_height);
  }
  trunk_pos_vel_at_last_.setZero();
  trunk_pos_acc_at_last_.setZero();
  trunk_axis_pos_at_last_ = tf2::Vector3(0.0, params_.trunk_pitch, 0.0);
  trunk_axis_vel_at_last_.setZero();
  trunk_axis_acc_at_last_.setZero();
}

WalkResponse QuinticWalk::computeCartesianPositionAtTime(double time) {
  //Evaluate target cartesian
  //state from trajectories
  WalkResponse response;
  response.is_double_support = is_double_support_.pos(time) >= 0.5;
  response.is_left_support_foot = is_left_support_foot_.pos(time) >= 0.5;
  response.support_foot_to_flying_foot = foot_.getTfTransform(time);
  response.support_foot_to_trunk = trunk_.getTfTransform(time);
  return response;
}

void QuinticWalk::point(bitbots_splines::SmoothSpline spline, double t, double pos, double vel, double acc) {
  spline.addPoint(t, pos, vel, acc);
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
  return is_double_support_.pos(getTrajsTime()) >= 0.5;
}

void QuinticWalk::reconfCallback(bitbots_quintic_walk_engine_paramsConfig &params, uint32_t level) {
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

bitbots_splines::Trajectories QuinticWalk::getSplines() const{
  bitbots_splines::Trajectories trajs;
  ROS_ERROR("Method getSplines not implemented");
  //TODO splines missing since they do not fit into spline container
  return trajs;
};

int QuinticWalk::getPercentDone() const{
  return (int) getTrajsTime()*100;
}

void QuinticWalk::setPauseDuration(double duration){
  pause_duration_ = duration;
}

double QuinticWalk::getFreq(){
  return params_.freq;
}

double QuinticWalk::getWantedTrunkPitch(){
  return params_.trunk_pitch + params_.trunk_pitch_p_coef_forward*footstep_.getNextPos().x()
      + params_.trunk_pitch_p_coef_turn*fabs(footstep_.getNextPos().z());
}

}

