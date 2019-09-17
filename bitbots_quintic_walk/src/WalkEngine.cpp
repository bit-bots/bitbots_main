/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/WalkEngine.hpp"

namespace bitbots_quintic_walk {

QuinticWalk::QuinticWalk() :
    _footstep(0.14, true),
    _phase(0.0),
    _lastPhase(0.0),
    _pauseRequested(false),
    _leftKickRequested(false),
    _rightKickRequested(false),
    _params(),
    _trunkPosAtLast(),
    _trunkVelAtLast(),
    _trunkAccAtLast(),
    _trunkAxisPosAtLast(),
    _trunkAxisVelAtLast(),
    _trunkAxisAccAtLast(),
    _trajs(),
    _timePaused(0.0) {
  // make sure to call the reset method after having the parameters
  _trajs = trajectoriesInit();
}

bool QuinticWalk::updateState(double dt, const Eigen::Vector3d &orders, bool walkable_state) {
  bool orders_zero = orders[0]==0.0 && orders[1]==0.0 && orders[2]==0.0;

  // First check if we are currently in pause state or idle, since we don't want to update the phase in this case
  if (_engineState=="paused") {
    if (_timePaused > _params.pauseDuration) {
      // our pause is finished, see if we can continue walking
      if (_pauseRequested) {
        // not yet, wait another pause duration
        _pauseRequested = false;
        _timePaused = 0.0;
        return false;
      } else {
        // we can continue
        _engineState = "walking";
        _timePaused = 0.0;
        /*buildNormalTrajectories(orders);
        updatePhase(dt);
        return true;*/
      }
    } else {
      _timePaused += dt;
      return false;
    }
    // we don't have to update anything more
  } else if (_engineState=="idle") {
    if (orders_zero || !walkable_state) {
      // we are in idle and are not supposed to walk. current state is fine, just do nothing
      return false;
    }
  }

  // update the current phase
  updatePhase(dt);

  // check if we will finish a half step with this update
  bool half_step_finished = (_lastPhase < 0.5 && _phase >= 0.5) || (_lastPhase > 0.5 && _phase < 0.5);

  // small state machine
  if (_engineState=="idle") {
    // state is idle and orders are not zero, we can start walking
    buildStartTrajectories(orders);
    _engineState = "startMovement";
  } else if (_engineState=="startMovement") {
    // in this state we do a single "step" where we only move the trunk
    if (half_step_finished) {
      if (orders_zero) {
        _engineState = "stopMovement";
        buildStopMovementTrajectories(orders);
      } else {
        //start step is finished, go to next state
        buildTrajectories(orders, false, true, false);
        _engineState = "startStep";
      }
    }
  } else if (_engineState=="startStep") {
    if (half_step_finished) {
      if (orders_zero) {
        // we have zero command vel -> we should stop
        _engineState = "stopStep";
        //_phase = 0.0;
        buildStopStepTrajectories(orders);
      } else {
        //start step is finished, go to next state
        buildNormalTrajectories(orders);
        _engineState = "walking";
      }
    }
  } else if (_engineState=="walking") {
    // check if a half step was finished and we are unstable
    if (half_step_finished && _pauseRequested) {
      // go into pause
      _engineState = "paused";
      _pauseRequested = false;
      return false;
    } else if (half_step_finished &&
        ((_leftKickRequested && !_footstep.isLeftSupport()) || (_rightKickRequested && _footstep.isLeftSupport()))) {
      // lets do a kick
      buildKickTrajectories(orders);
      _engineState = "kick";
      _leftKickRequested = false;
      _rightKickRequested = false;
    } else if (half_step_finished) {
      // current step is finished, lets see if we have to change state
      if (orders_zero) {
        // we have zero command vel -> we should stop
        _engineState = "stopStep";
        //_phase = 0.0;
        buildStopStepTrajectories(orders);
      } else {
        // we can keep on walking
        buildNormalTrajectories(orders);
      }
    }
  } else if (_engineState=="kick") {
    // in this state we do a kick while doing a step
    if (half_step_finished) {
      //kick step is finished, go on walking
      _engineState = "walking";
      buildNormalTrajectories(orders);
    }
  } else if (_engineState=="stopStep") {
    // in this state we do a step back to get feet into idle pose
    if (half_step_finished) {
      //stop step is finished, go to stop movement state
      _engineState = "stopMovement";
      buildStopMovementTrajectories(orders);
    }
  } else if (_engineState=="stopMovement") {
    // in this state we do a "step" where we move the trunk back to idle position
    if (half_step_finished) {
      //stop movement is finished, go to idle state
      _engineState = "idle";
      return false;
    }
  } else {
    ROS_ERROR("Somethings wrong with the walking engine state");
  }

  //Sanity check support foot state
  if ((_phase < 0.5 && !_footstep.isLeftSupport()) ||
      (_phase >= 0.5 && _footstep.isLeftSupport())) {
    ROS_ERROR_THROTTLE(1,
                       "QuinticWalk exception invalid state phase= %f support= %d dt= %f",
                       _phase,
                       _footstep.isLeftSupport(),
                       dt);
    return false;
  }
  _lastPhase = _phase;

  return true;
}

void QuinticWalk::updatePhase(double dt) {
  //Check for negative time step
  if (dt <= 0.0) {
    if (dt==0.0) { //sometimes happens due to rounding
      dt = 0.0001;
    } else {
      ROS_ERROR_THROTTLE(1, "QuinticWalk exception negative dt phase= %f dt= %f", _phase, dt);
      return;
    }
  }
  //Check for too long dt
  if (dt > 0.25/_params.freq) {
    ROS_ERROR_THROTTLE(1, "QuinticWalk error too long dt phase= %f dt= %f", _phase, dt);
    return;
  }

  //Update the phase
  _phase += dt*_params.freq;

  // reset to 0 if step complete
  if (_phase > 1.0) {
    _phase = 0.0;
  }
}

void QuinticWalk::endStep() {
  // ends the step earlier, e.g. when foot has already contact to ground
  if (_phase < 0.5) {
    _phase = 0.5;
  } else {
    _phase = 0.0;
  }
}

void QuinticWalk::reset() {
  _engineState = "idle";
  _phase = 0.0;
  _timePaused = 0.0;

  //Initialize the footstep
  _footstep.setFootDistance(_params.footDistance);
  _footstep.reset(false);
  //Reset the trunk saved state
  resetTrunkLastState();
}

void QuinticWalk::saveCurrentTrunkState() {
  //Evaluate current trunk state
  //(position, velocity, acceleration)
  //in next support foot frame

  // compute current point in time to save state
  // by multiplying the half_period time with the advancement of period time
  double half_period = 1.0/(2.0*_params.freq);
  double factor;
  if (_lastPhase < 0.5) {
    factor = _lastPhase/0.5;
  } else {
    factor = _lastPhase;
  }
  double period_time = half_period*factor;

  Eigen::Vector2d trunk_pos(
      _trajs.get("trunk_pos_x").pos(period_time),
      _trajs.get("trunk_pos_y").pos(period_time));
  Eigen::Vector2d trunk_vel(
      _trajs.get("trunk_pos_x").vel(period_time),
      _trajs.get("trunk_pos_y").vel(period_time));
  Eigen::Vector2d trunk_acc(
      _trajs.get("trunk_pos_x").acc(period_time),
      _trajs.get("trunk_pos_y").acc(period_time));
  //Convert in next support foot frame
  trunk_pos.x() -= _footstep.getNext().x();
  trunk_pos.y() -= _footstep.getNext().y();
  trunk_pos = Eigen::Rotation2Dd(
      -_footstep.getNext().z()).toRotationMatrix()
      *trunk_pos;
  trunk_vel = Eigen::Rotation2Dd(
      -_footstep.getNext().z()).toRotationMatrix()
      *trunk_vel;
  trunk_acc = Eigen::Rotation2Dd(
      -_footstep.getNext().z()).toRotationMatrix()
      *trunk_acc;
  //Save state
  _trunkPosAtLast.x() = trunk_pos.x();
  _trunkPosAtLast.y() = trunk_pos.y();
  _trunkVelAtLast.x() = trunk_vel.x();
  _trunkVelAtLast.y() = trunk_vel.y();
  _trunkAccAtLast.x() = trunk_acc.x();
  _trunkAccAtLast.y() = trunk_acc.y();
  //No transformation for height
  _trunkPosAtLast.z() = _trajs.get("trunk_pos_z").pos(period_time);
  _trunkVelAtLast.z() = _trajs.get("trunk_pos_z").vel(period_time);
  _trunkAccAtLast.z() = _trajs.get("trunk_pos_z").acc(period_time);
  //Evaluate and save trunk orientation
  //in next support foot frame
  Eigen::Vector3d trunk_axis(
      _trajs.get("trunk_axis_x").pos(period_time),
      _trajs.get("trunk_axis_y").pos(period_time),
      _trajs.get("trunk_axis_z").pos(period_time));
  //Convert in intrinsic euler angle
  Eigen::Matrix3d trunk_mat = bitbots_splines::AxisToMatrix(trunk_axis);
  Eigen::Vector3d trunk_euler = bitbots_splines::MatrixToEulerIntrinsic(trunk_mat);
  //Transform to next support foot
  trunk_euler.z() -= _footstep.getNext().z();
  //Reconvert to axis and save it
  trunk_mat = bitbots_splines::EulerIntrinsicToMatrix(trunk_euler);
  trunk_axis = bitbots_splines::MatrixToAxis(trunk_mat);
  _trunkAxisPosAtLast = trunk_axis;
  //Evaluate trunk orientation velocity
  //and acceleration without frame
  //transformation
  _trunkAxisVelAtLast.x() = _trajs.get("trunk_axis_x").vel(period_time);
  _trunkAxisVelAtLast.y() = _trajs.get("trunk_axis_y").vel(period_time);
  _trunkAxisVelAtLast.z() = _trajs.get("trunk_axis_z").vel(period_time);
  _trunkAxisAccAtLast.x() = _trajs.get("trunk_axis_x").acc(period_time);
  _trunkAxisAccAtLast.y() = _trajs.get("trunk_axis_y").acc(period_time);
  _trunkAxisAccAtLast.z() = _trajs.get("trunk_axis_z").acc(period_time);
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
    _trunkPosAtLast.y() -= _footstep.getNext().y();
    //trunkPos = Eigen::Rotation2Dd(-_footstep.getNext().z()).toRotationMatrix() * trunkPos;
  }

  if (start_movement) {
    // update support foot and compute odometry
    _footstep.stepFromOrders(Eigen::Vector3d::Zero());
  } else {
    _footstep.stepFromOrders(orders);
  }

  //Reset the trajectories
  _trajs = trajectoriesInit();
  //Set up the trajectories for the half cycle (single step)
  double half_period = 1.0/(2.0*_params.freq);
  // full period (double step) is needed for trunk splines
  double period = 2.0*half_period;

  //Time length of double and single support phase during the half cycle
  double double_support_length = _params.doubleSupportRatio*half_period;
  double single_support_length = half_period - double_support_length;

  //Sign of support foot with respect to lateral
  double support_sign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

  //The trunk trajectory is defined for a
  //complete cycle to handle trunk phase shift
  //Trunk phase shift is done due to the length of the double
  //support phase and can be adjusted optionally by a parameter
  // 0.5halfPeriod to be acyclic to the feet, 0.5doubleSupportLength to keep the double support phase centered between feet
  double time_shift = -0.5*half_period + 0.5*double_support_length + _params.trunkPhase*half_period;


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
  point("is_left_support_foot", 0.0, _footstep.isLeftSupport());
  point("is_left_support_foot", half_period, _footstep.isLeftSupport());

  //Flying foot position
  point("foot_pos_x", 0.0, _footstep.getLast().x());
  point("foot_pos_x", double_support_length, _footstep.getLast().x());
  if (kick_step) {
    point("foot_pos_x", double_support_length + single_support_length*_params.kickPhase,
          _footstep.getNext().x() + _params.kickLength,
          _params.kickVel);
  } else {
    point("foot_pos_x",
          double_support_length + single_support_length*_params.footPutDownPhase*_params.footOvershootPhase,
          _footstep.getNext().x() +
              _footstep.getNext().x()*_params.footOvershootRatio);
  }
  point("foot_pos_x", double_support_length + single_support_length*_params.footPutDownPhase,
        _footstep.getNext().x());
  point("foot_pos_x", half_period, _footstep.getNext().x());

  point("foot_pos_y", 0.0, _footstep.getLast().y());
  point("foot_pos_y", double_support_length, _footstep.getLast().y());
  point("foot_pos_y", double_support_length + single_support_length*_params.footPutDownPhase*_params.footOvershootPhase,
        _footstep.getNext().y() + (_footstep.getNext().y() - _footstep.getLast().y())*_params.footOvershootRatio);
  point("foot_pos_y", double_support_length + single_support_length*_params.footPutDownPhase,
        _footstep.getNext().y());
  point("foot_pos_y", half_period, _footstep.getNext().y());

  point("foot_pos_z", 0.0, 0.0);
  point("foot_pos_z", double_support_length, 0.0);
  point("foot_pos_z",
        double_support_length + single_support_length*_params.footApexPhase - 0.5*_params.footZPause*single_support_length,
        _params.footRise);
  point("foot_pos_z",
        double_support_length + single_support_length*_params.footApexPhase + 0.5*_params.footZPause*single_support_length,
        _params.footRise);
  point("foot_pos_z", double_support_length + single_support_length*_params.footPutDownPhase,
        _params.footPutDownZOffset);
  point("foot_pos_z", half_period, 0.0);

  //Flying foot orientation
  point("foot_axis_x", 0.0, 0.0);
  point("foot_axis_x", double_support_length + 0.1*single_support_length,
        0.0);
  point("foot_axis_x", double_support_length + single_support_length*_params.footPutDownPhase,
        _params.footPutDownRollOffset*support_sign);
  point("foot_axis_x", half_period, 0.0);

  point("foot_axis_y", 0.0, 0.0);
  point("foot_axis_y", half_period, 0.0);

  point("foot_axis_z", 0.0, _footstep.getLast().z());
  point("foot_axis_z", double_support_length, _footstep.getLast().z());
  point("foot_axis_z", double_support_length + single_support_length*_params.footPutDownPhase,
        _footstep.getNext().z());
  point("foot_axis_z", half_period, _footstep.getNext().z());


  //Half pause length of trunk swing
  //lateral oscillation
  double pauseLength = 0.5*_params.trunkPause*half_period;

  //Trunk support foot and next
  //support foot external
  //oscillating position
  Eigen::Vector2d trunkPointSupport(
      _params.trunkXOffset
          + _params.trunkXOffsetPCoefForward*_footstep.getNext().x()
          + _params.trunkXOffsetPCoefTurn*fabs(_footstep.getNext().z()),
      _params.trunkYOffset);
  Eigen::Vector2d trunkPointNext(
      _footstep.getNext().x() + _params.trunkXOffset
          + _params.trunkXOffsetPCoefForward*_footstep.getNext().x()
          + _params.trunkXOffsetPCoefTurn*fabs(_footstep.getNext().z()),
      _footstep.getNext().y() + _params.trunkYOffset);
  //Trunk middle neutral (no swing) position
  Eigen::Vector2d trunkPointMiddle =
      0.5*trunkPointSupport + 0.5*trunkPointNext;
  //Trunk vector from middle to support apex
  Eigen::Vector2d trunkVect =
      trunkPointSupport - trunkPointMiddle;
  //Apply swing amplitude ratio
  trunkVect.y() *= _params.trunkSwing;
  //Trunk support and next apex position
  Eigen::Vector2d trunkApexSupport =
      trunkPointMiddle + trunkVect;
  Eigen::Vector2d trunkApexNext =
      trunkPointMiddle - trunkVect;
  //Trunk forward velocity
  double trunkVelSupport =
      (_footstep.getNext().x() - _footstep.getLast().x())/period;
  double trunkVelNext =
      _footstep.getNext().x()/half_period;

  //Trunk position
  if (start_step) {
    point("trunk_pos_x", 0.0,
          0.0,
          0.0,
          0.0);
  } else {
    point("trunk_pos_x", 0.0,
          _trunkPosAtLast.x(),
          _trunkVelAtLast.x(),
          _trunkAccAtLast.x());
    point("trunk_pos_x", half_period + time_shift,
          trunkApexSupport.x(),
          trunkVelSupport);
  }
  point("trunk_pos_x", period + time_shift,
        trunkApexNext.x(),
        trunkVelNext);

  point("trunk_pos_y", 0.0,
        _trunkPosAtLast.y(),
        _trunkVelAtLast.y(),
        _trunkAccAtLast.y());
  if (start_step || start_movement) {
    point("trunk_pos_y", half_period + time_shift - pauseLength,
          trunkPointMiddle.y() + trunkVect.y()*_params.firstStepSwingFactor);
    point("trunk_pos_y", half_period + time_shift + pauseLength,
          trunkPointMiddle.y() + trunkVect.y()*_params.firstStepSwingFactor);
    point("trunk_pos_y", period + time_shift - pauseLength,
          trunkPointMiddle.y() - trunkVect.y()*_params.firstStepSwingFactor);
    point("trunk_pos_y", period + time_shift + pauseLength,
          trunkPointMiddle.y() - trunkVect.y()*_params.firstStepSwingFactor);
  } else {
    point("trunk_pos_y", half_period + time_shift - pauseLength,
          trunkApexSupport.y());
    point("trunk_pos_y", half_period + time_shift + pauseLength,
          trunkApexSupport.y());
    point("trunk_pos_y", period + time_shift - pauseLength,
          trunkApexNext.y());
    point("trunk_pos_y", period + time_shift + pauseLength,
          trunkApexNext.y());
  }

  point("trunk_pos_z", 0.0,
        _trunkPosAtLast.z(),
        _trunkVelAtLast.z(),
        _trunkAccAtLast.z());
  point("trunk_pos_z", half_period + time_shift,
        _params.trunkHeight);
  point("trunk_pos_z", period + time_shift,
        _params.trunkHeight);

  //Define trunk yaw target
  //orientation position and velocity
  //in euler angle and convertion
  //to axis vector
  Eigen::Vector3d eulerAtSuport(
      0.0,
      _params.trunkPitch
          + _params.trunkPitchPCoefForward*_footstep.getNext().x()
          + _params.trunkPitchPCoefTurn*fabs(_footstep.getNext().z()),
      0.5*_footstep.getLast().z() + 0.5*_footstep.getNext().z());
  Eigen::Vector3d eulerAtNext(
      0.0,
      _params.trunkPitch
          + _params.trunkPitchPCoefForward*_footstep.getNext().x()
          + _params.trunkPitchPCoefTurn*fabs(_footstep.getNext().z()),
      _footstep.getNext().z());
  Eigen::Matrix3d matAtSupport = bitbots_splines::EulerIntrinsicToMatrix(eulerAtSuport);
  Eigen::Matrix3d matAtNext = bitbots_splines::EulerIntrinsicToMatrix(eulerAtNext);
  Eigen::Vector3d axisAtSupport = bitbots_splines::MatrixToAxis(matAtSupport);
  Eigen::Vector3d axisAtNext = bitbots_splines::MatrixToAxis(matAtNext);
  Eigen::Vector3d axisVel(
      0.0, 0.0,
      bitbots_splines::AngleDistance(
          _footstep.getLast().z(),
          _footstep.getNext().z())/period);

  //Trunk orientation
  point("trunk_axis_x", 0.0,
        _trunkAxisPosAtLast.x(),
        _trunkAxisVelAtLast.x(),
        _trunkAxisAccAtLast.x());
  point("trunk_axis_x", half_period + time_shift,
        axisAtSupport.x(),
        axisVel.x());
  point("trunk_axis_x", period + time_shift,
        axisAtNext.x(),
        axisVel.x());

  point("trunk_axis_y", 0.0,
        _trunkAxisPosAtLast.y(),
        _trunkAxisVelAtLast.y(),
        _trunkAxisAccAtLast.y());
  point("trunk_axis_y", half_period + time_shift,
        axisAtSupport.y(),
        axisVel.y());
  point("trunk_axis_y", period + time_shift,
        axisAtNext.y(),
        axisVel.y());

  point("trunk_axis_z", 0.0,
        _trunkAxisPosAtLast.z(),
        _trunkAxisVelAtLast.z(),
        _trunkAxisAccAtLast.z());
  point("trunk_axis_z", half_period + time_shift,
        axisAtSupport.z(),
        axisVel.z());
  point("trunk_axis_z", period + time_shift,
        axisAtNext.z(),
        axisVel.z());
}

void QuinticWalk::buildWalkDisableTrajectories(const Eigen::Vector3d &orders, bool footInIdlePosition) {
  // save the current trunk state to use it later
  saveCurrentTrunkState();
  // update support foot and compute odometry
  _footstep.stepFromOrders(orders);

  //Reset the trajectories
  _trajs = trajectoriesInit();

  //Set up the trajectories
  //for the half cycle
  double halfPeriod = 1.0/(2.0*_params.freq);

  //Time length of double and single
  //support phase during the half cycle
  double doubleSupportLength = _params.doubleSupportRatio*halfPeriod;
  double singleSupportLength = halfPeriod - doubleSupportLength;

  //Sign of support foot with
  //respect to lateral
  double supportSign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

  //Set double support phase
  double isDoubleSupport = (footInIdlePosition ? 1.0 : 0.0);
  point("is_double_support", 0.0, isDoubleSupport);
  point("is_double_support", halfPeriod, isDoubleSupport);

  //Set support foot
  point("is_left_support_foot", 0.0, _footstep.isLeftSupport());
  point("is_left_support_foot", halfPeriod, _footstep.isLeftSupport());

  //Flying foot position
  point("foot_pos_x", 0.0,
        _footstep.getLast().x());
  point("foot_pos_x", doubleSupportLength,
        _footstep.getLast().x());
  point("foot_pos_x", doubleSupportLength + singleSupportLength*_params.footPutDownPhase*_params.footOvershootPhase,
        0.0 + (0.0 - _footstep.getLast().x())*_params.footOvershootRatio);
  point("foot_pos_x", doubleSupportLength + singleSupportLength*_params.footPutDownPhase,
        0.0);
  point("foot_pos_x", halfPeriod,
        0.0);

  point("foot_pos_y", 0.0,
        _footstep.getLast().y());
  point("foot_pos_y", doubleSupportLength,
        _footstep.getLast().y());
  point("foot_pos_y", doubleSupportLength + singleSupportLength*_params.footPutDownPhase*_params.footOvershootPhase,
        -supportSign*_params.footDistance
            + (-supportSign*_params.footDistance - _footstep.getLast().y())*_params.footOvershootRatio);
  point("foot_pos_x", doubleSupportLength + singleSupportLength*_params.footPutDownPhase,
        -supportSign*_params.footDistance);
  point("foot_pos_y", halfPeriod,
        -supportSign*_params.footDistance);

  //If the walk has just been disabled,
  //make one single step to neutral pose
  if (!footInIdlePosition) {
    point("foot_pos_z", 0.0, 0.0);
    point("foot_pos_z", doubleSupportLength, 0.0);
    point("foot_pos_z",
          doubleSupportLength + singleSupportLength*_params.footApexPhase - 0.5*_params.footZPause*singleSupportLength,
          _params.footRise);
    point("foot_pos_z",
          doubleSupportLength + singleSupportLength*_params.footApexPhase + 0.5*_params.footZPause*singleSupportLength,
          _params.footRise);
    point("foot_pos_z", doubleSupportLength + singleSupportLength*_params.footPutDownPhase,
          _params.footPutDownZOffset);
    point("foot_pos_z", halfPeriod,
          0.0);
  } else {
    //dont move the foot in last single step before stop since we only move the trunk back to the center
    point("foot_pos_z", 0.0, 0.0);
    point("foot_pos_z", halfPeriod, 0.0);
  }
  //Flying foot orientation
  point("foot_axis_x", 0.0, 0.0);
  point("foot_axis_x", halfPeriod, 0.0);

  point("foot_axis_y", 0.0, 0.0);
  point("foot_axis_y", halfPeriod, 0.0);

  point("foot_axis_z", 0.0, _footstep.getLast().z());
  point("foot_axis_z", doubleSupportLength,
        _footstep.getLast().z());
  point("foot_axis_z", doubleSupportLength + singleSupportLength*_params.footPutDownPhase,
        0.0);
  point("foot_axis_z", halfPeriod, 0.0);

  //Trunk position
  point("trunk_pos_x", 0.0,
        _trunkPosAtLast.x(),
        _trunkVelAtLast.x(),
        _trunkAccAtLast.x());
  point("trunk_pos_x", halfPeriod,
        _params.trunkXOffset);

  point("trunk_pos_y", 0.0,
        _trunkPosAtLast.y(),
        _trunkVelAtLast.y(),
        _trunkAccAtLast.y());
  point("trunk_pos_y", halfPeriod,
        -supportSign*0.5*_params.footDistance + _params.trunkYOffset);

  point("trunk_pos_z", 0.0,
        _trunkPosAtLast.z(),
        _trunkVelAtLast.z(),
        _trunkAccAtLast.z());
  point("trunk_pos_z", halfPeriod,
        _params.trunkHeight);
  //Trunk orientation
  point("trunk_axis_x", 0.0,
        _trunkAxisPosAtLast.x(),
        _trunkAxisVelAtLast.x(),
        _trunkAxisAccAtLast.x());
  point("trunk_axis_x", halfPeriod, 0.0);
  point("trunk_axis_y", 0.0,
        _trunkAxisPosAtLast.y(),
        _trunkAxisVelAtLast.y(),
        _trunkAxisAccAtLast.y());
  point("trunk_axis_y", halfPeriod,
        _params.trunkPitch);
  point("trunk_axis_z", 0.0,
        _trunkAxisPosAtLast.z(),
        _trunkAxisVelAtLast.z(),
        _trunkAxisAccAtLast.z());
  point("trunk_axis_z", halfPeriod,
        0.0);
}

void QuinticWalk::resetTrunkLastState() {
  if (_footstep.isLeftSupport()) {
    _trunkPosAtLast <<
                    _params.trunkXOffset,
        -_params.footDistance/2.0 + _params.trunkYOffset,
        _params.trunkHeight;
  } else {
    _trunkPosAtLast <<
                    _params.trunkXOffset,
        _params.footDistance/2.0 + _params.trunkYOffset,
        _params.trunkHeight;
  }
  _trunkVelAtLast.setZero();
  _trunkAccAtLast.setZero();
  _trunkAxisPosAtLast << 0.0, _params.trunkPitch, 0.0;
  _trunkAxisVelAtLast.setZero();
  _trunkAxisAccAtLast.setZero();
}

void QuinticWalk::computeCartesianPosition(Eigen::Vector3d &trunkPos,
                                           Eigen::Vector3d &trunkAxis,
                                           Eigen::Vector3d &footPos,
                                           Eigen::Vector3d &footAxis,
                                           bool &isLeftsupportFoot) {
  //Compute trajectories time
  double time = getTrajsTime();

  computeCartesianPositionAtTime(trunkPos, trunkAxis, footPos, footAxis, isLeftsupportFoot, time);

}

void QuinticWalk::computeCartesianPositionAtTime(Eigen::Vector3d &trunkPos,
                                                 Eigen::Vector3d &trunkAxis,
                                                 Eigen::Vector3d &footPos,
                                                 Eigen::Vector3d &footAxis,
                                                 bool &isLeftsupportFoot,
                                                 double time) {
  //Evaluate target cartesian
  //state from trajectories
  bool isDoubleSupport;
  trajectoriesTrunkFootPos(time, _trajs, trunkPos, trunkAxis, footPos, footAxis);
  trajectoriesSupportFootState(time, _trajs, isDoubleSupport, isLeftsupportFoot);
}

void QuinticWalk::point(std::string spline, double t, double pos, double vel, double acc) {
  _trajs.get(spline).addPoint(t, pos, vel, acc);
}

double QuinticWalk::getPhase() const {
  return _phase;
}

double QuinticWalk::getTrajsTime() const {
  double t;
  if (_phase < 0.5) {
    t = _phase/_params.freq;
  } else {
    t = (_phase - 0.5)/_params.freq;
  }

  return t;
}

Footstep QuinticWalk::getFootstep() {
  return _footstep;
}

bool QuinticWalk::isLeftSupport() {
  return _footstep.isLeftSupport();
}

bool QuinticWalk::isDoubleSupport() {
  // returns true if the value of the "is_double_support" spline is currently higher than 0.5
  // the spline should only have values of 0 or 1
  return _trajs.get("is_double_support").pos(getTrajsTime()) >= 0.5;
}

void QuinticWalk::reconf_callback(const bitbots_quintic_walk_paramsConfig &params) {
  _params = params;
  _footstep.setFootDistance(_params.footDistance);
}

void QuinticWalk::requestKick(bool left) {
  if (left) {
    _leftKickRequested = true;
  } else {
    _rightKickRequested = true;
  }
}

void QuinticWalk::requestPause() {
  _pauseRequested = true;
}

std::string QuinticWalk::getState() {
  return _engineState;
}

}

