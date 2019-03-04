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
    _trajs()
{   
    //Initialize the footstep
    _footstep.setFootDistance(_params.footDistance);
    _footstep.reset(false);
    //Reset the trunk saved state
    resetTrunkLastState();
    reset();
    _trajs = bitbots_splines::TrajectoriesInit();

}


bool QuinticWalk::updateState(double dt, const Eigen::Vector3d& orders, bool walkableState){
    bool ordersZero = orders[0] == 0.0 && orders[1] == 0.0 && orders[2] == 0.0;
        
    // check if we will finish a half step with this update
    bool halfStepFinished = (_lastPhase < 0.5 && _phase >= 0.5) || (_lastPhase > 0.5 && _phase < 0.5); 
    _lastPhase = _phase;

    if(halfStepFinished){
        ROS_WARN("halfstep_finished");
    }

    // small state machine
    if (_engineState == "idle") {
        if (ordersZero) {
            // we are in idle and are not supposed to walk. current state is fine, just do nothing
            return false;
        } else {
            // we should start walking if the robot is in the right state
            if (walkableState) {
                _phase = 0.0;
                buildStartTrajectories(orders);
                _engineState = "startMovement";
                updatePhase(dt);
            }else{
                // we can't start walking
                return false;
            }
        }
    } else if (_engineState == "startMovement") {
        // in this state we do a single "step" where we only move the trunk
        if (halfStepFinished) {
            //start step is finished, go to next state
            //_phase = 0.0;
            buildNormalTrajectories(orders);
            _engineState = "walking";
        }
        updatePhase(dt);
    } else if (_engineState == "walking") {
        // check if a half step was finished and we are unstable
        if (halfStepFinished && _pauseRequested){
            // go into pause
            _engineState = "paused";
        }else if(halfStepFinished &&
                (_leftKickRequested && !_footstep.isLeftSupport() || _rightKickRequested && _footstep.isLeftSupport())){
            // lets do a kick
            buildKickTrajectories(orders);
            _engineState = "kick";
            updatePhase(dt);
        }else if (halfStepFinished) {
            // current step is finished, lets see if we have to change state
            if (ordersZero) {
                // we have zero command vel -> we should stop
                _engineState = "stopStep";
                //_phase = 0.0;
                buildStopStepTrajectories(orders);
                updatePhase(dt);
            } else {
                // we can keep on walking
                //_phase = 0;
                buildNormalTrajectories(orders);
                updatePhase(dt);
            }
        }else{
            updatePhase(dt);
        }
    } else if (_engineState == "paused") {
        if (_timePaused > _params.pauseDuration) {
            // our pause is finished, whe can continue walking
            _engineState = "walking";
            _timePaused = 0.0;
        } else {
            _timePaused += dt;
        }
    } else if (_engineState == "kick") {
        // in this state we do a kick while doing a step
        if (halfStepFinished) {
            //kick step is finished, go on walking
            _phase = 0.0;
            _engineState = "walking";
            buildNormalTrajectories(orders);
        } else {
            updatePhase(dt);
        }
    } else if (_engineState == "stopStep") {
        // in this state we do a step back to get feet into idle pose
        if (halfStepFinished) {
            //stop step is finished, go to stop movement state
            _phase = 0.0;
            _engineState = "stopMovement";
            buildStopMovementTrajectories(orders);
        } else {
            updatePhase(dt);
        }
    }else if (_engineState == "stopMovement"){
        // in this state we do a "step" where we move the trunk back to idle position
        if (halfStepFinished) {
            //stop movement is finished, go to idle state
            _phase = 0.0;
            _engineState = "idle";
        }else{
            updatePhase(dt);
        }
    } else {
        ROS_ERROR("Somethings wrong with the walking engine state");
    }

    //Check support foot state
    if ((_lastPhase < 0.5 && !_footstep.isLeftSupport()) ||
        (_lastPhase >= 0.5 && _footstep.isLeftSupport())) {
        ROS_ERROR("QuinticWalk exception invalid state phase= %f support= %d dt= %f", _phase, _footstep.isLeftSupport(), dt);
        return false;
    }

    return true;
}

void QuinticWalk::updatePhase(double dt)
{
    //Check for negative time step
    if (dt <= 0.0) {
        if (dt == 0.0){ //sometimes happens due to rounding
            dt = 0.0001;
        }else{
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
    if(_phase > 1.0){
        _phase = 0.0;
    }
}

void QuinticWalk::endStep(){
    // ends the step earlier, e.g. when foot has already contact to ground
    if(_phase < 0.5){
        _phase = 0.5;
    }else{
        _phase = 1.0;
    }
}

void QuinticWalk::reset(){
    // completly reset the engine, e.g. when robot fell down
    _engineState = "idle";
    _phase = 0.0;
    _timePaused = 0.0;
}

void QuinticWalk::saveCurrentTrunkState(){
    //Evaluate current trunk state
    //(position, velocity, acceleration)
    //in next support foot frame
    double halfPeriod = 1.0/(2.0*_params.freq);
    Eigen::Vector2d trunkPos(
        _trajs.get("trunk_pos_x").pos(halfPeriod),
        _trajs.get("trunk_pos_y").pos(halfPeriod));
    Eigen::Vector2d trunkVel(
        _trajs.get("trunk_pos_x").vel(halfPeriod),
        _trajs.get("trunk_pos_y").vel(halfPeriod));
    Eigen::Vector2d trunkAcc(
        _trajs.get("trunk_pos_x").acc(halfPeriod),
        _trajs.get("trunk_pos_y").acc(halfPeriod));
    //Convert in next support foot frame
    trunkPos.x() -= _footstep.getNext().x();
    trunkPos.y() -= _footstep.getNext().y();
    trunkPos = Eigen::Rotation2Dd(
        -_footstep.getNext().z()).toRotationMatrix()
        *trunkPos;
    trunkVel = Eigen::Rotation2Dd(
        -_footstep.getNext().z()).toRotationMatrix()
        *trunkVel;
    trunkAcc = Eigen::Rotation2Dd(
        -_footstep.getNext().z()).toRotationMatrix()
        *trunkAcc;
    //Save state
    _trunkPosAtLast.x() = trunkPos.x();
    _trunkPosAtLast.y() = trunkPos.y();
    _trunkVelAtLast.x() = trunkVel.x();
    _trunkVelAtLast.y() = trunkVel.y();
    _trunkAccAtLast.x() = trunkAcc.x();
    _trunkAccAtLast.y() = trunkAcc.y();
    //No transformation for height
    _trunkPosAtLast.z() = _trajs.get("trunk_pos_z").pos(halfPeriod);
    _trunkVelAtLast.z() = _trajs.get("trunk_pos_z").vel(halfPeriod);
    _trunkAccAtLast.z() = _trajs.get("trunk_pos_z").acc(halfPeriod);
    //Evaluate and save trunk orientation
    //in next support foot frame
    Eigen::Vector3d trunkAxis(
        _trajs.get("trunk_axis_x").pos(halfPeriod),
        _trajs.get("trunk_axis_y").pos(halfPeriod),
        _trajs.get("trunk_axis_z").pos(halfPeriod));
    //Convert in intrinsic euler angle
    Eigen::Matrix3d trunkMat = bitbots_splines::AxisToMatrix(trunkAxis);
    Eigen::Vector3d trunkEuler = bitbots_splines::MatrixToEulerIntrinsic(trunkMat);
    //Transform to next support foot
    trunkEuler.z() -= _footstep.getNext().z();
    //Reconvert to axis and save it
    trunkMat = bitbots_splines::EulerIntrinsicToMatrix(trunkEuler);
    trunkAxis = bitbots_splines::MatrixToAxis(trunkMat);
    _trunkAxisPosAtLast = trunkAxis;
    //Evaluate trunk orientation velocity
    //and acceleration without frame 
    //transformation
    _trunkAxisVelAtLast.x() = _trajs.get("trunk_axis_x").vel(halfPeriod);
    _trunkAxisVelAtLast.y() = _trajs.get("trunk_axis_y").vel(halfPeriod);
    _trunkAxisVelAtLast.z() = _trajs.get("trunk_axis_z").vel(halfPeriod);
    _trunkAxisAccAtLast.x() = _trajs.get("trunk_axis_x").acc(halfPeriod);
    _trunkAxisAccAtLast.y() = _trajs.get("trunk_axis_y").acc(halfPeriod);
    _trunkAxisAccAtLast.z() = _trajs.get("trunk_axis_z").acc(halfPeriod);
}


void QuinticWalk::buildNormalTrajectories(const Eigen::Vector3d& orders){
    buildTrajectories(orders, false, false);
}

void QuinticWalk::buildKickTrajectories(const Eigen::Vector3d& orders){
    buildTrajectories(orders, false, true);
}

void QuinticWalk::buildStartTrajectories(const Eigen::Vector3d& orders){
    buildTrajectories(orders, true, false);
}

void QuinticWalk::buildStopStepTrajectories(const Eigen::Vector3d& orders){
    buildWalkDisableTrajectories(orders, false);
}

void QuinticWalk::buildStopMovementTrajectories(const Eigen::Vector3d& orders){
    buildWalkDisableTrajectories(orders, true);
}


void QuinticWalk::buildTrajectories(const Eigen::Vector3d& orders, bool startStep, bool kickStep)
{
    // save the current trunk state to use it later
    saveCurrentTrunkState();
    // update support foot and compute odometry
    _footstep.stepFromOrders(orders);

    //Reset the trajectories
    _trajs = bitbots_splines::TrajectoriesInit();
    //Set up the trajectories for the half cycle (single step)
    double halfPeriod = 1.0 / (2.0 * _params.freq);
    // full period (double step) is needed for trunk splines
    double period = 2.0 * halfPeriod;

    //Time length of double and single support phase during the half cycle
    double doubleSupportLength = _params.doubleSupportRatio * halfPeriod;
    double singleSupportLength = halfPeriod-doubleSupportLength;

    //Sign of support foot with respect to lateral
    double supportSign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

    //The trunk trajectory is defined for a
    //complete cycle to handle trunk phase shift
    //Trunk phase shift is done due to the length of the double 
    //support phase and can be adjusted optionally by a parameter
    // 0.5halfPeriod to be acyclic to the feet, 0.5doubleSupportLength to keep the double support phase centered between feet
    double timeShift = -0.5 * halfPeriod + 0.5 * doubleSupportLength + _params.trunkPhase * halfPeriod; 


    //Only move the trunk on the first half cycle after a walk enable
    if (startStep) {
        doubleSupportLength = halfPeriod;
        singleSupportLength = 0.0;
    }
    //Set double support phase
    point("is_double_support",    0.0,                   1.0);
    point("is_double_support",    doubleSupportLength,   1.0);
    point("is_double_support",    doubleSupportLength,   0.0);
    point("is_double_support",    halfPeriod,            0.0);

    //Set support foot
    point("is_left_support_foot", 0.0,            _footstep.isLeftSupport());
    point("is_left_support_foot", halfPeriod,     _footstep.isLeftSupport());

    //Flying foot position
    point("foot_pos_x",  0.0,                   _footstep.getLast().x());
    point("foot_pos_x",  doubleSupportLength,   _footstep.getLast().x());
    if(kickStep){
        point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.kickPhase,
        _footstep.getNext().x() + _params.kickLength);
    }
    point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase * _params.footOvershootPhase, 
        _footstep.getNext().x() + (_footstep.getNext().x()-_footstep.getLast().x()) *_params.footOvershootRatio);
    point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, 
        _footstep.getNext().x());
    point("foot_pos_x", halfPeriod, _footstep.getNext().x());

    point("foot_pos_y", 0.0,                 _footstep.getLast().y());
    point("foot_pos_y", doubleSupportLength, _footstep.getLast().y());
    point("foot_pos_y", doubleSupportLength + singleSupportLength *_params.footPutDownPhase *_params.footOvershootPhase, 
        _footstep.getNext().y() + (_footstep.getNext().y()-_footstep.getLast().y()) *_params.footOvershootRatio);
    point("foot_pos_y", doubleSupportLength + singleSupportLength *_params.footPutDownPhase, 
        _footstep.getNext().y());
    point("foot_pos_y", halfPeriod,          _footstep.getNext().y());

    point("foot_pos_z", 0.0,  0.0);
    point("foot_pos_z", doubleSupportLength, 0.0);
    point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footApexPhase - 0.5 * _params.footZPause * singleSupportLength, 
        _params.footRise);
    point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footApexPhase + 0.5 * _params.footZPause * singleSupportLength, 
        _params.footRise);
    point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footPutDownPhase,
        _params.footPutDownZOffset);
    point("foot_pos_z", halfPeriod, 0.0);

    //Flying foot orientation
    point("foot_axis_x", 0.0, 0.0);
    point("foot_axis_x", doubleSupportLength + 0.1 * singleSupportLength, 
        0.0);
    point("foot_axis_x", doubleSupportLength + singleSupportLength *  _params.footPutDownPhase, 
        _params.footPutDownRollOffset * supportSign);
    point("foot_axis_x", halfPeriod, 0.0);

    point("foot_axis_y", 0.0, 0.0);
    point("foot_axis_y", halfPeriod, 0.0);

    point("foot_axis_z", 0.0, _footstep.getLast().z());
    point("foot_axis_z", doubleSupportLength, _footstep.getLast().z());
    point("foot_axis_z", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, 
        _footstep.getNext().z());    
    point("foot_axis_z", halfPeriod, _footstep.getNext().z());


    //Half pause length of trunk swing 
    //lateral oscillation
    double pauseLength = 0.5*_params.trunkPause*halfPeriod;

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
        trunkPointSupport-trunkPointMiddle;
    //Apply swing amplitude ratio
    trunkVect.y() *= _params.trunkSwing;
    //Trunk support and next apex position
    Eigen::Vector2d trunkApexSupport = 
        trunkPointMiddle + trunkVect;
    Eigen::Vector2d trunkApexNext = 
        trunkPointMiddle - trunkVect;
    //Trunk forward velocity
    double trunkVelSupport = 
        (_footstep.getNext().x()-_footstep.getLast().x())/period;
    double trunkVelNext = 
        _footstep.getNext().x()/halfPeriod;

    //Trunk position
    point("trunk_pos_x", 0.0, 
        _trunkPosAtLast.x(), 
        _trunkVelAtLast.x(), 
        _trunkAccAtLast.x());
    point("trunk_pos_x", halfPeriod+timeShift, 
        trunkApexSupport.x(), 
        trunkVelSupport);
    point("trunk_pos_x", period+timeShift, 
        trunkApexNext.x(), 
        trunkVelNext);

    point("trunk_pos_y", 0.0, 
        _trunkPosAtLast.y(), 
        _trunkVelAtLast.y(), 
        _trunkAccAtLast.y());
    point("trunk_pos_y", halfPeriod+timeShift-pauseLength, 
        trunkApexSupport.y());
    point("trunk_pos_y", halfPeriod+timeShift+pauseLength, 
        trunkApexSupport.y());
    point("trunk_pos_y", period+timeShift-pauseLength, 
        trunkApexNext.y());
    point("trunk_pos_y", period+timeShift+pauseLength, 
        trunkApexNext.y());

    point("trunk_pos_z", 0.0, 
        _trunkPosAtLast.z(), 
        _trunkVelAtLast.z(), 
        _trunkAccAtLast.z());
    point("trunk_pos_z", halfPeriod+timeShift, 
        _params.trunkHeight);
    point("trunk_pos_z", period+timeShift, 
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
        0.5*_footstep.getLast().z()+0.5*_footstep.getNext().z());
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
    point("trunk_axis_x", halfPeriod+timeShift, 
        axisAtSupport.x(),
        axisVel.x());
    point("trunk_axis_x", period+timeShift, 
        axisAtNext.x(),
        axisVel.x());

    point("trunk_axis_y", 0.0, 
        _trunkAxisPosAtLast.y(),
        _trunkAxisVelAtLast.y(),
        _trunkAxisAccAtLast.y());
    point("trunk_axis_y", halfPeriod+timeShift, 
        axisAtSupport.y(),
        axisVel.y());
    point("trunk_axis_y", period+timeShift, 
        axisAtNext.y(),
        axisVel.y());

    point("trunk_axis_z", 0.0, 
        _trunkAxisPosAtLast.z(),
        _trunkAxisVelAtLast.z(),
        _trunkAxisAccAtLast.z());
    point("trunk_axis_z", halfPeriod+timeShift, 
        axisAtSupport.z(),
        axisVel.z());
    point("trunk_axis_z", period+timeShift, 
        axisAtNext.z(),
        axisVel.z());
}
        
void QuinticWalk::buildWalkDisableTrajectories(const Eigen::Vector3d& orders, bool footInIdlePosition){
    // save the current trunk state to use it later
    saveCurrentTrunkState();
    // update support foot and compute odometry
    _footstep.stepFromOrders(orders);

    //Reset the trajectories
    _trajs = bitbots_splines::TrajectoriesInit();

    //Set up the trajectories 
    //for the half cycle
    double halfPeriod = 1.0 / (2.0 * _params.freq);
    double period = 2.0 * halfPeriod;

    //Time length of double and single 
    //support phase during the half cycle
    double doubleSupportLength = _params.doubleSupportRatio * halfPeriod;
    double singleSupportLength = halfPeriod-doubleSupportLength;

    //Sign of support foot with 
    //respect to lateral
    double supportSign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

    //Set double support phase
    double isDoubleSupport = (footInIdlePosition ? 1.0 : 0.0);
    point("is_double_support", 0.0,         isDoubleSupport);
    point("is_double_support", halfPeriod,  isDoubleSupport);

    //Set support foot
    point("is_left_support_foot", 0.0,      _footstep.isLeftSupport());
    point("is_left_support_foot", halfPeriod, _footstep.isLeftSupport());

    //Flying foot position
    point("foot_pos_x", 0.0, 
        _footstep.getLast().x());
    point("foot_pos_x", doubleSupportLength, 
        _footstep.getLast().x());
    point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase * _params.footOvershootPhase, 
        0.0 + (0.0-_footstep.getLast().x()) * _params.footOvershootRatio);
    point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, 
        0.0);
    point("foot_pos_x", halfPeriod, 
        0.0);

    point("foot_pos_y", 0.0, 
        _footstep.getLast().y());
    point("foot_pos_y", doubleSupportLength, 
        _footstep.getLast().y());
    point("foot_pos_y", doubleSupportLength + singleSupportLength * _params.footPutDownPhase *_params.footOvershootPhase, 
        -supportSign*_params.footDistance + (-supportSign*_params.footDistance-_footstep.getLast().y()) *_params.footOvershootRatio);
    point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, 
        -supportSign*_params.footDistance);
    point("foot_pos_y", halfPeriod, 
        -supportSign*_params.footDistance);

    //If the walk has just been disabled,
    //make one single step to neutral pose
    if (!footInIdlePosition) {
        point("foot_pos_z", 0.0, 0.0);
        point("foot_pos_z", doubleSupportLength, 0.0);
        point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footApexPhase - 0.5 * _params.footZPause * singleSupportLength, 
            _params.footRise);
        point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footApexPhase + 0.5 * _params.footZPause * singleSupportLength, 
            _params.footRise);
        point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footPutDownPhase,
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
    point("foot_axis_z", doubleSupportLength + singleSupportLength * _params.footPutDownPhase,
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
    return;    
}

void QuinticWalk::resetTrunkLastState()
{
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

void QuinticWalk::computeCartesianPosition(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis, Eigen::Vector3d& footPos,
                                           Eigen::Vector3d& footAxis, bool& isLeftsupportFoot)
{
    //Compute trajectories time
    double time = getTrajsTime();

    computeCartesianPositionAtTime(trunkPos, trunkAxis, footPos, footAxis, isLeftsupportFoot, time);

}


void QuinticWalk::computeCartesianPositionAtTime(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis, Eigen::Vector3d& footPos,
                                                 Eigen::Vector3d& footAxis, bool& isLeftsupportFoot, double time)
{
    //Evaluate target cartesian
    //state from trajectories
    bool isDoubleSupport;
    TrajectoriesTrunkFootPos(time, _trajs, trunkPos, trunkAxis, footPos, footAxis);
    TrajectoriesSupportFootState(time, _trajs, isDoubleSupport, isLeftsupportFoot);
}

void QuinticWalk::point(std::string spline, double t, double pos, double vel, double acc){
    _trajs.get(spline).addPoint(t, pos, vel, acc);
}

void QuinticWalk::saveSplineCsv(const std::string& filename){
    _trajs.exportData(filename);
}

double QuinticWalk::getPhase() const
{
    return _phase;
}

double QuinticWalk::getTrajsTime() const
{
    double t;
    if (_phase < 0.5) {
        t = _phase/_params.freq;
    } else {
        t = (_phase - 0.5)/_params.freq;
    }

    return t;
}

const WalkingParameter& QuinticWalk::getParameters() const
{
    return _params;
}

Footstep QuinticWalk::getFootstep(){
    return _footstep;
}

const bitbots_splines::Trajectories& QuinticWalk::getTrajectories() const
{
    return _trajs;
}

bool QuinticWalk::isLeftSupport(){
    return _footstep.isLeftSupport();
}

bool QuinticWalk::isDoubleSupport(){
    // returns true if the value of the "is_double_support" spline is currently higher than 0.5
    // the spline should only have values of 0 or 1
    return _trajs.get("is_double_support").pos(getTrajsTime()) >= 0.5 ? true : false;
}

void QuinticWalk::setParameters(const WalkingParameter& params)
{
    _params = params;
    _footstep.setFootDistance(_params.footDistance);
}

void QuinticWalk::requestKick(bool left){
    if(left){
        _leftKickRequested = true;
    }else{
        _rightKickRequested = true;
    }
}

void QuinticWalk::requestPause(){
    _pauseRequested = true;
}


std::string QuinticWalk::getState(){
    return _engineState;
}

}

