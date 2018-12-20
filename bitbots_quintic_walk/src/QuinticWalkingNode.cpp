#include "bitbots_quintic_walk/QuinticWalkingNode.hpp"


QuinticWalkingNode::QuinticWalkingNode(){        
    // init variables
    _robotState = humanoid_league_msgs::RobotControlState::CONTROLABLE;
    _walkEngine = bitbots_quintic_walk::QuinticWalk();
    _stopRequest = true;    
    _imu_stop = false;
    walkingReset();
    _isLeftSupport = true;
    _supportFootOdom = tf::Transform();
    // this is important since rviz will crash without explicit initilization of the transform
    tf::Quaternion quat = tf::Quaternion();
    quat.setRPY(0,0,0);
    _supportFootOdom.setRotation(quat);   
    _supportFootOdom.setOrigin(tf::Vector3(0,0,0));

    _marker_id = 1;
    _odom_broadcaster = tf::TransformBroadcaster();
    
    // read config
    _nh.param<double>("engineFrequency", _engineFrequency, 100.0);
    _nh.param<std::string>("walking/ik_type", _ik_type, "bio_ik");
    _nh.param<std::string>("/robot_type_name", _robot_type, "notDefined");
    _nh.param<bool>("/simulation_active", _simulation_active, false);

    /* init publisher and subscriber */
    _joint_state_msg = sensor_msgs::JointState();
    _pubModelJointState = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    _command_msg = bitbots_msgs::JointCommand();
    _pubControllerCommand = _nh.advertise<bitbots_msgs::JointCommand>("walking_motor_goals", 1);

    _odom_msg = nav_msgs::Odometry();
    _pubOdometry = _nh.advertise<nav_msgs::Odometry>("walk_odometry", 1);
    _subCmdVel = _nh.subscribe("cmd_vel", 1, &QuinticWalkingNode::cmdVelCb, this, ros::TransportHints().tcpNoDelay());
    _subRobState = _nh.subscribe("robot_state", 1, &QuinticWalkingNode::robStateCb, this, ros::TransportHints().tcpNoDelay());
    _subJointStates = _nh.subscribe("joint_states", 1, &QuinticWalkingNode::jointStateCb, this, ros::TransportHints().tcpNoDelay());
    _subKick = _nh.subscribe("kick", 1, &QuinticWalkingNode::kickCb, this, ros::TransportHints().tcpNoDelay());
    _subImu = _nh.subscribe("imu/data", 1, &QuinticWalkingNode::imuCb, this, ros::TransportHints().tcpNoDelay());
    _subPressure = _nh.subscribe("pressure", 1, &QuinticWalkingNode::pressureCb, this, ros::TransportHints().tcpNoDelay());

    /* debug publisher */
    _pubDebug = _nh.advertise<bitbots_quintic_walk::WalkingDebug>("walk_debug", 1);
    _pubDebugMarker = _nh.advertise<visualization_msgs::Marker>("walk_debug_marker",1);
    
    //load MoveIt! model    
    _robot_model_loader = robot_model_loader::RobotModelLoader("/robot_description", false);
    _robot_model_loader.loadKinematicsSolvers(
            kinematics_plugin_loader::KinematicsPluginLoaderPtr(
                    new kinematics_plugin_loader::KinematicsPluginLoader()));
    _kinematic_model = _robot_model_loader.getModel();
    _all_joints_group = _kinematic_model->getJointModelGroup("All");
    _legs_joints_group = _kinematic_model->getJointModelGroup("Legs");
    _lleg_joints_group = _kinematic_model->getJointModelGroup("LeftLeg");
    _rleg_joints_group = _kinematic_model->getJointModelGroup("RightLeg");
    _goal_state.reset( new robot_state::RobotState( _kinematic_model ));
    _goal_state->setToDefaultValues();
    _current_state.reset(new robot_state::RobotState( _kinematic_model ));
    _current_state->setToDefaultValues();

    // initilize IK solver
    _bioIK_solver = bitbots_ik::BioIKSolver(*_all_joints_group, *_lleg_joints_group, *_rleg_joints_group);    

    // gravity compensator
    _gravity_compensator = bitbots_quintic_walk::GravityCompensator(_kinematic_model);
}


void QuinticWalkingNode::run(){
    /* 
    This is the main loop which takes care of stopping and starting of the walking.
    If the walking is active, computeWalking() is called to compute next motor goals.
    */

   int odom_counter = 0;

    while (ros::ok()){
        ros::Rate loopRate(_engineFrequency);
        if(_walkActive){
            // The robot is currently walking
            if((_robotState != humanoid_league_msgs::RobotControlState::FALLING)){
                // The robot is in the right state, let's compute next motor goals
                // First update orders. Tell robot to stop if it gets unstable
                if(_imuActive && _imu_stop){
                    _walkEngine.setOrders(_orders, !_stopRequest, true);    
                }else{
                    _walkEngine.setOrders(_orders, !_stopRequest, false);
                }
                
                // Calculate joint positions
                calculateWalking();
            }else{
                // The HCM changed from state walking to something else.
                // this means, we were forced by the HCM to do something else
                // for example standing up.
                // We reset the walking engine and stop
                walkingReset();
                ROS_WARN("reset");
            }
        }else{
            // We're not currently running, test if we want to start
            if(!_stopRequest && (_robotState == humanoid_league_msgs::RobotControlState::CONTROLABLE || _robotState == humanoid_league_msgs::RobotControlState::WALKING
                                 || _robotState == humanoid_league_msgs::RobotControlState::MOTOR_OFF)){
                _walkActive = true;
            }
        }
        odom_counter++;
        if (odom_counter > _odomPubFactor){
            publishOdometry();
            odom_counter = 0;
        }                
        ros::spinOnce();
        loopRate.sleep();
    }
}


void QuinticWalkingNode::walkingReset(){
    /* 
    Resets the walking and stops it *imediatly*. This means that it can also stop during a step, thus in an
    unstable position. Should be normally used when the robot is already falling.
    */
    _orders = {0.0, 0.0, 0.0};
    _walkEngine.setOrders(_orders, false, true);
    _walkActive = false;    
    _just_started = true;
}

void QuinticWalkingNode::calculateWalking(){
    /*
    This method computes the next motor goals as well as the odometry if the step was changed.
    */
   
    // save last step odometry if support foot changes
    _stepOdom = _walkEngine.getFootstep().getNext();     

    double dt = 0.01;
    if(!_simulation_active){
        std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
        // only take real time difference if walking was not stopped before
        // using c++ time since it is more performant than ros time. We only need a local difference, so it doesnt matter as long as we are not simulating
        if(! _just_started){        
            auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - _last_update_time);
            dt = time_diff_ms.count() / 1000.0;        
            if(dt == 0){
                ROS_WARN("dt was 0");
                dt = 0.001;
            }
        }
        _last_update_time = current_time;
    }else{
        ROS_WARN_ONCE("Simulation active, using ROS time");
        // use ros time for simulation
        double current_ros_time = ros::Time::now().toSec();
        if(! _just_started){ 
            dt = current_ros_time - _last_ros_update_time;
        }       
        _last_ros_update_time = current_ros_time;
    }
    _just_started = false;
    // compute new values from splines
    _walkEngine.update(dt);
    // read the positions and orientations for trunk and fly foot
    _walkEngine.computeCartesianPosition(_trunkPos, _trunkAxis, _footPos, _footAxis, _isLeftSupport);    
    
    // check if support foot has changed
    if(_isLeftSupport != _wasLeftSupport){
        _wasLeftSupport = _isLeftSupport;
        // add odometry change of last step to trunk odom if step was completed    
        // make transform
        /*tf::Transform step;
        step.setOrigin(tf::Vector3{_stepOdom[0], _stepOdom[1], 0.0});
        tf::Quaternion tf_quat = tf::Quaternion();
        tf_quat.setRPY(0, 0, _stepOdom[2]);
        step.setRotation(tf_quat);

        // transform global odometry
        _supportFootOdom = _supportFootOdom * step;*/

        //check if the walking came to a complete stop
        if((_stopRequest || _imu_stop) && !_walkEngine.getIsWalking()){            
            _walkActive = false;
            _just_started = true;
            // we came to a stop, if we stopped due to the imu, we can start again
            _imu_stop = false;            
        }
    }

    // change goals from support foot based coordinate system to trunk based coordinate system 
    tf::Vector3 tf_vec;
    tf::vectorEigenToTF(_trunkPos, tf_vec);
    tf::Quaternion tf_quat = tf::Quaternion();
    tf_quat.setRPY(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
    tf_quat.normalize();
    tf::Transform support_foot_to_trunk(tf_quat, tf_vec);
    tf::Transform trunk_to_support_foot_goal = support_foot_to_trunk.inverse();

    tf::vectorEigenToTF(_footPos, tf_vec);
    tf_quat.setRPY(_footAxis[0], _footAxis[1], _footAxis[2]);
    tf_quat.normalize();
    tf::Transform support_to_flying_foot(tf_quat, tf_vec);
    tf::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * support_to_flying_foot;

    // call ik solver    
    bool success = _bioIK_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal, _walkEngine.getFootstep().isLeftSupport(), _goal_state);


    // publish goals if sucessfull
    if(success){
        if(_compensate_gravity){
            compensateGravity();
        }

        std::vector<std::string> joint_names = _legs_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        _goal_state->copyJointGroupPositions(_legs_joints_group, joint_goals);
        publishControllerCommands(joint_names, joint_goals);
        if(_pub_model_joint_states){
            publishModelJointStates(joint_names, joint_goals);
        }
    }
    if(_debugActive){
        publishDebug(trunk_to_support_foot_goal, trunk_to_flying_foot_goal);
        publishMarkers();
    }
}

void QuinticWalkingNode::compensateGravity(){
    /**
     * This method changes the goal positions of the joints according to the computed torque that is introduced by the weight of the robot
     */
    // this double represents how the balance of the weight is between the feet
    // 0 if right foot is up, 1 if left foot is up
    _balance_left_right = _walkEngine.getWeightBalance();
    
    // update to prevent "dirty link transforms"
    _goal_state->update();
    // reset all efforts to 0
    for (size_t i = 0; i < _goal_state->getVariableCount(); i++) {            
          _goal_state->setVariableEffort(i, 0.0);
    }

    // compute the efforts
    _gravity_compensator.compensateGravity(
        *_goal_state,
        {{
            std::make_pair("l_foot", 1.0 - _balance_left_right),
            std::make_pair("r_foot", 0.0 + _balance_left_right),
        }});

    // set some joint efforts to 0 since we don't want to change them
    // gravity compensation should be done by the knee joints
    _goal_state->setVariableEffort("LAnkleRoll", 0);
    _goal_state->setVariableEffort("RAnkleRoll", 0);
    _goal_state->setVariableEffort("LAnklePitch", 0);
    _goal_state->setVariableEffort("RAnklePitch", 0);
    _goal_state->setVariableEffort("RHipYaw", 0);
    _goal_state->setVariableEffort("LHipYaw", 0);
    _goal_state->setVariableEffort("LHipRoll", 0);
    _goal_state->setVariableEffort("RHipRoll", 0);

    // make sure that we will not divide by 0
    if(_gravityP == 0){
        _gravityP = 1.0;
    }
    // change the goal position using inverse P controller    
    for (size_t i = 0; i < _goal_state->getVariableCount(); i++) {
        double newPosition= _goal_state->getVariablePosition(i) + _goal_state->getVariableEffort(i) / _gravityP;            
        _goal_state->setVariablePosition(i, newPosition);
    }
}

void QuinticWalkingNode::cmdVelCb(const geometry_msgs::Twist msg){
    // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
    // other axis. 
    // the engine ecepts orders in [m] not [m/s]. We have to compute by dividing by step frequency which is a double step
    double factor =  1.0/ (_params.freq);
    _orders = {msg.linear.x * factor, msg.linear.y * factor, msg.angular.z * factor};       
    // the orders should not extend beyond a maximal step size
    for(int i = 0; i < 3; i++){
        if (_orders[i] >= 0){
            _orders[i] = std::min(_orders[i], _max_step[i]);
        }else{
            _orders[i] = std::max(_orders[i], _max_step[i] * -1);
        }
    }
    // deactivate walking if goal is 0 movement, else activate it
    _stopRequest = (msg.linear.x == 0 && msg.linear.y == 0 && msg.angular.z == 0);
}

void QuinticWalkingNode::imuCb(const sensor_msgs::Imu msg){
    if(_imuActive) {
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

       if(abs(roll) > _imu_roll_threshold || abs(pitch) > _imu_pitch_threshold){
           _imu_stop = true;
       }
    }
}

void QuinticWalkingNode::pressureCb(const bitbots_msgs::FootPressure msg){

    // we only want to check stuff if we are in single support
    if(_walkEngine.isEnabled() && !_walkEngine.isDoubleSupport()){

        // we just want to look at the support foot. choose the 4 values from the message accordingly
        // s = support, n = not support, i = inside, o = outside, f = front, b = back
        double sob;
        double sof;
        double sif;
        double sib;

        double nob;
        double nof;
        double nif;
        double nib;

        if(_walkEngine.isLeftSupport()){
            sob = msg.l_l_b;
            sof = msg.l_l_f;
            sif = msg.l_r_f;
            sib = msg.l_r_b;

            nib = msg.r_l_b;
            nif = msg.r_l_f;
            nof = msg.r_r_f;
            nob = msg.r_r_b;
        }else{
            sib = msg.r_l_b;
            sif = msg.r_l_f;
            sof = msg.r_r_f;
            sob = msg.r_r_b;

            nob = msg.l_l_b;
            nof = msg.l_l_f;
            nif = msg.l_r_f;
            nib = msg.l_r_b;
        }

        // sum to get overall pressure on foot
        double s_sum = slb + slf + srf + srb;
        double n_sum = nlb + nlf + nrf + nrb;

        // ratios between pressures to get relative position of CoP
        double s_io_ratio = (sif + sib) / (sof + sob);
        double s_fb_ratio = (sif + sof) / (sib + sob);

        // check for phase reset
        // phase is far enough to have right foot lifted
        // foot has to have ground contact
        if(_walkEngine.getPhase() > 0.7 && n_sum > 1){
            _walkEngine.resetPhase();
        }

        // check if robot is unstable and should pause
        // this is true if the robot is falling to the outside or to front or back
        if(s_io_ratio < 0.5 || s_fb_ratio < 0.5 || s_fb_ratio > 2){
            //TODO pause
       }
    }
}

void QuinticWalkingNode::robStateCb(const humanoid_league_msgs::RobotControlState msg){    
    _robotState = msg.state;
}

void QuinticWalkingNode::jointStateCb(const sensor_msgs::JointState msg){
    std::vector<std::string> names_vec = msg.name;
    std::string* names = names_vec.data();    

    _current_state->setJointPositions(*names, msg.position.data());
    //todo
    //_current_state->setJointVelocities(*names, msg.velocity.data());
    //_current_state->setJointEfforts(*names, msg.effort.data());
}

void QuinticWalkingNode::kickCb(const std_msgs::BoolConstPtr msg){
    _kick = true;
}

void QuinticWalkingNode::reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level) {
    _params.freq =  config.freq;
    _params.doubleSupportRatio =  config.doubleSupportRatio;
    _params.footDistance =  config.footDistance;
    _params.footRise =  config.footRise;
    _params.footZPause = config.footZPause;
    _params.footPutDownZOffset =  config.footPutDownZOffset;
    _params.footPutDownPhase =  config.footPutDownPhase;
    _params.footApexPhase =  config.footApexPhase;
    _params.footOvershootRatio =  config.footOvershootRatio;
    _params.footOvershootPhase =  config.footOvershootPhase;
    _params.trunkHeight =  config.trunkHeight;
    _params.trunkPitch =  config.trunkPitch;
    _params.trunkPhase =  config.trunkPhase;
    _params.trunkXOffset =  config.trunkXOffset;
    _params.trunkYOffset =  config.trunkYOffset;
    _params.trunkSwing =  config.trunkSwing;
    _params.trunkPause =  config.trunkPause;
    _params.trunkXOffsetPCoefForward =  config.trunkXOffsetPCoefForward;
    _params.trunkXOffsetPCoefTurn =  config.trunkXOffsetPCoefTurn;
    _params.trunkPitchPCoefForward =  config.trunkPitchPCoefForward;
    _params.trunkPitchPCoefTurn =  config.trunkPitchPCoefTurn;
    _params.trunkYOnlyInDoubleSupport =  config.trunkYOnlyInDoubleSupport;
    _params.kickLength = config.kickLength;
    _params.kickPhase = config.kickPhase;
    _params.footPutDownRollOffset = config.footPutDownRollOffset;
    _params.kickVel = config.kickVel;
    _walkEngine.setParameters(_params);    
    _bioIK_solver.set_bioIK_timeout(config.bioIKTime);
    _bioIK_solver.set_use_approximate(config.bioIKApprox);
    
    _debugActive = config.debugActive;
    _pub_model_joint_states = config.pubModelJointStates;
    _engineFrequency = config.engineFreq;
    _odomPubFactor = config.odomPubFactor;
    _compensate_gravity = config.compensateGravity;
    _gravityP = config.gravityP;
    _max_step[0] = config.maxStepX;
    _max_step[1] = config.maxStepY;
    _max_step[2] = config.maxStepZ;
    _vel = config.vel;
    _acc = config.acc;
    _pwm = config.pwm;
    _imuActive = config.imuActive;
    _imu_pitch_threshold = config.imuPitchThreshold;
    _imu_roll_threshold = config.imuRollThreshold;
}


void QuinticWalkingNode::publishModelJointStates(std::vector <std::string> joint_names, std::vector<double> positions){
    _joint_state_msg.position = positions;
    _joint_state_msg.name = joint_names;
    _joint_state_msg.header.stamp = ros::Time::now();
    _pubModelJointState.publish(_joint_state_msg);
}

void QuinticWalkingNode::publishControllerCommands(std::vector <std::string> joint_names, std::vector<double> positions){
    // publishes the commands to the GroupedPositionController
    _command_msg.joint_names = joint_names;
    _command_msg.positions = positions;
    std::vector<double> ones(joint_names.size(),-1.0);
    std::vector<double> vels(joint_names.size(), _vel);
    std::vector<double> accs(joint_names.size(), _acc);
    std::vector<double> pwms(joint_names.size(), _pwm);
    _command_msg.velocities = vels;
    _command_msg.accelerations = accs;
    _command_msg.max_currents = pwms;

    _pubControllerCommand.publish(_command_msg);
}

void QuinticWalkingNode::publishOdometry(){
    // transformation from support leg to trunk
    Eigen::Affine3d trunk_to_support;
    if(_walkEngine.getFootstep().isLeftSupport()){
        trunk_to_support = _goal_state->getGlobalLinkTransform("l_sole");
    }else{
        trunk_to_support = _goal_state->getGlobalLinkTransform("r_sole");
    }
    Eigen::Affine3d support_to_trunk = trunk_to_support.inverse();
    tf::Transform tf_support_to_trunk;
    tf::transformEigenToTF(support_to_trunk, tf_support_to_trunk);

    // odometry to trunk is transform to support foot * transform from support to trunk    
    double x;
    double y;
    double yaw;
    if(_walkEngine.getFootstep().isLeftSupport()){
        x = _walkEngine.getFootstep().getLeft()[0];
        y = _walkEngine.getFootstep().getLeft()[1];
        yaw = _walkEngine.getFootstep().getLeft()[2];
    }else{
        x = _walkEngine.getFootstep().getRight()[0];
        y = _walkEngine.getFootstep().getRight()[1];
        yaw = _walkEngine.getFootstep().getRight()[2];
    }

    tf::Transform supportFootTf;
    supportFootTf.setOrigin(tf::Vector3{x, y, 0.0});
    tf::Quaternion supportFootQuat = tf::Quaternion();
    supportFootQuat.setRPY(0, 0, yaw);
    supportFootTf.setRotation(supportFootQuat);
    tf::Transform odom_to_trunk = supportFootTf * tf_support_to_trunk;    
    tf::Vector3 pos = odom_to_trunk.getOrigin();
    geometry_msgs::Quaternion quat_msg;
    
    tf::quaternionTFToMsg(odom_to_trunk.getRotation().normalize(), quat_msg);    

    ros::Time current_time = ros::Time::now();
    _odom_trans = geometry_msgs::TransformStamped();
    _odom_trans.header.stamp = current_time;
    _odom_trans.header.frame_id = "odom";
    _odom_trans.child_frame_id = "base_link";

    _odom_trans.transform.translation.x = pos[0];
    _odom_trans.transform.translation.y = pos[1];
    _odom_trans.transform.translation.z = pos[2];
    _odom_trans.transform.rotation = quat_msg;

    //send the transform
    //todo this kills rviz
    _odom_broadcaster.sendTransform(_odom_trans);

    // send the odometry also as message
    _odom_msg.header.stamp = current_time;
    _odom_msg.header.frame_id = "odom";
    _odom_msg.child_frame_id = "base_link";
    _odom_msg.pose.pose.position.x = pos[0];
    _odom_msg.pose.pose.position.y = pos[1];
    _odom_msg.pose.pose.position.z = pos[2];    

    _odom_msg.pose.pose.orientation = quat_msg;
    geometry_msgs::Twist twist;
    twist.linear.x = _orders[0] * _params.freq;
    twist.linear.y = _orders[1] * _params.freq;
    twist.angular.z = _orders[2] * _params.freq;
    _odom_msg.twist.twist = twist;
    _pubOdometry.publish(_odom_msg);
}

void QuinticWalkingNode::publishDebug(tf::Transform& trunk_to_support_foot_goal, tf::Transform& trunk_to_flying_foot_goal) {
    /*
    This method publishes various debug / visualization information.
    */
    bitbots_quintic_walk::WalkingDebug msg;   
    bool is_left_support = _walkEngine.isLeftSupport();
    msg.is_left_support = is_left_support;
    msg.is_double_support = _walkEngine.isDoubleSupport();
    msg.header.stamp = ros::Time::now();

    // times
    msg.phase_time = _walkEngine.getPhase();    
    msg.traj_time = _walkEngine.getTrajsTime();    

    msg.balance_left_right = _balance_left_right;

    // engine output
    geometry_msgs::Pose pose_msg;
    tf::pointEigenToMsg(_footPos, pose_msg.position);
    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(_footAxis[0], _footAxis[1], _footAxis[2]);
    msg.engine_fly_goal = pose_msg;
    tf::pointEigenToMsg(_trunkPos, pose_msg.position);
    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
    msg.engine_trunk_goal = pose_msg;

    // goals
    geometry_msgs::Pose pose_support_foot_goal;
    tf::pointTFToMsg (trunk_to_support_foot_goal.getOrigin(), pose_support_foot_goal.position);
    tf::quaternionTFToMsg (trunk_to_support_foot_goal.getRotation(), pose_support_foot_goal.orientation);
    msg.support_foot_goal = pose_support_foot_goal;
    geometry_msgs::Pose pose_fly_foot_goal;
    tf::pointTFToMsg (trunk_to_flying_foot_goal.getOrigin(), pose_fly_foot_goal.position);
    tf::quaternionTFToMsg (trunk_to_flying_foot_goal.getRotation(), pose_fly_foot_goal.orientation);
    msg.fly_foot_goal = pose_fly_foot_goal;
    if(is_left_support){
        msg.left_foot_goal = pose_support_foot_goal;
        msg.right_foot_goal = pose_fly_foot_goal;
    }else{
        msg.left_foot_goal = pose_fly_foot_goal;
        msg.right_foot_goal = pose_support_foot_goal;        
    }

    // IK results     
    geometry_msgs::Pose pose_left_result;
    tf::poseEigenToMsg (_goal_state->getGlobalLinkTransform("l_sole"), pose_left_result);
    msg.left_foot_ik_result = pose_left_result;
    geometry_msgs::Pose pose_right_result;
    tf::poseEigenToMsg (_goal_state->getGlobalLinkTransform("r_sole"), pose_right_result);
    msg.right_foot_ik_result = pose_right_result;
    if(is_left_support){
        msg.support_foot_ik_result = pose_left_result;
        msg.fly_foot_ik_result = pose_right_result;
    }else{
        msg.support_foot_ik_result = pose_right_result;
        msg.fly_foot_ik_result = pose_left_result;
    }    

    // IK offsets
    tf::Vector3 support_off;
    tf::Vector3 fly_off;
    tf::Vector3 tf_vec_left;
    tf::Vector3 tf_vec_right;
    tf::vectorEigenToTF(_goal_state->getGlobalLinkTransform("l_sole").translation(), tf_vec_left);
    tf::vectorEigenToTF(_goal_state->getGlobalLinkTransform("r_sole").translation(), tf_vec_right);
    geometry_msgs::Vector3 vect_msg;
    if (is_left_support) {
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_right;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.left_foot_ik_offset = vect_msg;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.right_foot_ik_offset = vect_msg;
    }else{
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_left;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.left_foot_ik_offset = vect_msg;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.right_foot_ik_offset = vect_msg;
    }
    tf::vector3TFToMsg(support_off, vect_msg);
    msg.support_foot_ik_offset = vect_msg;
    tf::vector3TFToMsg(fly_off, vect_msg);
    msg.fly_foot_ik_offset = vect_msg;

    // actual positions
    geometry_msgs::Pose pose_left_actual;
    tf::poseEigenToMsg (_current_state->getGlobalLinkTransform("l_sole"), pose_left_actual);
    msg.left_foot_position = pose_left_actual;
    geometry_msgs::Pose pose_right_actual;
    tf::poseEigenToMsg (_current_state->getGlobalLinkTransform("r_sole"), pose_right_actual);
    msg.right_foot_position = pose_right_actual;
    if(is_left_support){
        msg.support_foot_position = pose_left_actual;
        msg.fly_foot_position = pose_right_actual;
    }else{
        msg.support_foot_position = pose_right_actual;
        msg.fly_foot_position = pose_left_actual;
    }    

    // actual offsets
    tf::vectorEigenToTF(_current_state->getGlobalLinkTransform("l_sole").translation(), tf_vec_left);
    tf::vectorEigenToTF(_current_state->getGlobalLinkTransform("r_sole").translation(), tf_vec_right);
    if (is_left_support) {
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_right;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.left_foot_actual_offset = vect_msg;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.right_foot_actual_offset = vect_msg;
    }else{
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_left;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.left_foot_actual_offset = vect_msg;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.right_foot_actual_offset = vect_msg;
    }
    tf::vector3TFToMsg(support_off, vect_msg);
    msg.support_foot_actual_offset = vect_msg;
    tf::vector3TFToMsg(fly_off, vect_msg);
    msg.fly_foot_actual_offset = vect_msg;

    _pubDebug.publish(msg);
}

void QuinticWalkingNode::publishMarkers(){
    //publish markers
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    if(_walkEngine.getFootstep().isLeftSupport()) {
        marker_msg.header.frame_id = "l_sole";
    }else{
        marker_msg.header.frame_id = "r_sole";
    }
    marker_msg.type = marker_msg.CUBE;
    marker_msg.action = 0;
    marker_msg.lifetime = ros::Duration(0.0);
    geometry_msgs::Vector3 scale;
    scale.x = 0.20;
    scale.y = 0.10;
    scale.z = 0.01;
    marker_msg.scale = scale;
    //last step
    marker_msg.ns = "last_step";
    marker_msg.id = 1;
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker_msg.color = color;
    geometry_msgs::Pose pose;
    Eigen::Vector3d step_pos = _walkEngine.getFootstep().getLast();
    geometry_msgs::Point point;
    point.x = step_pos[0];
    point.y = step_pos[1];
    point.z = 0;
    pose.position = point;
    pose.orientation = tf::createQuaternionMsgFromYaw(step_pos[2]);
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);

    //last step center
    marker_msg.ns = "step_center";
    marker_msg.id = _marker_id;
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;
    marker_msg.scale = scale;
    _pubDebugMarker.publish(marker_msg);

    // next step
    marker_msg.id = _marker_id ;
    marker_msg.ns = "next_step";
    scale.x = 0.20;
    scale.y = 0.10;
    scale.z = 0.01;
    marker_msg.scale = scale;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 0.5;
    marker_msg.color = color;
    step_pos = _walkEngine.getFootstep().getNext();
    point.x = step_pos[0];
    point.y = step_pos[1];
    pose.position = point;
    pose.orientation = tf::createQuaternionMsgFromYaw(step_pos[2]);
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);



    //trunk
    marker_msg.type = marker_msg.SPHERE;
    marker_msg.ns = "trunk";
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;
    if(_walkEngine.isDoubleSupport()){
        color.r = 0;
        color.g = 0;
        color.b = 1;
        color.a = 1;
    }else if(_walkEngine.isLeftSupport()){
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1;
    }else{
        color.r = 1;
        color.g = 1;
        color.b = 0;
        color.a = 1;
    }
    marker_msg.color = color;
    marker_msg.scale = scale;
    marker_msg.id = _marker_id; 
    marker_msg.header.frame_id = "base_link";
    point.x = 0;
    point.y = 0;
    point.z = 0;
    pose.position = point;
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);

    _marker_id++;
    if(_marker_id > 1000){
        _marker_id = 1;
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "quintic_walking");
    // init node
    QuinticWalkingNode node = QuinticWalkingNode();
    // set the dynamic reconfigure and load standard params
    dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig> server;
    dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>::CallbackType f;
    f = boost::bind(&QuinticWalkingNode::reconf_callback,&node, _1, _2);
    server.setCallback(f);
    // run the node
    node.run();
}

