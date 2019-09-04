#include "bitbots_ik/AnalyticIKSolver.hpp"
namespace bitbots_ik{

AnalyticIKSolver::AnalyticIKSolver(){

}

AnalyticIKSolver::AnalyticIKSolver(std::string robot_type, const robot_state::JointModelGroup &lleg_joints_group, const robot_state::JointModelGroup &rleg_joints_group){
    _robot_type = robot_type;
    _lleg_joints_group = &lleg_joints_group;
    _rleg_joints_group = &rleg_joints_group;
}

bool AnalyticIKSolver::solve(tf2::Transform& trunk_to_support_foot, tf2::Transform& trunk_to_flying_foot, bool is_left_support, robot_state::RobotStatePtr &robot_state_ptr) {
    std::vector<double> support_joints;
    std::vector<double> fly_joints;
    tf2::Transform goal;
    tf2::Quaternion tf_quat;
    tf2::Vector3 tf_vec;

    tf2::Vector3 trunk_to_LHipYaw;
    tf2::Vector3 trunk_to_RHipYaw;
    urdf::JointConstSharedPtr lHipYaw, rHipYaw;
    urdf::Model robot;
    //load URDF
    if(robot.initParam("/robot_description") == 0){
        ROS_ERROR("Loading URDF failed");
    }
    //get necessary values from URDF
    if(_robot_type == "Minibot" || _robot_type == "Wolfgang"){
        lHipYaw = robot.getJoint("LHipYaw");
        rHipYaw = robot.getJoint("RHipYaw");
        trunk_to_LHipYaw.setValue(lHipYaw->parent_to_joint_origin_transform.position.x, lHipYaw->parent_to_joint_origin_transform.position.y, 
                                lHipYaw->parent_to_joint_origin_transform.position.z);
        trunk_to_RHipYaw.setValue(rHipYaw->parent_to_joint_origin_transform.position.x, rHipYaw->parent_to_joint_origin_transform.position.y, 
                                rHipYaw->parent_to_joint_origin_transform.position.z);
    }else{
        ROS_ERROR("Robot type is not correctly defined, analytic IK wont work.");
    }

    // compute joint positions for fly and support leg
    bool success = true;
    if(is_left_support){
        // move from the trunk frame to the hip yaw frame and compute legIK
        goal.setRotation(trunk_to_support_foot.getRotation());
        goal.setOrigin(trunk_to_support_foot.getOrigin() - trunk_to_LHipYaw);
        success = success && legIK(goal, support_joints, true, robot);
        goal.setRotation(trunk_to_flying_foot.getRotation());
        goal.setOrigin(trunk_to_flying_foot.getOrigin() - trunk_to_RHipYaw);
        success = success && legIK(goal, fly_joints, false, robot);        
    }else{
        goal.setRotation(trunk_to_flying_foot.getRotation());
        goal.setOrigin(trunk_to_flying_foot.getOrigin() - trunk_to_LHipYaw);
        success = success && legIK(goal, fly_joints, true, robot);
        goal.setRotation(trunk_to_support_foot.getRotation());
        goal.setOrigin(trunk_to_support_foot.getOrigin() - trunk_to_RHipYaw);
        success = success && legIK(goal, support_joints, false, robot);
    }
    //success = success && legIK(trunk_to_support_foot, support_joints);
    //success = success && legIK(trunk_to_flying_foot, fly_joints);

    // if IK was successful, asign positions to legs
    if(success){
        std::vector<double> leg_joints;    
        if(is_left_support) {
            //todo
            //_robot_state_ptr->setJointPositions(_left_leg_joint_names, support_joints);
            //_robot_state_ptr->setJointPositions(_right_leg_joint_names, fly_joints);
            robot_state_ptr->setJointGroupPositions(_lleg_joints_group, support_joints);            
            robot_state_ptr->setJointGroupPositions(_rleg_joints_group, fly_joints);            
        }else{
            //_robot_state_ptr->setJointPositions(_left_leg_joint_names, fly_joints);
            //_robot_state_ptr->setJointPositions(_right_leg_joint_names, support_joints);
            robot_state_ptr->setJointGroupPositions(_lleg_joints_group, fly_joints);            
            robot_state_ptr->setJointGroupPositions(_rleg_joints_group, support_joints);            
        }
    }
    return success;
}

bool AnalyticIKSolver::legIK(tf2::Transform& goal, std::vector<double>& positions, bool isLeftLeg, urdf::Model robot){
    // this method works only for Darwin-OP like kinematics
    // the roll and pitch joints have to rotate around the same axis
    // the hip yaw has to be before the others

    // transform to foot from hip_yaw motor
    double hipYawToPitchX;
    double hipYawToPitchY;
    double hipYawToPitchZ;
    double hip_pitch_to_knee;
    double knee_to_ankle;
    double ankle_to_sole;
    urdf::JointConstSharedPtr hipPitch, hipRoll, kneeJoint, anklePitch, footToSole;
    
    //get necessary values from URDF
    hipPitch = robot.getJoint("LHipPitch");
    hipRoll = robot.getJoint("LHipRoll");
    kneeJoint = robot.getJoint("LKnee");
    anklePitch = robot.getJoint("LAnklePitch");

    if(_robot_type == "Wolfgang"){
        footToSole = robot.getJoint("left_foot_to_sole");
        hipYawToPitchX = hipPitch->parent_to_joint_origin_transform.position.x + hipRoll->parent_to_joint_origin_transform.position.x;
        hipYawToPitchY = hipPitch->parent_to_joint_origin_transform.position.y + hipRoll->parent_to_joint_origin_transform.position.y;
        hipYawToPitchZ = hipPitch->parent_to_joint_origin_transform.position.z + hipRoll->parent_to_joint_origin_transform.position.z;
        hip_pitch_to_knee = sqrt(pow(kneeJoint->parent_to_joint_origin_transform.position.x,2)+pow(kneeJoint->parent_to_joint_origin_transform.position.y,2)+pow(kneeJoint->parent_to_joint_origin_transform.position.z,2));
        knee_to_ankle = sqrt(pow(anklePitch->parent_to_joint_origin_transform.position.x,2)+pow(anklePitch->parent_to_joint_origin_transform.position.y,2)+pow(anklePitch->parent_to_joint_origin_transform.position.z,2));
        ankle_to_sole = footToSole->parent_to_joint_origin_transform.position.z;
    }else if(_robot_type=="Minibot"){
        footToSole = robot.getJoint("l_foot_to_sole");
        hipYawToPitchX = hipPitch->parent_to_joint_origin_transform.position.x - hipRoll->parent_to_joint_origin_transform.position.x;
        hipYawToPitchY = hipPitch->parent_to_joint_origin_transform.position.y - hipRoll->parent_to_joint_origin_transform.position.y;
        hipYawToPitchZ = hipPitch->parent_to_joint_origin_transform.position.z - hipRoll->parent_to_joint_origin_transform.position.z;
        hip_pitch_to_knee = fabs(kneeJoint->parent_to_joint_origin_transform.position.x);
        knee_to_ankle = fabs(anklePitch->parent_to_joint_origin_transform.position.x);
        ankle_to_sole = footToSole->parent_to_joint_origin_transform.position.x;
    }else{
        ROS_ERROR("Robot type is not correctly defined, analytic IK wont work.");
    }

    //ROS_INFO("Foot goal length %f", goal.getOrigin().length());
    // get RPY values for foot
    double foot_roll, foot_pitch, foot_yaw;
    tf2::Matrix3x3(goal.getRotation()).getRPY(foot_roll, foot_pitch, foot_yaw);
    // yaw can only be set with hip yaw. Compute it from goal    
    double hip_yaw = foot_yaw;
    //ROS_WARN("Goal from trunk  x:%f y:%f z:%f  OOO  r:%f, p:%f, y:%f", goal.getOrigin()[0], goal.getOrigin()[1], goal.getOrigin()[2], foot_roll, foot_pitch, foot_yaw);
    
    // translate the goal vector so that it starts at rotate point of hip_pitch and hip_roll    
    tf2::Vector3 hip_yaw_to_pitch {hipYawToPitchX, hipYawToPitchY, hipYawToPitchZ};
    hip_yaw_to_pitch = hip_yaw_to_pitch.rotate(tf2::Vector3{0.0, 0.0, 1.0}, foot_yaw);
    goal.setOrigin(goal.getOrigin() - hip_yaw_to_pitch);
    //ROS_WARN("Goal from hip  x:%f y:%f z:%f", goal.getOrigin()[0], goal.getOrigin()[1], goal.getOrigin()[2]);

    //ROS_INFO("goal from hip length %f", goal.getOrigin().length());

    // we compute the goal vector to the ankle joint
    // transform from goal position to ankle is given by ankle_sole distance in z direction
    tf2::Transform goal_to_ankle;
    goal_to_ankle.setOrigin(tf2::Vector3{0.0, 0.0, -1* ankle_to_sole});
    tf2::Transform to_ankle = goal * goal_to_ankle;    
    tf2::Vector3 ankle_goal = to_ankle.getOrigin();
    //ROS_WARN("Ankle Goal from hip  x:%f y:%f z:%f", ankle_goal[0], ankle_goal[1], ankle_goal[2]);
    //ROS_INFO("Ankle goal length %f", ankle_goal.length());

    // we can only find a solution if the ankle_goal is not too far away
    if(hip_pitch_to_knee + knee_to_ankle <= ankle_goal.length()){
        //the leg is not long enough to reach this
        ROS_ERROR("Analytic IK got no solution.");
        return false;
    }

    // we compute the hip roll
    // lookin from front, we can see that this is triangle between hip_roll and ankle_roll
    double hip_roll = atan2(ankle_goal[2], ankle_goal[1]);
    hip_roll = hip_roll + M_PI/2;
    // the ankle roll can now be computed
    // to keep the ankle parallel to the ground it has to be the inverse of the hip roll
    // we have to add the angle from the goal    
    double ankle_roll = -1* hip_roll + foot_roll;

    // we can now compute the remaining pitch values
    // the hip pitch value can be computed similar to the hip roll, but in the XZ plane
    double hip_pitch = atan2(ankle_goal[2], ankle_goal[0]);
    hip_pitch = hip_pitch + M_PI/2;
    // the ankle pitch is again similar to the ankle roll
    double ankle_pitch = -1*hip_pitch - foot_pitch;
    // now only the knee is left    
    // we have a triangle where two sides are the links from hip to knee and from knee to ankle
    // the thrid is the length of the ankle goal vector
    // we can use the law of cosines to compute the angle oposed to the ankle goal vector
    double beta = acos((pow(hip_pitch_to_knee,2) + pow(knee_to_ankle, 2) - pow(ankle_goal.length(), 2))/(2*hip_pitch_to_knee*knee_to_ankle));
    // the angle of the knee joint is 180- beta, since the knee is elongated at 0
    double knee = (M_PI - beta)*-1;

    //todo this is only true for legs with same length in upper and lower leg
    // adjust hip_pitch and ankle_pitch after calculation of the knee joint
    // calculation is done by the law of sines
    double alpha = asin(hip_pitch_to_knee*sin(beta) / fabs(ankle_goal.length()));
    double gamma = asin(knee_to_ankle*sin(beta) / fabs(ankle_goal.length()));
    hip_pitch += alpha;
    ankle_pitch += gamma;
    if(!isLeftLeg){
        hip_pitch = hip_pitch *-1;
        knee = knee * -1;
        ankle_pitch = ankle_pitch * -1;
        //hip_yaw = hip_yaw * -1;
    }
    if(_robot_type=="Minibot"){
        ankle_pitch = ankle_pitch *-1;
        hip_roll = hip_roll * -1;
        hip_yaw = hip_yaw * -1;
    }
    if(_robot_type=="Wolfgang"){
        ankle_pitch = ankle_pitch *-1;
        ankle_roll = ankle_roll * -1;
    }
    positions = std::vector<double>  {hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll};

    return true;
}
}
