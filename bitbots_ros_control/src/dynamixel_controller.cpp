#include <bitbots_ros_control/dynamixel_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <bitbots_ros_control/JointCommand.h>

namespace dynamixel_controller
{
bool DynamixelController::init(hardware_interface::PosVelAccCurJointInterface* hw, ros::NodeHandle &n){
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
        return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }   
    for(unsigned int i=0; i<n_joints_; i++)
    {
        try
        {
        joints_.push_back(hw->getHandle(joint_names_[i]));          
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
        }
    }

    //commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    sub_command_ = n.subscribe("command", 1, &DynamixelController::commandCB, this);
    return true;
}

void DynamixelController::starting(const ros::Time& time){}
void DynamixelController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    bitbots_ros_control::JointCommand & command_msg = *commands_buffer_.readFromRT();

    //Todo check if length of joint names and commands is the same
    //iterate over all joint names in the message
    for(unsigned int i = 0; i < command_msg.joint_names.size(); i++){
        // find the index of this joint name by iterating over the saved joint name order
        for(unsigned int j = 0; j < n_joints_; j++){
            if(command_msg.joint_names[i] == joint_names_[j]){
                joints_[j].setCommand(command_msg.positions[i], command_msg.velocities[i], command_msg.accelerations[i], command_msg.max_currents[i]);
                continue;
            }
            // if the name is not know, skip it
            if(i == n_joints_){
                ROS_WARN("Joint %s in JointCommand message is not known.", command_msg.joint_names[i]);
                continue;
            }
        }        
    }
}
}
PLUGINLIB_EXPORT_CLASS(dynamixel_controller::DynamixelController, controller_interface::ControllerBase)
