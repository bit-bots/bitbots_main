#include <bitbots_ros_control/dynamixel_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <bitbots_ros_control/JointCommand.h>

namespace dynamixel_controller
{
bool DynamixelController::init(hardware_interface::PosVelAccCurJointInterface* hw, ros::NodeHandle &n){
    // List of controlled joints
        _pubControllerCommand = n.advertise<bitbots_ros_control::JointCommand>("/DynamixelController/command_test", 1);

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
        _joint_map[joint_names_[i]] = i;          
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
        }
    }

    //commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    sub_command_ = n.subscribe("command", 1, &DynamixelController::commandCB, this, ros::TransportHints().tcpNoDelay());
    return true;
}

void DynamixelController::starting(const ros::Time& time){}
void DynamixelController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {    
    std::vector<JointCommandData> & buf_data  = *commands_buffer_.readFromRT();

    for(unsigned int i = 0; i < buf_data.size(); i++){
        joints_[buf_data[i].id].setCommand(buf_data[i].pos, buf_data[i].vel, buf_data[i].acc, buf_data[i].cur);        
    }
   //std::cout << ::getpid();

}
}
PLUGINLIB_EXPORT_CLASS(dynamixel_controller::DynamixelController, controller_interface::ControllerBase)
