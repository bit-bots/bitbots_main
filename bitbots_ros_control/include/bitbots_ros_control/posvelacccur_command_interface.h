#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_POSVELACCCUR_COMMAND_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_POSVELACCCUR_COMMAND_INTERFACE_H_

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/posvelacc_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelAccCurJointHandle : public PosVelAccJointHandle
{
public:
  PosVelAccCurJointHandle() : PosVelAccJointHandle(), cmd_cur_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd_pos A pointer to the storage for this joint's output command position
   * \param cmd_vel A pointer to the storage for this joint's output command velocity
   * \param cmd_acc A pointer to the storage for this joint's output command acceleration
   * \param eff_cmd A pointer to the storage for this joint's output command current 
   */
  PosVelAccCurJointHandle(const JointStateHandle& js, double* cmd_pos, double* cmd_vel, double *cmd_acc,  double* cmd_cur)
    : PosVelAccJointHandle(js, cmd_pos, cmd_vel, cmd_acc), cmd_cur_(cmd_cur)
  {
    if (!cmd_cur)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command current data pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_vel, double cmd_acc, double cmd_cur)
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandAcceleration(cmd_acc);
    setCommandCurrent(cmd_cur);
  }

  void setCommandCurrent(double cmd_cur) {assert(cmd_cur_); *cmd_cur_ = cmd_cur;}
  double getCommandCurrent() const {assert(cmd_cur_); return *cmd_cur_;}

private:
  double* cmd_cur_;
};


/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by position, velocity &
 * current together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelAccCurJointInterface : public HardwareResourceManager<PosVelAccCurJointHandle, ClaimResources> {};

}

#endif