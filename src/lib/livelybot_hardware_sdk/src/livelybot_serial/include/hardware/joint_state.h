#ifndef LIVELYBOT_SERIAL_JOINT_STATE_H
#define LIVELYBOT_SERIAL_JOINT_STATE_H

#include "motor.h"
#include <optional>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace livelybot_serial
{

/** Build a complete joint state only after every motor has reported feedback. */
inline std::optional<sensor_msgs::msg::JointState> make_joint_state(const std::vector<motor *> &motors)
{
    if (motors.empty())
        return std::nullopt;

    sensor_msgs::msg::JointState js;
    for (motor *m : motors)
    {
        const motor_back_t *data = m->get_current_motor_state();
        // Partial messages can make consumers substitute incorrect initial joint positions.
        if (data->time == 0.0)
            return std::nullopt;

        js.name.push_back(m->get_motor_name());
        js.position.push_back(data->position);
        js.velocity.push_back(data->velocity);
        js.effort.push_back(data->torque);
    }
    return js;
}

} // namespace livelybot_serial

#endif
