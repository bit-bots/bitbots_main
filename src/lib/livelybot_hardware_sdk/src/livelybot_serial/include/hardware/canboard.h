#ifndef _CANBOARD_H_
#define _CANBOARD_H_

#include <iostream>
#include <vector>
#include "canport.h"
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Represents one CAN board, which hosts one or more CAN ports.
 *
 * The board reads its port count from the ROS 2 parameter server and
 * constructs the corresponding canport objects.
 */
class canboard
{
private:
    int CANport_num;
    rclcpp::Node::SharedPtr node_;
    std::vector<canport *> CANport;

public:
    /**
     * @param board_id  1-based index of this board in the robot configuration.
     * @param ser       Pointer to the vector of serial ports (one per CAN port).
     * @param node      Shared ROS 2 node used for parameter lookup and logging.
     */
    canboard(int board_id, std::vector<lively_serial *> *ser, rclcpp::Node::SharedPtr node);
    ~canboard() {}

    std::vector<canport *>& get_CANport();
    int get_CANport_num();

    /** Append all ports owned by this board into the provided vector. */
    void push_CANport(std::vector<canport *> *dest);

    void motor_send_2();
    void set_stop();
    void set_reset();

    /** Configure motor count on each port; returns the firmware version. */
    float set_port_motor_num();

    void send_get_motor_state_cmd();
    void send_get_motor_state_cmd2();
    void send_get_motor_version_cmd();
    void set_fun_v(fun_version v);
    void set_data_reset();
    void set_reset_zero();
    void set_motor_runzero();
    void set_time_out(int16_t t_ms);
    void set_time_out(uint8_t portx, int16_t t_ms);
    void canboard_bootloader();
    void canboard_fdcan_reset();
};

#endif /* _CANBOARD_H_ */
