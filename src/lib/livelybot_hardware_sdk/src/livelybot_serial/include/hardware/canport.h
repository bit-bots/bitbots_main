#ifndef _CANPORT_H_
#define _CANPORT_H_

#include "motor.h"
#include "../lively_serial.h"
#include <condition_variable>
#include <thread>
#include <unordered_set>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

/** Maximum number of motors addressable on a single CAN port. */
#define PORT_MOTOR_NUM_MAX  30

/**
 * @brief Manages one CAN port (one serial connection + its set of motors).
 *
 * Reads the motor count and per-motor configuration from the ROS 2 parameter
 * server, then constructs motor objects and ties them to the shared TX message
 * buffer.
 */
class canport
{
private:
    int motor_num;
    rclcpp::Node::SharedPtr node_;
    std::vector<motor *> Motors;
    std::map<int, motor *> Map_Motors_p;
    int canboard_id, canport_id;
    lively_serial *ser;
    cdc_tr_message_s cdc_tr_message;
    int id_max = 0;
    float port_version = 0.0f;
    fun_version fun_v = fun_v1;
    std::unordered_set<int> motors_id;
    int mode_flag = 0;
    std::vector<int> port_motor_id;
    std::vector<cdc_rx_motor_version_s *> motor_version;

public:
    /**
     * @param port_num   1-based port index within the board.
     * @param board_num  1-based board index.
     * @param ser        Serial port servicing this CAN port.
     * @param node       Shared ROS 2 node for parameter access and logging.
     */
    canport(int port_num, int board_num, lively_serial *ser, rclcpp::Node::SharedPtr node);

    /** Send MODE_SET_NUM and wait for firmware version acknowledgement. */
    float set_motor_num();

    int set_conf_load();
    int set_conf_load(int id);
    int set_reset_zero();
    int set_reset_zero(int id);
    void set_stop();
    void set_motor_runzero();
    void set_reset();
    void set_conf_write();
    int set_conf_write(int id);
    void send_get_motor_state_cmd();
    void send_get_motor_state_cmd2();
    void send_get_motor_version_cmd();
    void set_fun_v(fun_version v);
    void set_data_reset();
    void set_time_out(int16_t t_ms);

    /** Append all motors on this port into the provided vector. */
    void puch_motor(std::vector<motor *> *dest);

    /** Transmit the current TX buffer via the associated serial port. */
    void motor_send_2();

    int get_motor_num();
    int get_canboard_id();
    int get_canport_id();
    void canboard_bootloader();
    void canboard_fdcan_reset();
};

#endif /* _CANPORT_H_ */
