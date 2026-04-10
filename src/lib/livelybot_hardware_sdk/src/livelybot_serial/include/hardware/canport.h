#ifndef _CANPORT_H_
#define _CANPORT_H_
#include "ros/ros.h"
#include "motor.h"
#include <condition_variable>
#include <thread>
#include "../lively_serial.h"
#include <unordered_set>
#include <iostream>


#define  PORT_MOTOR_NUM_MAX  30

class canport
{
private:
    int motor_num;
    ros::NodeHandle n;
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
    canport(int _CANport_num, int _CANboard_num, lively_serial *_ser);
    // ~canport();

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
    void puch_motor(std::vector<motor *> *_Motors);
    void motor_send_2();
    int get_motor_num();
    int get_canboard_id();
    int get_canport_id();
    void canboard_bootloader();
    void canboard_fdcan_reset();
};

#endif
