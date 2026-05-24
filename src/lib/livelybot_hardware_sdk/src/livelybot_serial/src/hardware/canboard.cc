#include "canboard.h"
#include <thread>
#include <chrono>


canboard::canboard(int _CANboard_ID, std::vector<lively_serial *> *ser, rclcpp::Node::SharedPtr node)
    : node_(node)
{
    const std::string board_base = "robot.CANboard.No_" + std::to_string(_CANboard_ID) + "_CANboard";

    if (!node_->get_parameter(board_base + ".CANport_num", CANport_num))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params CANport_num");
    }

    for (int j = 1; j <= CANport_num; j++)  // one serial port per CAN port
    {
        CANport.push_back(new canport(j, _CANboard_ID,
                                     (*ser)[(_CANboard_ID - 1) * CANport_num + j - 1], node_));
    }
}


std::vector<canport*>& canboard::get_CANport()
{
    return CANport;
}


int canboard::get_CANport_num()
{
    return CANport_num;
}


void canboard::push_CANport(std::vector<canport*> *_CANport)
{
    for (canport *c : CANport)
    {
        _CANport->push_back(c);
    }
}


void canboard::motor_send_2()
{
    for (canport *c : CANport)
    {
        c->motor_send_2();
    }
}


void canboard::set_stop()
{
    for (canport *c : CANport)
    {
        c->set_stop();
    }
}


void canboard::set_reset()
{
    for (canport *c : CANport)
    {
        c->set_reset();
    }
}


float canboard::set_port_motor_num()
{
    float v = 0;
    for (canport *c : CANport)
    {
        v = c->set_motor_num();
    }

    return v;
}


void canboard::send_get_motor_state_cmd()
{
    for (canport *c : CANport)
    {
        c->send_get_motor_state_cmd();
    }
}


void canboard::send_get_motor_state_cmd2()
{
    for (canport *c : CANport)
    {
        c->send_get_motor_state_cmd2();
    }
}


void canboard::send_get_motor_version_cmd()
{
    for (canport *c : CANport)
    {
        c->send_get_motor_version_cmd();
    }
}


void canboard::set_fun_v(fun_version v)
{
    for (canport *c : CANport)
    {
        c->set_fun_v(v);
    }
}


void canboard::set_data_reset()
{
    for (canport *c : CANport)
    {
        c->set_data_reset();
    }
}


void canboard::set_reset_zero()
{
    for (canport *c : CANport)
    {
        for (int i = 0; i < 5; i++)
        {
            c->set_reset();
            c->motor_send_2();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (c->set_reset_zero() == 0)
        {
            c->set_conf_write();
        }
        c->set_reset();
        c->motor_send_2();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        c->motor_send_2();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


void canboard::set_motor_runzero()
{
    for (canport *c : CANport)
    {
        c->set_motor_runzero();
    }
}


void canboard::set_time_out(int16_t t_ms)
{
    for (canport *c : CANport)
    {
        c->set_time_out(t_ms);
    }
}


void canboard::set_time_out(uint8_t portx, int16_t t_ms)
{
    CANport[portx]->set_time_out(t_ms);
}


void canboard::canboard_bootloader()
{
    CANport[0]->canboard_bootloader();
}


void canboard::canboard_fdcan_reset()
{
    CANport[0]->canboard_fdcan_reset();
}
