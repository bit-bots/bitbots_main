#include "canboard.h"


canboard::canboard(int _CANboard_ID, std::vector<lively_serial *> *ser)
{
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_ID) + "_CANboard/CANport_num", CANport_num))
    {
        // ROS_INFO("Got params CANport_num: %d",CANport_num);
    }
    else
    {
        ROS_ERROR("Faile to get params CANport_num");
    }
    for (size_t j = 1; j <= CANport_num; j++) // 一个串口对应一个CANport
    {
        CANport.push_back(new canport(j, _CANboard_ID, (*ser)[(_CANboard_ID - 1) * CANport_num + j - 1]));
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
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1).sleep();
        if (c->set_reset_zero() == 0)
        {
            c->set_conf_write();
        }
        c->set_reset();
        c->motor_send_2();
        ros::Duration(1).sleep();
        c->motor_send_2();
        ros::Duration(1).sleep();
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