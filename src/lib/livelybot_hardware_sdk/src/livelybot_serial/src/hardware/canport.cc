#include "canport.h"



canport::canport(int _CANport_num, int _CANboard_num, lively_serial *_ser) : ser(_ser)
{
    canboard_id = _CANboard_num;
    canport_id = _CANport_num;
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor_num", motor_num))
    {
        // ROS_INFO("Got params motor_num: %d",motor_num);
    }
    else
    {
        ROS_ERROR("Faile to get params motor_num");
    }

    if (PORT_MOTOR_NUM_MAX < motor_num)
    {
        ROS_ERROR("Too many motors, Supports up to %d motors, but there are actually %d motors", PORT_MOTOR_NUM_MAX, motor_num);
        exit(-1);
    }        

    for (int i = 1; i <= motor_num; i++)
    {
        int temp_id = 0;
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(i) + "/id", temp_id))
        {
            port_motor_id.push_back(temp_id);
            if (id_max < temp_id)
            {
                id_max = temp_id;
            }
        }
        else
        {
            ROS_ERROR("Faile to get params id");
        }
    }
    for (size_t i = 1; i <= motor_num; i++)
    {
        Motors.push_back(new motor(i, _CANport_num, _CANboard_num, &cdc_tr_message, id_max));
    }
    for (motor *m : Motors)
    {
        Map_Motors_p.insert(std::pair<int, motor *>(m->get_motor_id(), m));
    }
    ser->init_map_motor(&Map_Motors_p);
    ser->port_version_init(&port_version);
    ser->port_motors_id_init(&motors_id, &mode_flag);
    ser->port_fun_v_init(&fun_v);
}


float canport::set_motor_num()
{
    if (cdc_tr_message.head.s.cmd != MODE_SET_NUM)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_SET_NUM;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }

    cdc_tr_message.data.data[0] = motor_num;
    
    int t = 0;
    #define MAX_DALAY 1000  // 单位ms
    while (t++ < MAX_DALAY)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        if (port_version >= 2)
        {
            // ROS_INFO("\033[1;32m ttt %d\033[0m", t);
            break;
        }
    }

    if (t < MAX_DALAY)
    {
        ROS_INFO("\033[1;32mCANboard(%d) version is: v%.1f\033[0m", canboard_id, port_version);
    }
    else
    {
        ROS_ERROR("CANboard(%d) CANport(%d) Connection disconnected!!!", canboard_id, canport_id);
    }

    return port_version;
}


int canport::set_conf_load()
{
    if (cdc_tr_message.head.s.cmd != MODE_CONF_LOAD)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_CONF_LOAD;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    int t = 0;
    int num = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        num = 0;
        if (mode_flag == MODE_CONF_LOAD)
        {
            for (int i = 1; i <= motor_num; i++)
            {
                int id = port_motor_id[i - 1];
                if (motors_id.count(id) == 1)
                {
                    ++num;
                }
            }
        }

        if (num == motor_num)
        {
            break;
        }
    }

    if (num == motor_num)
    {
        ROS_INFO("\033[1;32mSettings have been restored. Initiating motor zero point reset.\033[0m");
        return 0;
    }
    else 
    {
        ROS_ERROR("Restoration of settings failed.");
        return 1;
    }
}


int canport::set_conf_load(int id)
{
    
    if (cdc_tr_message.head.s.cmd != MODE_CONF_LOAD)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_CONF_LOAD;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = id;

    int t = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        if (mode_flag == MODE_CONF_LOAD && motors_id.count(id) == 1)
        {
            return 0;
        }
    }

    return 1;
}


int canport::set_reset_zero()
{
    if (cdc_tr_message.head.s.cmd != MODE_RESET_ZERO)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RESET_ZERO;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    int t = 0;
    int num = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        num = 0;
        if (mode_flag == MODE_RESET_ZERO)
        {
            for (int i = 1; i <= motor_num; i++)
            {
                // if (motors_id.count(i) == 1)
                int id = port_motor_id[i - 1];
                if (motors_id.count(id) == 1)
                {
                    ++num;
                }
            }
        }

        if (num == motor_num)
        {
            break;
        }
    }

    if (num == motor_num)
    {
        ROS_INFO("\033[1;32mMotor zero position reset successfully, waiting for the motor to save the settings.\033[0m");
        return 0;
    }
    else 
    {
        ROS_ERROR("Motor reset to zero position failed.");
        return 1;
    }
}


int canport::set_reset_zero(int id)
{
    
    if (cdc_tr_message.head.s.cmd != MODE_RESET_ZERO)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RESET_ZERO;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = id;

    int t = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        if (mode_flag == MODE_RESET_ZERO && motors_id.count(id) == 1)
        {
            return 0;
        }
    }

    return 1;
}


void canport::set_stop()
{
    if (cdc_tr_message.head.s.cmd != MODE_STOP)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_STOP;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
}


void canport::set_motor_runzero()
{
    if (cdc_tr_message.head.s.cmd != MODE_RUNZERO)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RUNZERO;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    motor_send_2();
}


void canport::set_reset()
{
    if (cdc_tr_message.head.s.cmd != MODE_RESET)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RESET;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
}


void canport::set_conf_write()
{
    
    if (cdc_tr_message.head.s.cmd != MODE_CONF_WRITE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_CONF_WRITE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    int t = 0;
    int num = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        num = 0;
        if (mode_flag == MODE_CONF_WRITE)
        {
            for (int i = 1; i <= motor_num; i++)
            {
                int id = port_motor_id[i - 1];
                if (motors_id.count(id) == 1)
                {
                    ++num;
                }
            }
        }

        if (num == motor_num)
        {
            break;
        }
    }

    if (num == motor_num)
    {
        ROS_INFO("\033[1;32mSettings saved successfully.\033[0m");
    }
    else 
    {
        ROS_INFO("\033[1;32mFailed to save settings.\033[0m");
        exit(-1);
    }
}


int canport::set_conf_write(int id)
{
    
    if (cdc_tr_message.head.s.cmd != MODE_CONF_WRITE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_CONF_WRITE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = id;

    int t = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        if (mode_flag == MODE_CONF_WRITE && motors_id.count(id) == 1)
        {
            return 0;
        }
    }

    return 1;
}


void canport::send_get_motor_state_cmd()
{
    if (cdc_tr_message.head.s.cmd != MODE_MOTOR_STATE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_MOTOR_STATE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_2();
}


void canport::send_get_motor_state_cmd2()
{
    if (cdc_tr_message.head.s.cmd != MODE_MOTOR_STATE2)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_MOTOR_STATE2;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_2();
}


void canport::send_get_motor_version_cmd()
{
    if (cdc_tr_message.head.s.cmd != MODE_MOTOR_VERSION)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_MOTOR_VERSION;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_2();
}


void canport::set_fun_v(fun_version v)
{
    if (cdc_tr_message.head.s.cmd != MODE_FUN_V)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_FUN_V;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = v;

    int t = 0;
    #define MAX_DALAY 1000  // 单位ms
    while (t++ < MAX_DALAY)
    {
        motor_send_2();
        ros::Duration(0.02).sleep();
        if (v == fun_v)
        {
            // ROS_INFO("\033[1;32m ttt %d\033[0m", t);
            break;
        }
    }

    if (t == MAX_DALAY)
    {
        ROS_ERROR("CANboard(%d) CANport(%d) fun_v err!!!", canboard_id, canport_id);
    }
}


void canport::set_data_reset()
{
    memset(cdc_tr_message.data.data, 0xFFFFFFFF, CDC_TR_MESSAGE_DATA_LEN / sizeof(int));
}


void canport::set_time_out(int16_t t_ms)
{
    if (cdc_tr_message.head.s.cmd != MODE_TIME_OUT)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_TIME_OUT;
        cdc_tr_message.head.s.len = motor_num * 2;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }

    for (int i = 0; i < motor_num; i++)
    {
        cdc_tr_message.data.timeout[i] = t_ms;
    }
    
    motor_send_2();
}


void canport::puch_motor(std::vector<motor *> *_Motors)
{
    for (motor *m : Motors)
    {
        _Motors->push_back(m);
    }
}


void canport::motor_send_2()
{
    ser->send_2(&cdc_tr_message);
}


int canport::get_motor_num()
{
    return motor_num;
}


int canport::get_canboard_id()
{
    return canboard_id;
}


int canport::get_canport_id()
{
    return canport_id;
}


void canport::canboard_bootloader()
{
    if (cdc_tr_message.head.s.cmd != MODE_BOOTLOADER)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_BOOTLOADER;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    
    motor_send_2();
    motor_send_2();
    motor_send_2();
}


void canport::canboard_fdcan_reset()
{
    if (cdc_tr_message.head.s.cmd != MODE_FDCAN_RESET)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_FDCAN_RESET;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    
    motor_send_2();
    motor_send_2();
    motor_send_2();
}
