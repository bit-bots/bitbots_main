#include "motor.h"


motor::motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max)
: CANport_num(_CANport_num), CANboard_num(_CANboard_num), p_cdc_tx_message(_p_cdc_tx_message), id_max(_id_max)
{
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/name", motor_name))
    {
        // ROS_INFO("Got params name: %s",motor_name);
    }
    else
    {
        ROS_ERROR("Faile to get params name");
    }
    _motor_pub = n.advertise<livelybot_msg::MotorState>("/livelybot_real_real/" + motor_name + "_controller/state", 1);

    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/id", id))
    {
        // ROS_INFO("Got params id: %d",id);
    }
    else
    {
        ROS_ERROR("Faile to get params id");
    }

    std::string type_str;
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/type", type_str))
    {
        try 
        {
            ROS_INFO("Got params type: %s", type_str.c_str());
            type = motor_type2.at(type_str);
        } 
        catch (const std::out_of_range& e) 
        {
            ROS_ERROR("Motor model error: %s", type_str.c_str());
            exit(-2); 
        }
    }
    else
    {
        ROS_ERROR("Faile to get params type");
    }
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/num", num))
    {
        // ROS_INFO("Got params num: %d",num);
    }
    else
    {
        ROS_ERROR("Faile to get params num");
    }
    // position limit
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/pos_limit_enable", pos_limit_enable))
    {
        // ROS_INFO("Got params pos_limit_enable: %s",pos_limit_enable?"true":"false");
    }
    else
    {
        ROS_ERROR("Faile to get params pos_upper");
    }
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/pos_upper", pos_upper))
    {
        // ROS_INFO("Got params pos_upper: %f",pos_upper);
    }
    else
    {
        ROS_ERROR("Faile to get params pos_upper");
    }
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/pos_lower", pos_lower))
    {
        // ROS_INFO("Got params pos_lower: %f",pos_lower);
    }
    else
    {
        ROS_ERROR("Faile to get params pos_lower");
    }
    // torque limit
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/tor_limit_enable", tor_limit_enable))
    {
        // ROS_INFO("Got params tor_limit_enable: %s",tor_limit_enable?"true":"false");
    }
    else
    {
        ROS_ERROR("Faile to get params tor_upper");
    }
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/tor_upper", tor_upper))
    {
        // ROS_INFO("Got params tor_upper: %f",tor_upper);
    }
    else
    {
        ROS_ERROR("Faile to get params tor_upper");
    }
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/tor_lower", tor_lower))
    {
        // ROS_INFO("Got params tor_lower: %f",tor_lower);
    }
    else
    {
        ROS_ERROR("Faile to get params tor_lower");
    }
    if (n.getParam("robot/control_type", control_type))
    {
        // ROS_INFO("Got params ontrol_type: %f",SDK_version);
    }
    else
    {
        ROS_ERROR("Faile to get params control_type");
    }
    set_motor_type(type);
    data.time = 0;
    data.ID = id;
    data.mode = 0;
    data.fault = 0;
    data.position = 999.0f;
    data.velocity = 0;
    data.torque = 0;
}


inline int16_t motor::pos_float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return (int16_t)(in_data / my_2pi * 10000.0);
    case (pos_vel_convert_type::angle_360):
        return (int16_t)(in_data / 360.0 * 10000.0);
    case (pos_vel_convert_type::turns):
        return (int16_t)(in_data * 10000.0);
    default:
        return int16_t();
    }
}

inline int16_t motor::vel_float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return (int16_t)(in_data / my_2pi * 4000.0);
    case (pos_vel_convert_type::angle_360):
        return (int16_t)(in_data / 360.0 * 4000.0);
    case (pos_vel_convert_type::turns):
        return (int16_t)(in_data * 4000.0);
    default:
        return int16_t();
    }
}

inline int16_t motor::tqe_float2int(float in_data, motor_type motor_type)
{
#define TQE_ADJUST(data, k, d)  ((int16_t)(((data) - (d)) / (k)))

    switch (motor_type)
    {
    case motor_type::null:
        ROS_ERROR("motor type not set,fresh command fault");
        return int16_t();
    case motor_type::m3536_32:
        return TQE_ADJUST(in_data, 0.004581, -0.105);
    case motor_type::m5046_20:
        return TQE_ADJUST(in_data, 0.005280, -0.07);
    case motor_type::m4538_19:
        return TQE_ADJUST(in_data, 0.004450, -0.05);
    case motor_type::m5047_36:
        return TQE_ADJUST(in_data, 0.004938, -0.03313);
    case motor_type::m5047_09:
        return TQE_ADJUST(in_data, 0.005330, -0.034809);
    case motor_type::m4438_30:
        return TQE_ADJUST(in_data, 0.005256, -0.05);
    case motor_type::m4438_32:
        return TQE_ADJUST(in_data, 0.005584, -0.083);
    case (motor_type::m5047_36_2):
        return TQE_ADJUST(in_data, 0.008030, -0.35);
    case (motor_type::m6056_36):
        return TQE_ADJUST(in_data, 0.006770, -0.1);
    case (motor_type::m7256_35):
        return TQE_ADJUST(in_data,  0.00677, -0.244);
    case (motor_type::m60sg_35):
    case (motor_type::m60bm_35):
        return TQE_ADJUST(in_data, 0.007942, -0.18);
    case (motor_type::m5043_20):
        return TQE_ADJUST(in_data, 0.009660, -0.115);
    case motor_type::mGeneral:
        return TQE_ADJUST(in_data, 0.005000, 0);
    default:
        ROS_ERROR("Motor Type SettingMotor type setting error");
        exit(-1);
        return int16_t();
    }
}


inline float motor::tqe_int2float(int16_t in_data, motor_type type)
{
#define TQE_RESTORE(data, k, d) ((data) * (k) + (d))

    switch (type)
    {
    case motor_type::null:
        ROS_ERROR("motor type not set,fresh command fault");
        return int16_t();
    case motor_type::m3536_32:
        return TQE_RESTORE(in_data, 0.004581, -0.105);
    case motor_type::m5046_20:
        return TQE_RESTORE(in_data, 0.005280, -0.07);
    case motor_type::m4538_19:
        return TQE_RESTORE(in_data, 0.004450, -0.05);
    case motor_type::m5047_36:
        return TQE_RESTORE(in_data, 0.004938, -0.03313);
    case motor_type::m5047_09:
        return TQE_RESTORE(in_data, 0.005330, -0.034809);
    case motor_type::m4438_30:
        return TQE_RESTORE(in_data, 0.005256, -0.05);
    case motor_type::m4438_32:
        return TQE_RESTORE(in_data, 0.005584, -0.083);
    case (motor_type::m5047_36_2):
        return TQE_RESTORE(in_data, 0.008030, -0.35);
    case (motor_type::m6056_36):
        return TQE_RESTORE(in_data, 0.006770, -0.1);
    case (motor_type::m7256_35):
        return TQE_RESTORE(in_data,  0.00677, -0.244);
    case (motor_type::m60sg_35):
    case (motor_type::m60bm_35):
        return TQE_RESTORE(in_data, 0.007942, -0.18);
    case (motor_type::m5043_20):
        return TQE_RESTORE(in_data, 0.009660, -0.115);
    case motor_type::mGeneral:
        return TQE_RESTORE(in_data, 0.005000, 0);
    default:
        ROS_ERROR("Motor Type SettingMotor type setting error");
        exit(-1);
        return int16_t();
    }
}


inline float motor::pid_scale(float in_data, motor_type motor_type)
{
#define PID_SCALE(data, k)  (((data) / (k)))

    switch (motor_type)
    {
    case motor_type::null:
        ROS_ERROR("motor type not set,fresh command fault");
        return int16_t();
    case motor_type::m3536_32:
        return PID_SCALE(in_data, 0.004581);
    case motor_type::m5046_20:
        return PID_SCALE(in_data, 0.5330);
    case motor_type::m4538_19:
        return PID_SCALE(in_data, 0.4938);
    case motor_type::m5047_36:
        return PID_SCALE(in_data, 0.4938);
    case motor_type::m5047_09:
        return PID_SCALE(in_data, 0.5470);
    case motor_type::m4438_30:
        return PID_SCALE(in_data, 0.5256);
    case motor_type::m4438_32:
        return PID_SCALE(in_data, 0.5584);
    case (motor_type::m5047_36_2):
        return PID_SCALE(in_data, 0.8030);
    case (motor_type::m6056_36):
        return PID_SCALE(in_data, 0.6770);
    case (motor_type::m7256_35):
        return PID_SCALE(in_data,  0.00677);
    case (motor_type::m60sg_35):
    case (motor_type::m60bm_35):
        return PID_SCALE(in_data, 0.7942);
    case (motor_type::m5043_20):
        return PID_SCALE(in_data, 0.9660);
    case motor_type::mGeneral:
        return PID_SCALE(in_data, 0.5000);
    default:
        ROS_ERROR("Motor Type SettingMotor type setting error");
        exit(-1);
        return int16_t();
    }

    return in_data;
}

inline int16_t motor::int16_limit(int32_t data)
{
    if (data >= 32700)
    {
        ROS_INFO("\033[1;32mPID output has reached the saturation limit.\033[0m");
        return (int16_t)32700;
    }
    else if (data <= -32700)
    {
        ROS_INFO("\033[1;32mPID output has reached the saturation limit.\033[0m");
        return (int16_t)-32700;
    }

    return (int16_t)data;
}


inline int16_t motor::kp_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        tqe = (int32_t)(in_data * 10 * my_2pi);
        break;
    case (pos_vel_convert_type::angle_360):
        tqe = (int32_t)(in_data * 10 * 360);
        break;
    case (pos_vel_convert_type::turns):
        tqe = (int32_t)(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }
    return int16_limit(tqe);
}


inline int16_t motor::ki_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        tqe = (int32_t)(in_data * 10 * my_2pi);
        break;
    case (pos_vel_convert_type::angle_360):
        tqe = (int32_t)(in_data * 10 * 360);
        break;
    case (pos_vel_convert_type::turns):
        tqe = (int32_t)(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }

    return int16_limit(tqe);
}

inline int16_t motor::kd_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        tqe = (int32_t)(in_data * 10 * my_2pi);
        break;
    case (pos_vel_convert_type::angle_360):
        tqe = (int32_t)(in_data * 10 * 360);
        break;
    case (pos_vel_convert_type::turns):
        tqe = (int32_t)(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }

    return int16_limit(tqe);
}

inline float motor::pos_int2float(int16_t in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return (float)(in_data * my_2pi / 10000.0);
    case (pos_vel_convert_type::angle_360):
        return (float)(in_data * 360.0 / 10000.0);
    case (pos_vel_convert_type::turns):
        return (float)(in_data / 10000.0);
    default:
        return float();
    }
}

inline float motor::vel_int2float(int16_t in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return (float)(in_data * my_2pi / 4000.0);
    case (pos_vel_convert_type::angle_360):
        return (float)(in_data * 360.0 / 4000.0);
    case (pos_vel_convert_type::turns):
        return (float)(in_data / 4000.0);
    default:
        return float();
    }
}


void motor::fresh_cmd_int16(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current)
{
    switch (control_type)
    {
    case (1):
        motor::position(position);
        break;
    case (2):
        motor::velocity(velocity);
        break;
    case (3):
        motor::torque(torque);
        break;
    case (4):
        motor::voltage(voltage);
        break;
    case (5):
        motor::current(current);
        break;
    case (6):
        motor::pos_vel_MAXtqe(position, velocity, torque);
        break;
    case (7):
        ROS_ERROR("This mode has beenThis mode is deprecated.");
        exit(-1);
    case (8):
        ROS_ERROR("This mode has beenThis mode is deprecated.");
        exit(-1);
    case (9):
        motor::pos_vel_tqe_kp_kd(position, velocity, torque, kp, kd);
        break;
    case (10):
        motor::pos_vel_kp_kd(position, velocity, kp, kd);
        break;
    case (11):
        motor::pos_vel_acc(position, velocity, acc);
        break;
    case (12):
        motor::pos_vel_tqe_kp_kd2(position, velocity, torque, kp, kd);
        break;
    default:
        ROS_ERROR("Incorrect setting of operation mode.");
        exit(-3);
        break;
    }
}

void motor::position(float position)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POSITION)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POSITION;
        p_cdc_tx_message->head.s.len = id_max * sizeof(int16_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.position[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.position[MEM_INDEX_ID(id)] = pos_float2int(position, pos_vel_type);
}

void motor::velocity(float velocity)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_VELOCITY)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_VELOCITY;
        p_cdc_tx_message->head.s.len = id_max * sizeof(int16_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.position[i] = 0x0000;
        }
    }

    p_cdc_tx_message->data.velocity[MEM_INDEX_ID(id)] = vel_float2int(velocity, pos_vel_type);
}

void motor::torque(float torque)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_TORQUE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_TORQUE;
        p_cdc_tx_message->head.s.len = id_max * sizeof(int16_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.torque[i] = 0x0000;
        }
    }

    p_cdc_tx_message->data.torque[MEM_INDEX_ID(id)] = tqe_float2int(torque, type_);
}

void motor::voltage(float voltage)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_VOLTAGE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_VOLTAGE;
        p_cdc_tx_message->head.s.len = id_max * sizeof(int16_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.voltage[i] = 0x0000;
        }
    }

    p_cdc_tx_message->data.voltage[MEM_INDEX_ID(id)] = (int16_t)(voltage * 10);
}

void motor::current(float current)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_CURRENT)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_CURRENT;
        p_cdc_tx_message->head.s.len = id_max * sizeof(int16_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.current[i] = 0x0000;
        }
    }

    p_cdc_tx_message->data.current[MEM_INDEX_ID(id)] = (int16_t)(current * 10);
}

void motor::set_motorout(int16_t t_ms)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_TIME_OUT)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_TIME_OUT;
        p_cdc_tx_message->head.s.len = id_max * sizeof(int16_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.timeout[i] = 0x0000;
        }
    }

    p_cdc_tx_message->data.timeout[MEM_INDEX_ID(id)] = t_ms;
}

void motor::pos_vel_MAXtqe(float position, float velocity, float torque_max)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_TQE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_TQE;
        p_cdc_tx_message->head.s.len = id_max * sizeof(motor_pos_val_tqe_s);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.pos_val_tqe[i].pos = 0x8000;
            p_cdc_tx_message->data.pos_val_tqe[i].val = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe[i].tqe = 0x0000;
        }
    }
    p_cdc_tx_message->data.pos_val_tqe[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_val_tqe[MEM_INDEX_ID(id)].val = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_val_tqe[MEM_INDEX_ID(id)].tqe = tqe_float2int(torque_max, type_);
}

void motor::pos_vel_acc(float position, float velocity, float acc)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_ACC)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_ACC;
        p_cdc_tx_message->head.s.len = id_max * sizeof(motor_pos_val_tqe_rpd_s);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.pos_val_acc[i].pos = 0x8000;
            p_cdc_tx_message->data.pos_val_acc[i].val = 0x0000;
            p_cdc_tx_message->data.pos_val_acc[i].acc = 0x0000;
        }
    }
    p_cdc_tx_message->data.pos_val_acc[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_val_acc[MEM_INDEX_ID(id)].val = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_val_acc[MEM_INDEX_ID(id)].acc = (int16_t)(acc * 1000);
}

void motor::pos_vel_tqe_kp_kd(float position, float velocity, float torque, float kp, float kd)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_TQE_KP_KD)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_TQE_KP_KD;
        p_cdc_tx_message->head.s.len = id_max * sizeof(motor_pos_val_tqe_rpd_s);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].pos = 0x8000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].val = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].tqe = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].rkp = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].rkd = 0x0000;
        }
    }
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].val = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].tqe = tqe_float2int(torque, type_);
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].rkp = kp_float2int(kp, pos_vel_type, type_); 
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].rkd = kd_float2int(kd, pos_vel_type, type_);
}

void motor::pos_vel_tqe_kp_kd2(float position, float velocity, float torque, float kp, float kd)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_TQE_KP_KD2)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_TQE_KP_KD2;
        p_cdc_tx_message->head.s.len = id_max * sizeof(motor_pos_val_tqe_rpd_s);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].pos = 0x8000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].val = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].tqe = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].rkp = 0x0000;
            p_cdc_tx_message->data.pos_val_tqe_rpd[i].rkd = 0x0000;
        }
    }
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].val = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].tqe = tqe_float2int(torque, type_);
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].rkp = kp_float2int(kp, pos_vel_type, type_); 
    p_cdc_tx_message->data.pos_val_tqe_rpd[MEM_INDEX_ID(id)].rkd = kd_float2int(kd, pos_vel_type, type_);
}

void motor::pos_vel_kp_kd(float position, float velocity, float kp, float kd)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_KP_KD)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_KP_KD;
        p_cdc_tx_message->head.s.len = id_max * sizeof(motor_pos_val_rpd_s);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.pos_val_rpd[i].pos = 0x8000;
            p_cdc_tx_message->data.pos_val_rpd[i].val = 0x0000;
            p_cdc_tx_message->data.pos_val_rpd[i].rkp = 0x0000;
            p_cdc_tx_message->data.pos_val_rpd[i].rkd = 0x0000;
        }
    }
    p_cdc_tx_message->data.pos_val_rpd[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_val_rpd[MEM_INDEX_ID(id)].val = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_val_rpd[MEM_INDEX_ID(id)].rkp = kp_float2int(kp, pos_vel_type, type_);  
    p_cdc_tx_message->data.pos_val_rpd[MEM_INDEX_ID(id)].rkd = kd_float2int(kd, pos_vel_type, type_); 
}


void motor::fresh_data(uint8_t mode, uint8_t fault, int16_t position, int16_t velocity, int16_t torque)
{
    data.mode = mode;
    data.fault = fault;
    p_msg.pos = data.position = pos_int2float(position, pos_vel_type);
    p_msg.vel = data.velocity = vel_int2float(velocity, pos_vel_type);
    p_msg.tau = data.torque = tqe_int2float(torque, type_);
    ros::Time now = ros::Time::now();
    // 将时间转换为double类型
    data.time = now.toSec();
    if(pos_limit_enable)
    {
        // 判断是否超过电机限制角度
        if(data.position > pos_upper)
        {
            ROS_ERROR("Motor %d exceed position upper limit.", id);
            pos_limit_flag = 1;
        }
        else if(data.position < pos_lower)
        {
            ROS_ERROR("Motor %d exceed position lower limit.", id);
            pos_limit_flag = -1;
        }
    }
    
    if(tor_limit_enable)
    {
        // 判断是否超过电机扭矩限制
        if(data.torque > tor_upper)
        {
            ROS_ERROR("Motor %d exceed torque upper limit.", id);
            tor_limit_flag = 1;
        }
        else if(data.torque < tor_lower)
        {
            ROS_ERROR("Motor %d exceed torque lower limit.", id);
            tor_limit_flag = -1;
        }
    }
    
    // std::cout << "test " << id << ": " << data.position << "  " << data.velocity << "  " << data.torque << std::endl;
    _motor_pub.publish(p_msg);
}


/***
 * @brief setting motor type
 * @param type correspond to  different motor type 0~null 1~5046 2~5047_36减速比 3~5047_9减速比
 */
void motor::set_motor_type(size_t type)
{
    type_ = static_cast<motor_type>(type);
    // std::cout << "type_:" << type_ << std::endl;
}


void motor::set_motor_type(motor_type type)
{
    type_ = type;
}


int motor::get_motor_id() 
{ 
    return id; 
}


int motor::get_motor_type() 
{ 
    return type; 
}


motor_type motor::get_motor_enum_type()
{ 
    return type_; 
}


int motor::get_motor_num() 
{ 
    return num; 
}


int motor::get_motor_belong_canport() 
{ 
    return CANport_num; 
}


int motor::get_motor_belong_canboard() 
{ 
    return CANboard_num; 
}

motor_pos_val_tqe_rpd_s* motor::return_pos_val_tqe_rpd_p()
{
    return &cmd_int16_5param;
}

size_t motor::return_size_motor_pos_val_tqe_rpd_s()
{
    return sizeof(motor_pos_val_tqe_rpd_s);
}

motor_back_t* motor::get_current_motor_state()
{
    return &data;
}

std::string motor::get_motor_name()
{
    return motor_name;
}


void motor::set_version(cdc_rx_motor_version_s &v)
{
    version.id = v.id;
    version.major = v.major;
    version.minor = v.minor;
    version.patch = v.patch;

    // ROS_INFO("ID: %d, version = %d.%d.%d", version.id, version.major, version.minor, version.patch);
}


cdc_rx_motor_version_s& motor::get_version()
{
    return version;
}


void motor::print_version()
{
    ROS_INFO("ID: %d, version = %d.%d.%d", version.id, version.major, version.minor, version.patch);
}


void motor::set_type(motor_type t)
{
    type_ = t;
}
