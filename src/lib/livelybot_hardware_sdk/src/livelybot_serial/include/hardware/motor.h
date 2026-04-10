#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "../serial_struct.h"
#include <stdint.h>
#include <unordered_map>
#include "ros/ros.h"
#include "livelybot_msg/MotorState.h"


#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)

#define MEM_INDEX_ID(id) ((id) - 1)    


enum motor_type  // 注释掉的暂无力矩修正系数
{
    null = 0,
    m3536_32,
    m4538_19,
    m5046_20,
    m5047_09,
    // m5047_19,
    // m5047_20,
    // m5047_30,
    m5047_36,
    // m4438_08,
    // m4438_16,
    m4438_30,
    m4438_32,
    // m7136_07,
    // m7233_08,
    // m6056_08,
    m6056_36,
    // m3536_32,
    m5043_20,
    // m5043_35,
    m7256_35,
    // m6057_36,
    m60sg_35,
    m60bm_35,

    m5047_36_2,

    mGeneral,  // 力矩已在电机内部修正
};


const std::unordered_map<std::string, motor_type> motor_type2 =  // 注释掉的暂无力矩修正系数
{
    {"NULL", motor_type::null},
    {"3536_32", motor_type::m3536_32},
    {"4538_19", motor_type::m4538_19},
    {"5046_20", motor_type::m5046_20},
    {"5047_9", motor_type::m5047_09},
    // {"5047_19", motor_type::m5047_19},
    // {"5047_20", motor_type::m5047_20},
    // {"5047_30", motor_type::m5047_30},
    {"5047_36", motor_type::m5047_36},    // 老款5047_36力矩系数，
    // {"4438_8", motor_type::m4438_08},
    // {"4438_16", motor_type::m4438_16},
    {"4438_30", motor_type::m4438_30},
    {"4438_32", motor_type::m4438_32},
    // {"7136_7", motor_type::m7136_07},
    // {"7233_8", motor_type::m7233_08},
    // {"6056_8", motor_type::m6056_08},
    {"6056_36", motor_type::m6056_36},
    // {"3536_32", motor_type::m3536_32},
    {"5043_20", motor_type::m5043_20},
    // {"5043_35", motor_type::m5043_35},
    {"7256_35", motor_type::m7256_35},
    // {"6057_36", motor_type::m6057_36},
    {"60SG_35", motor_type::m60sg_35},
    {"60BM_35", motor_type::m60bm_35},
    {"5047_36_2", motor_type::m5047_36_2},  // 新版5047_36（目前的电机都是新款）的力矩系数，建议新算法的5047_36电机都采用此系数
    {"General", motor_type::mGeneral},  // 力矩已在电机内部修正
};


enum pos_vel_convert_type
{
    radian_2pi = 0,  // 弧度制
    angle_360,       // 角度制
    turns,           // 圈数
};

extern const std::unordered_map<std::string, motor_type> motor_type2;


class motor
{
private:
    int type, id, num, CANport_num, CANboard_num;
    ros::NodeHandle n;
    motor_back_t data;
    ros::Publisher _motor_pub;
    livelybot_msg::MotorState p_msg;
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *p_cdc_tx_message = NULL;
    int id_max = 0;
    int control_type = 0;
    pos_vel_convert_type pos_vel_type = radian_2pi; 
    bool pos_limit_enable = false; 
    float pos_upper = 0.0f;
    float pos_lower = 0.0f;
    bool tor_limit_enable = false;
    float tor_upper = 0.0f;
    float tor_lower = 0.0f;
    cdc_rx_motor_version_s version = {0};

public:
    motor_pos_val_tqe_rpd_s cmd_int16_5param;
    int pos_limit_flag = 0;     // 0 表示正常，1 表示超出上限， -1 表示超出下限
    int tor_limit_flag = 0;     // 0 表示正常，1 表示超出上限
    motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max);
    ~motor() {}

    inline int16_t pos_float2int(float in_data, uint8_t type);
    inline int16_t vel_float2int(float in_data, uint8_t type);
    inline int16_t tqe_float2int(float in_data, motor_type motor_type);
    inline float pos_int2float(int16_t in_data, uint8_t type);
    inline float vel_int2float(int16_t in_data, uint8_t type);
    inline float tqe_int2float(int16_t in_data, motor_type type);
    inline float pid_scale(float in_data, motor_type motor_type);
    inline int16_t kp_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t ki_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kd_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t int16_limit(int32_t data);


    void fresh_cmd_int16(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current);

    void position(float position);
    void velocity(float velocity);
    void torque(float torque);
    void voltage(float voltage);
    void current(float current);
    void set_motorout(int16_t t_ms);
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float Kp, float Kd);
    void pos_vel_tqe_kp_kd2(float position, float velocity, float torque, float kp, float kd);
    void pos_vel_kp_kd(float position, float velocity, float Kp, float Kd);
    void pos_vel_acc(float position, float velocity, float acc);
    void pos_vel_kp_ki_kd(float position, float velocity, float torque, float kp, float ki, float kd);

    void fresh_data(uint8_t mode, uint8_t fault, int16_t position, int16_t velocity, int16_t torque);

    int get_motor_id();
    int get_motor_type();
    motor_type get_motor_enum_type();
    int get_motor_num();
    void set_motor_type(size_t type);
    void set_motor_type(motor_type type);
    int get_motor_belong_canport();
    int get_motor_belong_canboard();
    motor_pos_val_tqe_rpd_s *return_pos_val_tqe_rpd_p();
    size_t return_size_motor_pos_val_tqe_rpd_s();
    motor_back_t *get_current_motor_state();
    std::string get_motor_name();
    cdc_rx_motor_version_s& get_version();
    void set_version(cdc_rx_motor_version_s &v);
    void print_version();
    void set_type(motor_type t);
};
#endif