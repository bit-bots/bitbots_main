#ifndef _SERIAL_STRUCT_H_
#define _SERIAL_STRUCT_H_
#include <stdint.h>
#ifndef RELEASE
#include "crc16.h"
#include "crc8.h"
#else
#include "crc/crc16.h"
#include "crc/crc8.h"
#endif

#define  CDC_TR_MESSAGE_DATA_LEN    256


#define  MODE_POSITION              0X80
#define  MODE_VELOCITY              0X81
#define  MODE_TORQUE                0X82
#define  MODE_VOLTAGE               0X83
#define  MODE_CURRENT               0X84
#define  MODE_TIME_OUT              0x85

#define  MODE_POS_VEL_TQE           0X90
#define  MODE_POS_VEL_TQE_KP_KD     0X93
#define  MODE_POS_VEL_TQE_KP_KI_KD  0X98
#define  MODE_POS_VEL_KP_KD         0X9E
// #define  MODE_POS_VEL_TQE_RKP_RKD   0XA3  // 弃用
// #define  MODE_POS_VEL_RKP_RKD       0XA8  // 弃用
#define  MODE_POS_VEL_ACC           0XAD
#define  MODE_POS_VEL_TQE_KP_KD2    0XB0


#define  MODE_RESET_ZERO            0X01  // 重置电机零位
#define  MODE_CONF_WRITE            0X02  // 保存设置
#define  MODE_STOP                  0X03  // 电机停止
#define  MODE_BRAKE                 0X04  // 电机刹车
#define  MODE_SET_NUM               0X05  // 设置通道电机数量，并查询固件版本
#define  MODE_MOTOR_STATE           0X06  // 电机状态
#define  MODE_CONF_LOAD             0X07  // 还原设置（从flash重新加载设置）
#define  MODE_RESET                 0X08  // 电机重启
#define  MODE_RUNZERO               0X09  // 上电自动回零
#define  MODE_MOTOR_STATE2          0X0A  // 电机状态2(带模式和错误码)
#define  MODE_MOTOR_VERSION         0X0B  // 电机版本号
#define  MODE_FUN_V                 0X0C  // 设置功能版本号
#define  MODE_BOOTLOADER            0X0D  // 升级通讯板固件
#define  MODE_FDCAN_RESET           0X0E  // 重新初始化 FDCAN


#define COMBINE_VERSION(major, minor, patch) (((major) << 12) | ((minor) << 4) | (patch))

typedef enum 
{
    fun_v1 = 1,  // v3
    fun_v2,      // v4 电机模式和错误码
    fun_v3,      // v4 5参数可选电机不响应
    fun_v4,      // v4 1701
    fun_v5,      // v4 
    fun_v6,
    fun_v7,
    fun_v8,
    fun_v9,

    fun_vz = 99,
} fun_version;

#pragma pack(1)
typedef struct 
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
} motor_pos_val_tqe_s;

typedef struct 
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
    int16_t rkp;
    int16_t rkd;
} motor_pos_val_tqe_rpd_s;

typedef struct 
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
    int16_t kp;
    int16_t ki;
    int16_t kd;
} motor_pos_val_tqe_pid_s;

typedef struct 
{
    int16_t pos;
    int16_t val;
    int16_t rkp;
    int16_t rkd;
} motor_pos_val_rpd_s;

typedef struct 
{
    int16_t pos;
    int16_t val;
    int16_t acc;
} motor_pos_val_acc_s;

typedef struct 
{
    uint8_t id;
    int16_t pos;
    int16_t val;
    int16_t tqe;
} cdc_rx_motor_state_s;

typedef struct
{
    uint8_t id;
    uint8_t mode;
    uint8_t fault;
    int16_t pos;
    int16_t val;
    int16_t tqe;
} cdc_rx_motor_state2_s;

typedef struct 
{
    uint8_t id;
    uint8_t major;
    uint8_t minor; 
    uint8_t patch;
} cdc_rx_motor_version_s;

typedef struct 
{
    uint8_t head;
    uint8_t cmd;
    uint16_t len;
    uint8_t  crc8;
    uint16_t crc16;
} cdc_tr_message_head_data_s;


typedef struct
{
    union 
    {
        cdc_tr_message_head_data_s s;
        uint8_t data[sizeof(cdc_tr_message_head_data_s)];
    };
} cdc_tr_message_head_s;


typedef struct
{
    union
    {
        int16_t position[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t velocity[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t torque[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t voltage[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t current[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t timeout[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        motor_pos_val_tqe_s pos_val_tqe[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_tqe_s)];
        motor_pos_val_tqe_rpd_s pos_val_tqe_rpd[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_tqe_rpd_s)];
        motor_pos_val_rpd_s pos_val_rpd[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_rpd_s)];
        motor_pos_val_acc_s pos_val_acc[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_acc_s)];
        cdc_rx_motor_state_s motor_state[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_state_s)];
        cdc_rx_motor_state2_s motor_state2[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_state2_s)];
        cdc_rx_motor_version_s motor_version[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_version_s)];
        uint8_t data[CDC_TR_MESSAGE_DATA_LEN];
    };
} cdc_tr_message_data_s;


typedef struct 
{
    cdc_tr_message_head_s head;
    cdc_tr_message_data_s data;
} cdc_tr_message_s;


typedef struct motor_back_struct
{
    double time;
    uint8_t ID;
    uint8_t mode;
    uint8_t fault;
    float position;
    float velocity;
    float torque;
} motor_back_t;

#pragma pack()


#endif