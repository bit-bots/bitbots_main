#ifndef _SERIAL_STRUCT_H_
#define _SERIAL_STRUCT_H_
#include <stdint.h>
#include "crc/crc16.h"
#include "crc/crc8.h"

/** Maximum number of bytes in a single CAN-over-USB data payload. */
#define  CDC_TR_MESSAGE_DATA_LEN    256

/* Motor control command mode IDs */
#define  MODE_POSITION              0X80  /**< Position control mode */
#define  MODE_VELOCITY              0X81  /**< Velocity control mode */
#define  MODE_TORQUE                0X82  /**< Torque control mode */
#define  MODE_VOLTAGE               0X83  /**< Voltage control mode */
#define  MODE_CURRENT               0X84  /**< Current control mode */
#define  MODE_TIME_OUT              0x85  /**< Set communication timeout */

#define  MODE_POS_VEL_TQE           0X90  /**< Position + velocity + max torque */
#define  MODE_POS_VEL_TQE_KP_KD     0X93  /**< Position + velocity + torque + Kp + Kd */
#define  MODE_POS_VEL_TQE_KP_KI_KD  0X98  /**< Position + velocity + torque + PID gains */
#define  MODE_POS_VEL_KP_KD         0X9E  /**< Position + velocity + Kp + Kd */
#define  MODE_POS_VEL_ACC           0XAD  /**< Position + velocity + acceleration */
#define  MODE_POS_VEL_TQE_KP_KD2    0XB0  /**< Alternative position + velocity + torque + Kp + Kd encoding */

/* Configuration / utility command IDs */
#define  MODE_RESET_ZERO            0X01  /**< Reset motor zero position */
#define  MODE_CONF_WRITE            0X02  /**< Save configuration to flash */
#define  MODE_STOP                  0X03  /**< Stop motor */
#define  MODE_BRAKE                 0X04  /**< Apply motor brake */
#define  MODE_SET_NUM               0X05  /**< Set number of motors on channel; query firmware version */
#define  MODE_MOTOR_STATE           0X06  /**< Query motor state (pos / vel / torque) */
#define  MODE_CONF_LOAD             0X07  /**< Restore configuration from flash */
#define  MODE_RESET                 0X08  /**< Reboot motor */
#define  MODE_RUNZERO               0X09  /**< Auto-home on power-up */
#define  MODE_MOTOR_STATE2          0X0A  /**< Extended motor state (includes mode and fault code) */
#define  MODE_MOTOR_VERSION         0X0B  /**< Query motor firmware version */
#define  MODE_FUN_V                 0X0C  /**< Set communication feature version */
#define  MODE_BOOTLOADER            0X0D  /**< Enter bootloader for firmware update */
#define  MODE_FDCAN_RESET           0X0E  /**< Re-initialize FDCAN peripheral */

/** Pack a major.minor.patch triple into a 16-bit version word. */
#define COMBINE_VERSION(major, minor, patch) (((major) << 12) | ((minor) << 4) | (patch))

/**
 * @brief Feature version enum.
 *
 * Controls which variant of the CAN-over-USB protocol is used.
 * Newer motor firmware supports additional fields and modes.
 */
typedef enum
{
    fun_v1 = 1,  /**< Baseline protocol (v3 firmware) */
    fun_v2,      /**< Extended state with mode and fault code */
    fun_v3,      /**< 5-parameter mode; unresponsive motors skip */
    fun_v4,      /**< 1701 variant */
    fun_v5,
    fun_v6,
    fun_v7,
    fun_v8,
    fun_v9,
    fun_vz = 99,
} fun_version;

#pragma pack(1)

/** 3-field motor command: position + velocity + torque (int16 each). */
typedef struct
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
} motor_pos_val_tqe_s;

/** 5-field motor command: pos + vel + torque + Kp + Kd. */
typedef struct
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
    int16_t rkp;
    int16_t rkd;
} motor_pos_val_tqe_rpd_s;

/** 6-field motor command: pos + vel + torque + Kp + Ki + Kd (full PID). */
typedef struct
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
    int16_t kp;
    int16_t ki;
    int16_t kd;
} motor_pos_val_tqe_pid_s;

/** 4-field motor command: pos + vel + Kp + Kd (no torque). */
typedef struct
{
    int16_t pos;
    int16_t val;
    int16_t rkp;
    int16_t rkd;
} motor_pos_val_rpd_s;

/** 3-field motor command: pos + vel + acceleration. */
typedef struct
{
    int16_t pos;
    int16_t val;
    int16_t acc;
} motor_pos_val_acc_s;

/** Single-motor feedback from the CAN board (basic state). */
typedef struct
{
    uint8_t id;
    int16_t pos;
    int16_t val;
    int16_t tqe;
} cdc_rx_motor_state_s;

/** Single-motor feedback including operational mode and fault code. */
typedef struct
{
    uint8_t id;
    uint8_t mode;
    uint8_t fault;
    int16_t pos;
    int16_t val;
    int16_t tqe;
} cdc_rx_motor_state2_s;

/** Motor firmware version information. */
typedef struct
{
    uint8_t id;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} cdc_rx_motor_version_s;

/** Frame header fields (packed into 7 bytes). */
typedef struct
{
    uint8_t  head;    /**< Start-of-frame byte: always 0xF7 */
    uint8_t  cmd;     /**< Command / mode ID */
    uint16_t len;     /**< Payload length in bytes */
    uint8_t  crc8;    /**< CRC-8 over [cmd, len(2 bytes)] */
    uint16_t crc16;   /**< CRC-CCITT over the payload */
} cdc_tr_message_head_data_s;

typedef struct
{
    union
    {
        cdc_tr_message_head_data_s s;
        uint8_t data[sizeof(cdc_tr_message_head_data_s)];
    };
} cdc_tr_message_head_s;

/** Payload union covering all possible command data layouts. */
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
        motor_pos_val_tqe_s       pos_val_tqe    [CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_tqe_s)];
        motor_pos_val_tqe_rpd_s   pos_val_tqe_rpd[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_tqe_rpd_s)];
        motor_pos_val_rpd_s       pos_val_rpd    [CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_rpd_s)];
        motor_pos_val_acc_s       pos_val_acc    [CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_val_acc_s)];
        cdc_rx_motor_state_s      motor_state    [CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_state_s)];
        cdc_rx_motor_state2_s     motor_state2   [CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_state2_s)];
        cdc_rx_motor_version_s    motor_version  [CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_version_s)];
        uint8_t data[CDC_TR_MESSAGE_DATA_LEN];
    };
} cdc_tr_message_data_s;

/** Complete CAN-over-USB frame: header + payload. */
typedef struct
{
    cdc_tr_message_head_s head;
    cdc_tr_message_data_s data;
} cdc_tr_message_s;

/** Decoded motor feedback stored in the motor object. */
typedef struct motor_back_struct
{
    double  time;      /**< Timestamp (steady_clock seconds) of the last update */
    uint8_t ID;        /**< Motor CAN ID */
    uint8_t mode;      /**< Current operational mode */
    uint8_t fault;     /**< Fault code (0 = no fault) */
    float   position;  /**< Joint position (rad) */
    float   velocity;  /**< Joint velocity (rad/s) */
    float   torque;    /**< Output torque (N·m) */
} motor_back_t;

#pragma pack()

#endif /* _SERIAL_STRUCT_H_ */
