#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "../serial_struct.h"
#include <stdint.h>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include "livelybot_msg/msg/motor_state.hpp"

#define my_2pi (6.28318530717f)
#define my_pi  (3.14159265358f)

/** Map a 1-based motor ID to its 0-based memory index. */
#define MEM_INDEX_ID(id) ((id) - 1)

/**
 * @brief Supported motor model types.
 *
 * Each entry corresponds to a specific torque / position conversion calibration.
 * Use `mGeneral` for motors whose torque correction is done internally by the firmware.
 */
enum motor_type
{
    null = 0,
    m3536_32,
    m4538_19,
    m5046_20,
    m5047_09,
    m5047_36,
    m4438_30,
    m4438_32,
    m6056_36,
    m5043_20,
    m7256_35,
    m60sg_35,
    m60bm_35,
    m5047_36_2,   /**< Newer 5047_36 variant with revised torque coefficients */
    mGeneral,     /**< Generic: torque correction is handled inside the motor firmware */
};

/** String-to-motor_type lookup used when parsing the YAML robot configuration. */
const std::unordered_map<std::string, motor_type> motor_type2 =
{
    {"NULL",      motor_type::null},
    {"3536_32",   motor_type::m3536_32},
    {"4538_19",   motor_type::m4538_19},
    {"5046_20",   motor_type::m5046_20},
    {"5047_9",    motor_type::m5047_09},
    {"5047_36",   motor_type::m5047_36},
    {"4438_30",   motor_type::m4438_30},
    {"4438_32",   motor_type::m4438_32},
    {"6056_36",   motor_type::m6056_36},
    {"5043_20",   motor_type::m5043_20},
    {"7256_35",   motor_type::m7256_35},
    {"60SG_35",   motor_type::m60sg_35},
    {"60BM_35",   motor_type::m60bm_35},
    {"5047_36_2", motor_type::m5047_36_2},
    {"General",   motor_type::mGeneral},
};

/** Unit convention used for position / velocity conversion. */
enum pos_vel_convert_type
{
    radian_2pi = 0,  /**< Radians (full rotation = 2π) */
    angle_360,       /**< Degrees (full rotation = 360) */
    turns,           /**< Turns (full rotation = 1) */
};

extern const std::unordered_map<std::string, motor_type> motor_type2;

/**
 * @brief Represents a single motor on a CAN port.
 *
 * Handles bidirectional communication for one motor: encoding float commands
 * into the wire protocol and decoding raw feedback into physical units.
 * Also publishes per-motor state on a ROS 2 topic.
 */
class motor
{
private:
    int type, id, num, CANport_num, CANboard_num;
    rclcpp::Node::SharedPtr node_;
    motor_back_t data;
    rclcpp::Publisher<livelybot_msg::msg::MotorState>::SharedPtr motor_pub_;
    livelybot_msg::msg::MotorState p_msg;
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *p_cdc_tx_message = nullptr;
    int id_max = 0;
    int control_type = 0;
    pos_vel_convert_type pos_vel_type = radian_2pi;
    bool pos_limit_enable = false;
    float pos_upper = 0.0f;
    float pos_lower = 0.0f;
    bool tor_limit_enable = false;
    float tor_upper = 0.0f;
    float tor_lower = 0.0f;
    cdc_rx_motor_version_s version = {};

    /* Unit-conversion helpers — all inlined for performance */
    inline int16_t pos_float2int(float in_data, uint8_t type);
    inline int16_t vel_float2int(float in_data, uint8_t type);
    inline int16_t tqe_float2int(float in_data, motor_type motor_type);
    inline float   pos_int2float(int16_t in_data, uint8_t type);
    inline float   vel_int2float(int16_t in_data, uint8_t type);
    inline float   tqe_int2float(int16_t in_data, motor_type type);
    inline float   pid_scale(float in_data, motor_type motor_type);
    inline int16_t kp_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t ki_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kd_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t int16_limit(int32_t data);

public:
    motor_pos_val_tqe_rpd_s cmd_int16_5param;
    int pos_limit_flag = 0;  /**< Position limit status: 0=ok, 1=upper exceeded, -1=lower exceeded */
    int tor_limit_flag = 0;  /**< Torque limit status:   0=ok, 1=upper exceeded */

    float default_kp_ = 0.01f;
    float default_kd_ = 0.01f;
    float default_max_torque_ = 1.0f;
    float default_velocity_ = 1.0f;

    /**
     * @param motor_num  1-based motor index within the port.
     * @param port_num   1-based CAN port index.
     * @param board_num  1-based CAN board index.
     * @param p_msg      Pointer to the shared TX message buffer for this port.
     * @param id_max     Highest motor ID on this port (used to size the buffer).
     * @param node       Shared pointer to the ROS 2 node for logging and publishing.
     */
    motor(int motor_num, int port_num, int board_num,
          cdc_tr_message_s *p_msg, int id_max,
          rclcpp::Node::SharedPtr node);
    ~motor() {}

    /**
     * @brief Update the TX buffer with a fully specified command.
     *
     * The active sub-command is selected by the node parameter `robot.control_type`.
     */
    void fresh_cmd_int16(float position, float velocity, float torque,
                         float kp, float ki, float kd,
                         float acc, float voltage, float current);

    /** @name Single-axis command setters */
    /** @{ */
    void position(float position);
    void velocity(float velocity);
    void torque(float torque);
    void voltage(float voltage);
    void current(float current);
    /** @} */

    /** Set per-motor communication timeout (ms). */
    void set_motorout(int16_t t_ms);

    /** @name Combined command setters */
    /** @{ */
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float kp, float kd);
    void pos_vel_tqe_kp_kd2(float position, float velocity, float torque, float kp, float kd);
    void pos_vel_kp_kd(float position, float velocity, float kp, float kd);
    void pos_vel_acc(float position, float velocity, float acc);
    void pos_vel_kp_ki_kd(float position, float velocity, float torque, float kp, float ki, float kd);
    /** @} */

    /**
     * @brief Update stored feedback from raw CAN data and publish to ROS 2.
     *
     * Called by lively_serial on every received frame. Checks position and
     * torque limits and sets the corresponding limit flags.
     */
    void fresh_data(uint8_t mode, uint8_t fault,
                    int16_t position, int16_t velocity, int16_t torque);

    int         get_motor_id();
    int         get_motor_type();
    motor_type  get_motor_enum_type();
    int         get_motor_num();
    void        set_motor_type(size_t type);
    void        set_motor_type(motor_type type);
    int         get_motor_belong_canport();
    int         get_motor_belong_canboard();
    motor_pos_val_tqe_rpd_s *return_pos_val_tqe_rpd_p();
    size_t      return_size_motor_pos_val_tqe_rpd_s();
    motor_back_t *get_current_motor_state();
    std::string  get_motor_name();
    cdc_rx_motor_version_s& get_version();
    void set_version(cdc_rx_motor_version_s &v);
    void print_version();
    void set_type(motor_type t);
};

#endif /* _MOTOR_H_ */
