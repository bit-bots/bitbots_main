#ifndef _LIVELY_SERIAL_H_
#define _LIVELY_SERIAL_H_

#include "serial_struct.h"
#include "serial/serial.h"
#include "hardware/motor.h"
#include <unordered_set>
#include <map>
#include <string>

/**
 * @brief Manages one physical serial port connected to a CAN board.
 *
 * Each lively_serial instance wraps a wjwwood serial::Serial object,
 * owns the send / receive protocol and routes received motor data to
 * the correct motor objects via a motor-ID map.
 *
 * Non-copyable; use pointers or std::unique_ptr.
 */
class lively_serial
{
private:
    serial::Serial _ser;
    bool init_flag;
    std::map<int, motor *> Map_Motors_p;

    float                    *p_port_version = nullptr;
    std::unordered_set<int>  *p_motor_id     = nullptr;
    int                      *p_mode_flag    = nullptr;
    fun_version              *p_fun_v        = nullptr;

public:
    bool error_flag;

    /**
     * @param port     Pointer to the device path string (e.g. "/dev/ttyACM0").
     * @param baudrate Serial baud rate (typically 4 000 000 bps).
     */
    lively_serial(std::string *port, uint32_t baudrate);
    ~lively_serial();

    /**
     * @brief Blocking receive loop — run in its own thread.
     *
     * Parses incoming 0xF7-framed packets and dispatches them to the
     * associated motor objects.  Exits when `rclcpp::ok()` returns false
     * or when the port signals an error.
     */
    void recv_1for6_42();

    /** Send a pre-filled TX frame; calculates CRC fields automatically. */
    void send_2(cdc_tr_message_s *cdc_tr_message);

    /** Connect the port firmware version pointer (written on MODE_SET_NUM reply). */
    void port_version_init(float *p);

    /** Connect the motor-ID acknowledgement set and mode flag. */
    void port_motors_id_init(std::unordered_set<int> *p_motor_id, int *p_mode_flag);

    /** Populate the motor map used by the receive parser. */
    void init_map_motor(std::map<int, motor *> *map);

    /** Connect the feature-version pointer (updated on MODE_FUN_V reply). */
    void port_fun_v_init(fun_version *p);

    bool is_serial_error() const;
    void set_run_flag(bool flag);
    bool get_run_flag() const;
    void close();

    lively_serial(const lively_serial &) = delete;
    lively_serial &operator=(const lively_serial &) = delete;
};

#endif /* _LIVELY_SERIAL_H_ */
