#ifndef _LIVELY_SERIAL_H_
#define _LIVELY_SERIAL_H_

#include "serial_struct.h"
#include "serial/serial.h"
#include "ros/ros.h"
#ifndef RELEASE
#include "motor.h"
#else
#include "hardware/motor.h"
#endif
#include <unordered_set>

class lively_serial
{
private:
    serial::Serial _ser;
    bool init_flag;
    std::map<int, motor *> Map_Motors_p;

    float *p_port_version = NULL;
    std::unordered_set<int> *p_motor_id = NULL;
    int *p_mode_flag = NULL;
    fun_version *p_fun_v = NULL;

public:
    lively_serial(std::string *port, uint32_t baudrate);
    ~lively_serial();

    bool error_flag;
    void send_2(cdc_tr_message_s *cdc_tr_message);
    uint16_t return_data_len(uint8_t cmd_id);
    void recv_1for6_42();
    void port_version_init(float *p);
    void port_motors_id_init(std::unordered_set<int> *_p_motor_id, int *_p_mode_flag);
    void init_map_motor(std::map<int, motor *> *_Map_Motors_p);
    void port_fun_v_init(fun_version *_p_fun_v);
    lively_serial(const lively_serial &) = delete;
    lively_serial &operator=(const lively_serial &) = delete;
    bool is_serial_error(void);
    void set_run_flag(bool flag);
    bool get_run_flag(void);
    void close(void);
};

#endif