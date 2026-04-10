#include <stdio.h>
#include <string.h>
#include "serial/serial.h"

#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <vector>

#define OLED_UART_DEV_NAME "/dev/ttyS4"

typedef struct
{
    bool imu_con_sta;
    float rpy[3];
}imu_status_s;

typedef struct
{
    int can1_num;
    int can2_num;
    int can3_num;
    int can4_num;
    unsigned char motor_status[40];
}motor_status_s;

class Sensor_actuator_status
{
public:
    imu_status_s imu_status;
    motor_status_s motor_status;
    serial::Serial _ser;
    int suc_flag;
    unsigned char send_buff[64];
    Sensor_actuator_status(int can1_num, int can2_num, int can3_num, int can4_num);
    ~Sensor_actuator_status();
    void send_imu_actuator_status(bool iomu_exist, float *rpy, unsigned char *m_status);
    void send_imu_status(bool imu_exist, float *rpy);
    void send_motor_status(unsigned char* m_status);
    void send_ip_addr(unsigned int* ip_data, unsigned char len);
    void send_battery_volt(float volt);
    void send_fsm_state(int32_t fsm_state);
    int serial_pid_vid(const char *name);
    std::vector<std::string> list_serial_ports(const std::string& full_prefix);
};

