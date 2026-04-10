#include "sensor_actuator_status.hpp"
#include <iostream>
#include <unistd.h>
#include <ctime>

int main(int argc, char** argv)
{
    Sensor_actuator_status status(7, 6, 5, 5); // 人形23关节
    // Sensor_actuator_status status(6, 6, 0, 0); // 小派12关节
    float imu[3] = {1.23f, 2.45f, 5.6f};
    uint8_t motor_status[23] = {1,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,1,1,0,1,0,1,0};
    uint32_t ip_addr[3] = {345735, 837438, 998877};
    uint8_t fsm_status = 0;
    float battery = 23.2f;
  
    std::clock_t delay = 0.1 * CLOCKS_PER_SEC;
    std::clock_t start = std::clock();
    while(true)
    {

        // IMU
        status.send_imu_status(true, imu);
        start = std::clock();
        while(std::clock() - start < 0.1*delay);
        // Motor state
        status.send_motor_status(motor_status);
        start = std::clock();
        while(std::clock() - start < 0.1*delay);
        // IP
        status.send_ip_addr(ip_addr, 3);
        start = std::clock();
        while(std::clock() - start < 0.1*delay);
        // Battery
        status.send_battery_volt(battery);
        start = std::clock();
        while(std::clock() - start < 0.1*delay);
        // Robot State
        status.send_fsm_state(fsm_status);
        
        start = std::clock();
        while(std::clock() - start < delay);

        std::cout << "Send" << std::endl;
    }
}