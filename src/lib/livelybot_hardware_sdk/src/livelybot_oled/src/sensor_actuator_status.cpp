#include "sensor_actuator_status.hpp"
#include <iostream>
#include <libserialport.h>
#include <dirent.h>
#include <algorithm>
Sensor_actuator_status::Sensor_actuator_status(int can1_num, int can2_num, int can3_num, int can4_num)
{
    this->motor_status.can1_num = can1_num;
    this->motor_status.can2_num = can2_num;
    this->motor_status.can3_num = can3_num;
    this->motor_status.can4_num = can4_num;
    if(OLED_UART_DEV_NAME == "/dev/ttyACM")
    {
        std::vector<std::string> ports = this->list_serial_ports(OLED_UART_DEV_NAME);
        for (const std::string& port : ports) 
        {
            if (this->serial_pid_vid(port.c_str()) > 0)
            {
                // str.push_back(port);
                printf("port: %s\n", port.c_str());
                this->_ser.setPort(port); 
            }
        }
    }
    else
    {
        this->_ser.setPort(OLED_UART_DEV_NAME); 
    }
    this->_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    this->_ser.setTimeout(to);
    try
    {
        this->_ser.open();
    }
    catch (const std::exception &e)
    {
        //
    }
    if(this->_ser.isOpen())
    {
        this->suc_flag = 1;
    }
}

Sensor_actuator_status::~Sensor_actuator_status()
{
    this->_ser.close();
}

void Sensor_actuator_status::send_imu_status(bool imu_exist, float *rpy)
{
    this->imu_status.imu_con_sta = imu_exist;
    // memcpy(this->imu_status.rpy, rpy,12);

    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 14;
    this->send_buff[3] = 0x10;      // IMU包
    this->send_buff[4] = this->imu_status.imu_con_sta;
    memcpy(&this->send_buff[5], rpy, 12);
    this->send_buff[17] = 0x66;
    this->send_buff[18] = 0x47;
    this->send_buff[19] = 0x74;

    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_motor_status(unsigned char *motor_status)
{
    // memcpy(this->motor_status.motor_status, motor_status, this->motor_status.can_nums*this->motor_status.motor_nums);

    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 3 + this->motor_status.can1_num + this->motor_status.can2_num + this->motor_status.can3_num + this->motor_status.can4_num;
    this->send_buff[3] = 0x11;      // Motor

    this->send_buff[4] = (this->motor_status.can1_num & 0x0F) | ((this->motor_status.can2_num & 0x0F) << 4);
    this->send_buff[5] = (this->motor_status.can3_num & 0x0F) | ((this->motor_status.can4_num & 0x0F) << 4);
    memcpy(&this->send_buff[6], motor_status, this->motor_status.can1_num + this->motor_status.can2_num + this->motor_status.can3_num + this->motor_status.can4_num);
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_ip_addr(unsigned int* ip_data, unsigned char len)
{
    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = len * 4 + 1;
    this->send_buff[3] = 0x12;      // IP
    memcpy(&this->send_buff[4], ip_data, len * 4);
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6); // 数据包长度+帧头两个字节
}

void Sensor_actuator_status::send_battery_volt(float volt)
{
    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 4 + 1;
    this->send_buff[3] = 0x13;      // Voltage
    memcpy(&this->send_buff[4], &volt, 4);
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_fsm_state(int32_t fsm_state)
{
    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 4 + 1;
    this->send_buff[3] = 0x14;      // fsm_state
    memcpy(&this->send_buff[4], &fsm_state, 4);
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

int Sensor_actuator_status::serial_pid_vid(const char *name)
{
    int pid, vid;
    int r = 0;
    struct sp_port *port;
    
    sp_get_port_by_name(name, &port);
    sp_open(port, SP_MODE_READ);
    if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
    {
        r = -1;
    } 
    else 
    {
        if(vid == 1155 && pid == 22339)
        {
            r = 1;
        }
        else
        {
            r = -2;
        }
    }
    // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

    // 关闭端口
    sp_close(port);
    sp_free_port(port);

    return r;
}

std::vector<std::string> Sensor_actuator_status::list_serial_ports(const std::string& full_prefix) 
{
    std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
    std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
    std::vector<std::string> serial_ports;
    DIR *directory;
    struct dirent *entry;

    directory = opendir(base_path.c_str());
    if (!directory)
    {
        std::cerr << "Could not open the directory " << base_path << std::endl;
        return serial_ports; // Return an empty vector if cannot open directory
    }

    while ((entry = readdir(directory)) != NULL)
    {
        std::string entryName = entry->d_name;
        if (entryName.find(prefix) == 0)
        { // Check if the entry name starts with the given prefix
            serial_ports.push_back(base_path + entryName);
        }
    }

    closedir(directory);

    // Sort the vector in ascending order
    std::sort(serial_ports.begin(), serial_ports.end());

    return serial_ports;
}
