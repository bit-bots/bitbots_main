#include "sensor_actuator_status.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <algorithm>

// ---------------------------------------------------------------------------
// USB VID/PID detection via Linux sysfs (replaces libserialport)
// ---------------------------------------------------------------------------
bool Sensor_actuator_status::read_usb_vid_pid(const std::string &port_path, int &vid, int &pid)
{
    // Extract the device name from the full path, e.g. "/dev/ttyACM0" -> "ttyACM0"
    const std::string devname = port_path.substr(port_path.rfind('/') + 1);

    // Walk up the sysfs device tree from /sys/class/tty/<devname>/device/ to find
    // the USB interface that exposes idVendor / idProduct.
    const std::string base = "/sys/class/tty/" + devname + "/device/";

    auto try_read = [](const std::string &path, int &out) -> bool {
        std::ifstream f(path);
        if (!f.is_open()) { return false; }
        std::string s;
        std::getline(f, s);
        try { out = std::stoi(s, nullptr, 16); return true; } catch (...) { return false; }
    };

    // Check current level, then one and two levels up (../  ../../)
    for (const std::string &prefix : {base, base + "../", base + "../../"}) {
        int v = 0, p = 0;
        if (try_read(prefix + "idVendor", v) && try_read(prefix + "idProduct", p)) {
            vid = v;
            pid = p;
            return true;
        }
    }
    return false;
}

Sensor_actuator_status::Sensor_actuator_status(int can1_num, int can2_num, int can3_num, int can4_num)
{
    this->motor_status.can1_num = can1_num;
    this->motor_status.can2_num = can2_num;
    this->motor_status.can3_num = can3_num;
    this->motor_status.can4_num = can4_num;
    this->suc_flag = 0;

    if (std::string(OLED_UART_DEV_NAME).find("/dev/ttyACM") == 0) {
        // Dynamic USB serial device: scan ports and check VID/PID
        const int target_vid = 0x0483;  // STMicroelectronics
        const int target_pid = 0x5740;  // Virtual COM Port
        std::vector<std::string> ports = this->list_serial_ports(OLED_UART_DEV_NAME);
        for (const std::string &port : ports) {
            int vid = 0, pid = 0;
            if (read_usb_vid_pid(port, vid, pid) && vid == target_vid && pid == target_pid) {
                printf("OLED serial port: %s\n", port.c_str());
                this->_ser.setPort(port);
                break;
            }
        }
    } else {
        this->_ser.setPort(OLED_UART_DEV_NAME);
    }

    this->_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    this->_ser.setTimeout(to);

    try {
        this->_ser.open();
    } catch (const std::exception &) {
        // Serial port may not be connected during development; continue without it.
    }

    if (this->_ser.isOpen()) {
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

    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 14;
    this->send_buff[3] = 0x10;  // IMU packet type
    this->send_buff[4] = static_cast<unsigned char>(this->imu_status.imu_con_sta);
    memcpy(&this->send_buff[5], rpy, 12);
    this->send_buff[17] = 0x66;
    this->send_buff[18] = 0x47;
    this->send_buff[19] = 0x74;

    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_motor_status(unsigned char *motor_status_data)
{
    const int total_motors = this->motor_status.can1_num + this->motor_status.can2_num +
                             this->motor_status.can3_num + this->motor_status.can4_num;

    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = static_cast<unsigned char>(3 + total_motors);
    this->send_buff[3] = 0x11;  // Motor packet type

    this->send_buff[4] = static_cast<unsigned char>(
        (this->motor_status.can1_num & 0x0F) | ((this->motor_status.can2_num & 0x0F) << 4));
    this->send_buff[5] = static_cast<unsigned char>(
        (this->motor_status.can3_num & 0x0F) | ((this->motor_status.can4_num & 0x0F) << 4));
    memcpy(&this->send_buff[6], motor_status_data, static_cast<size_t>(total_motors));

    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_ip_addr(unsigned int *ip_data, unsigned char len)
{
    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = static_cast<unsigned char>(len * 4 + 1);
    this->send_buff[3] = 0x12;  // IP packet type
    memcpy(&this->send_buff[4], ip_data, len * 4);
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_battery_volt(float volt)
{
    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 5;
    this->send_buff[3] = 0x13;  // Voltage packet type
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
    this->send_buff[2] = 5;
    this->send_buff[3] = 0x14;  // FSM state packet type
    memcpy(&this->send_buff[4], &fsm_state, 4);
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    this->_ser.write(this->send_buff, this->send_buff[2] + 6);
}

void Sensor_actuator_status::send_imu_actuator_status(bool imu_exist, float *rpy, unsigned char *m_status)
{
    send_imu_status(imu_exist, rpy);
    send_motor_status(m_status);
}

std::vector<std::string> Sensor_actuator_status::list_serial_ports(const std::string &full_prefix)
{
    const std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
    const std::string prefix    = full_prefix.substr(full_prefix.rfind('/') + 1);

    std::vector<std::string> serial_ports;
    DIR *directory = opendir(base_path.c_str());
    if (!directory) {
        std::cerr << "Could not open directory: " << base_path << std::endl;
        return serial_ports;
    }

    struct dirent *entry;
    while ((entry = readdir(directory)) != nullptr) {
        const std::string name = entry->d_name;
        if (name.find(prefix) == 0) {
            serial_ports.push_back(base_path + name);
        }
    }
    closedir(directory);
    std::sort(serial_ports.begin(), serial_ports.end());
    return serial_ports;
}
