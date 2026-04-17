#include "yesense_driver.h"
#include <map>
#include <vector>
#include <algorithm>
#include <array>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <chrono>
#include <dirent.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace yesense{

YesenseDriver::YesenseDriver(rclcpp::Node::SharedPtr node)
    : node_(node)
    , buffer_size_(4096)
    , wait_response_flag_(false)
    , check_respose_flag_(false)
    , error_respose_cnt_(0)
    , mode_(0)
    , configured_(false)
{
    param_listener_ = std::make_shared<yesense_imu::ParamListener>(node_);
    params_ = param_listener_->get_params();

    g_imu_.header.frame_id = params_.frame_id;

    g_imu_.linear_acceleration_covariance[0] = params_.linear_acceleration_stddev;
    g_imu_.linear_acceleration_covariance[4] = params_.linear_acceleration_stddev;
    g_imu_.linear_acceleration_covariance[8] = params_.linear_acceleration_stddev;

    g_imu_.angular_velocity_covariance[0] = params_.angular_velocity_stddev;
    g_imu_.angular_velocity_covariance[4] = params_.angular_velocity_stddev;
    g_imu_.angular_velocity_covariance[8] = params_.angular_velocity_stddev;

    g_imu_.orientation_covariance[0] = params_.orientation_stddev;
    g_imu_.orientation_covariance[4] = params_.orientation_stddev;
    g_imu_.orientation_covariance[8] = params_.orientation_stddev;

    // data ring-buffer
    data_buffer_ptr_ = std::make_unique<std::deque<char>>();

    // variables for reading serial data
    index_    = 0;
    mode_     = 0;
    bytes_    = 0;
    checksum_ = 0;


    initSerial();


    /*********     Parameter settings   ********/

    // product information queries
    sub_product_info_ = node_->create_subscription<std_msgs::msg::Int8>(
        "production_query", 1,
        std::bind(&YesenseDriver::onProductionInformationQuery, this, std::placeholders::_1));

    // baud rate
    sub_baudrate_request_ = node_->create_subscription<std_msgs::msg::Empty>(
        "baudrate_query", 1,
        std::bind(&YesenseDriver::onBaudrateQuery, this, std::placeholders::_1));
    sub_baudrate_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "baudrate_setting", 1,
        std::bind(&YesenseDriver::onBaudrateSetting, this, std::placeholders::_1));

    // output frequency
    sub_frequency_request_ = node_->create_subscription<std_msgs::msg::Empty>(
        "freequency_query", 1,
        std::bind(&YesenseDriver::onFrequencyQuery, this, std::placeholders::_1));
    sub_frequency_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "freequency_setting", 1,
        std::bind(&YesenseDriver::onFrequencySetting, this, std::placeholders::_1));

    // output content
    sub_output_content_request_ = node_->create_subscription<std_msgs::msg::Empty>(
        "output_content_query", 1,
        std::bind(&YesenseDriver::onOutputContentQuery, this, std::placeholders::_1));
    sub_output_content_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "output_content_setting", 1,
        std::bind(&YesenseDriver::onOutputContentSetting, this, std::placeholders::_1));

    sub_standard_request_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "standard_param_query", 1,
        std::bind(&YesenseDriver::onStandardParamQuery, this, std::placeholders::_1));
    sub_standard_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "standard_param_setting", 1,
        std::bind(&YesenseDriver::onStandardParamSetting, this, std::placeholders::_1));

    sub_mode_request_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "mode_query", 1,
        std::bind(&YesenseDriver::onModeSettingQuery, this, std::placeholders::_1));
    sub_mode_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "mode_setting", 1,
        std::bind(&YesenseDriver::onModeSettingSetting, this, std::placeholders::_1));

    sub_nmea_request_ = node_->create_subscription<std_msgs::msg::Empty>(
        "nmea_query", 1,
        std::bind(&YesenseDriver::onNmeaQuery, this, std::placeholders::_1));
    sub_nmea_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "nmea_setting", 1,
        std::bind(&YesenseDriver::onNmeaSetting, this, std::placeholders::_1));

    sub_gyro_bias_estimate_ = node_->create_subscription<std_msgs::msg::String>(
        "yesense/gyro_bias_estimate", 1,
        std::bind(&YesenseDriver::on_gyro_bias_estimate, this, std::placeholders::_1));

    // data publishers
    imu_pub_        = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 2);
    imu_pose_pub_   = node_->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
    imu_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("imu/marker", 100);
    imu_data_pub_   = node_->create_publisher<yesense_imu::msg::YesenseImuAllData>("imu/original_data", 100);
    imu_path_pub_   = node_->create_publisher<nav_msgs::msg::Path>("imu/paths", 100);

    imu_all_data_pub_    = node_->create_publisher<yesense_imu::msg::YesenseImuAllData>("yesense/all_data", 100);
    imu_gnss_data_pub_   = node_->create_publisher<yesense_imu::msg::YesenseImuGnssData>("yesense/gnss_data", 100);
    imu_gps_data_pub_    = node_->create_publisher<yesense_imu::msg::YesenseImuGpsData>("yesense/gps_data", 100);
    imu_status_pub_      = node_->create_publisher<yesense_imu::msg::YesenseImuStatus>("yesense/imu_status", 100);
    imu_sensor_data_pub_ = node_->create_publisher<yesense_imu::msg::YesenseImuSensorData>("yesense/sensor_data", 100);
    pub_cmd_exec_resp_   = node_->create_publisher<yesense_imu::msg::YesenseImuCmdResp>("yesense/command_resp", 100);

    deseralize_thread_ = std::thread([this]() { _spin(); });
}

YesenseDriver::~YesenseDriver()
{
    RCLCPP_INFO(node_->get_logger(), "Close yesense device.");
    if(serial_.isOpen())
    {
        serial_.close();
    }

    configured_ = false;
    cv_data_.notify_all();
    deseralize_thread_.join();

    data_buffer_ptr_.reset();
}

void YesenseDriver::run()
{
    try
    {
        while(rclcpp::ok())
        {
            size_t avail = serial_.available();
            if (avail > 0)
            {
                data_ = serial_.read(avail);

                if (!data_.empty())
                {
                    {
                        std::lock_guard<std::mutex> lock(m_mutex_);
                        if (data_buffer_ptr_->size() + data_.size() > kMaxPendingBytes)
                        {
                            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "IMU parser falling behind, dropping %zu buffered bytes",
                                data_buffer_ptr_->size());
                            data_buffer_ptr_->clear();
                        }
                        for (char c : data_)
                            data_buffer_ptr_->push_back(c);
                    }
                    cv_data_.notify_one();
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            rclcpp::spin_some(node_);
        }

        RCLCPP_WARN(node_->get_logger(), "ROS Exited !");
    }
    catch (std::exception &err)
    {
        RCLCPP_ERROR(node_->get_logger(), "error in 'run' function, msg: %s", err.what());
    }
}

int YesenseDriver::serial_pid_vid(const char *name)
{
    const std::string devname = std::string(name);
    const std::string dev = devname.substr(devname.rfind('/') + 1);
    for (const std::string &prefix : {"/sys/class/tty/" + dev + "/device/",
                                      "/sys/class/tty/" + dev + "/device/../",
                                      "/sys/class/tty/" + dev + "/device/../../"}) {
        std::ifstream vid_f(prefix + "idVendor"), pid_f(prefix + "idProduct");
        if (!vid_f.is_open() || !pid_f.is_open()) continue;
        std::string vid_s, pid_s;
        std::getline(vid_f, vid_s); std::getline(pid_f, pid_s);
        try {
            int vid = std::stoi(vid_s, nullptr, 16);
            int pid = std::stoi(pid_s, nullptr, 16);
            if (vid == 0x5953 && pid == 0x5543) return 1;
        } catch (...) {}
    }
    return -1;
}

std::vector<std::string> list_serial_ports(const std::string& full_prefix)
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

void YesenseDriver::initSerial()
{
    while (serial_.isOpen() == false)
    {
        try
        {
            serial_.setPort(params_.yesense_port);
            serial_.setBaudrate(params_.yesense_baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_INFO(node_->get_logger(), "Unable to open serial port: %s ,Trying again in 5 seconds.", serial_.getPort().c_str());
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    if (serial_.isOpen())
    {
        RCLCPP_INFO(node_->get_logger(), "Serial port: %s initialized and opened.", serial_.getPort().c_str());

        configured_ = true;
    }
}

void YesenseDriver::onProductionInformationQuery(std_msgs::msg::Int8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};
    if(msg->data == 1) // query software version
    {
        //header
        buffer[0] = 0x59;
        buffer[1] = 0x53;

        //class
        buffer[2]    = PORDUCTION_INFO;

        // id & length
        unsigned short id_length = 0;

        // set length
        id_length |= (1 << 3);

        buffer[3] = id_length & 0xff;
        buffer[4] = id_length >> 8 & 0xff;

        //data
        buffer[5] = 0x02;

        //crc check
        uint8_t ck1=0, ck2=0;
        for(int i=0;i<4;i++)
        {
            ck1 += buffer[2+i];
            ck2 += ck1;
        }

        buffer[6] = ck1;
        buffer[7] = ck2;
    }
    else if(msg->data == 2)
    {
        // query product info
        //header
        buffer[0] = 0x59;
        buffer[1] = 0x53;

        //class
        buffer[2]    = PORDUCTION_INFO;

        // id & length
        unsigned short id_length = 0;

        // set length
        id_length |= (1 << 3);

        buffer[3] = id_length & 0xff;
        buffer[4] = id_length >> 8 & 0xff;

        //data
        buffer[5] = 0x04;

        //crc check
        uint8_t ck1=0, ck2=0;
        for(int i=0;i<4;i++)
        {
            ck1 += buffer[2+i];
            ck2 += ck1;
        }

        buffer[6] = ck1;
        buffer[7] = ck2;
    }

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<8;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 8);

        // ignore partial-write failure for now
        if(size == 8)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//
}

/*
 * Query the current baud rate value
*/
void YesenseDriver::onBaudrateQuery(std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = BAUDRATE;

    // id & length
    unsigned short id_length = 0;

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<3;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[5] = ck1;
    buffer[6] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<7;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 7);

        if(size == 7)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//
}

/*
 * Set baud rate
*/
void YesenseDriver::onBaudrateSetting(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = BAUDRATE;

    // id & length
    unsigned short id_length = 0;

    // set id (MSB=1: write to RAM, MSB=0: write to flash)
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     // write to RAM
    else
        id_length |= 1 << 1;     // write to flash

    // set length to 1
    id_length |= (1 << 3);


    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    //data
    buffer[5] = (msg->data) & 0x0f;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<4;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    // crc
    buffer[6] = ck1;
    buffer[7] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<8;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 8);
        if(size == 8)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

/*
 * Query output frequency
*/
void YesenseDriver::onFrequencyQuery(std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_FREEQUENCY;

    // id & length
    unsigned short id_length = 0;

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<3;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[5] = ck1;
    buffer[6] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<7;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 7);
        if(size == 7)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//


}

/*
 * Set output frequency
*/
void YesenseDriver::onFrequencySetting(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_FREEQUENCY;

    // id & length
    unsigned short id_length = 0;

    // set id
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     // write to RAM,   id=0
    else
        id_length |= 1 << 1;     // write to flash, id=1

    // set length to 1
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    // lower 4 bits are the parameter value
    buffer[5] = (msg->data) & 0x0f;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<4;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[6] = ck1;
    buffer[7] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<8;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 8);
        if(size == 8)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

/*
 * Query output content
*/
void YesenseDriver::onOutputContentQuery(std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_CONTENT;

    // id & length
    unsigned short id_length = 0;

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<3;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[5] = ck1;
    buffer[6] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<7;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 7);

        if(size == 7)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

/*
 * Set output content
*/
void YesenseDriver::onOutputContentSetting(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_CONTENT;

    // id & length
    unsigned short id_length = 0;

     // set id
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     // write to RAM,   id=0
    else
        id_length |= 1 << 1;     // write to flash, id=1

    // set length to 2
    id_length |= (1 << 4);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    // lower 4 bits are the parameter value
    uint8_t data_type = (msg->data) & 0x0f;
    if(data_type == 0x00)
    {
        // output nothing
        buffer[5] = 0x00;
        buffer[6] = 0x00;
    }
    else if (data_type == 0x01)
    {
        // output accel, gyro, mag, euler, quaternion
        buffer[5] = 0xf8;
        buffer[6] = 0x00;
    }
    else if(data_type == 0x02)
    {
        // output position, velocity, UTC, accel, gyro, mag, euler, quaternion
        buffer[5] = 0xff;
        buffer[6] = 0x00;
    }

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<5;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[7] = ck1;
    buffer[8] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<9;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 9);

        if(size == 9)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

 // standard parameter settings 0x05
void YesenseDriver::onStandardParamQuery(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = STANDARD_PARAM;

    // id & length
    unsigned short id_length = 0;

    // set length to 1
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    if(msg->data == 1)
        buffer[5] = 0x03;    // gyro user bias
    else if(msg->data ==2)
        buffer[5] = 0x81;    // read static threshold

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<4;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[6] = ck1;
    buffer[7] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<8;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 8);

        if(size == 8)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

// standard parameter settings 0x05
void YesenseDriver::onStandardParamSetting(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    int pkg_length = 0;

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = STANDARD_PARAM;

    // id & length
    // unsigned short id_length = 0;

    // set id
    if(msg->data >> 7 & 0x01 )
    {
        // write to RAM
        buffer[3]    = 0x71;
        buffer[4]    = 0x00;

        buffer[5]     = 0x03;   buffer[6]    = 0x0C;    buffer[7]     = 0x00;    buffer[8]     = 0x00;    buffer[9]     = 0x00;   buffer[10]    = 0x00;
        buffer[11]    = 0x00;   buffer[12]   = 0x00;    buffer[13]    = 0x00;    buffer[14]    = 0x00;    buffer[15]    = 0x00;   buffer[16]    = 0x00;
        buffer[17]    = 0x00;   buffer[18]   = 0x00;

        //crc check
        uint8_t ck1=0, ck2=0;
        for(int i=0;i<17;i++)
        {
            ck1 += buffer[2+i];
            ck2 += ck1;
        }
        buffer[19]   = ck1;
        buffer[20]   = ck2;

        pkg_length = 21;
    }
    else
    {
        // write to flash, id=2
        uint8_t mode = msg->data & 0x0f;
        if(mode == 1)
        {
            // set attitude angle to 0
            buffer[3]    = 0x32;
            buffer[4]    = 0x00;

            buffer[5]     = 0x11;   buffer[6]    = 0x04;    buffer[7]     = 0x00;    buffer[8]     = 0x00;    buffer[9]     = 0x00;   buffer[10]    = 0x00;

            //crc check
            uint8_t ck1=0, ck2=0;
            for(int i=0;i<9;i++)
            {
                ck1 += buffer[2+i];
                ck2 += ck1;
            }
            buffer[11]   = ck1;
            buffer[12]   = ck2;

            pkg_length = 13;
        }
        else if(mode ==2)
        {
            // set heading to 0
            buffer[3]    = 0x22;
            buffer[4]    = 0x00;

            buffer[5]     = 0x12;   buffer[6]    = 0x02;    buffer[7]     = 0x00;    buffer[8]     = 0x00;

            //crc check
            uint8_t ck1=0, ck2=0;
            for(int i=0;i<7;i++)
            {
                ck1 += buffer[2+i];
                ck2 += ck1;
            }
            buffer[9]   = ck1;
            buffer[10]   = ck2;

            pkg_length = 11;
        }
        else if(mode == 3)
        {
            // set gyro user bias to 0
            buffer[3]    = 0x72;
            buffer[4]    = 0x00;

            buffer[5]    = 0x03;  buffer[6]     = 0x0C;  buffer[7]     = 0x00;  buffer[8]     = 0x00;   buffer[9]    = 0x00;
            buffer[10]   = 0x00;  buffer[11]    = 0x00;  buffer[12]    = 0x00;  buffer[13]    = 0x00;   buffer[14]   = 0x00;
            buffer[15]   = 0x00;  buffer[16]    = 0x00;  buffer[17]    = 0x00;  buffer[18]    = 0x00;

            //crc check
            uint8_t ck1=0, ck2=0;
            for(int i=0;i<17;i++)
            {
                ck1 += buffer[2+i];
                ck2 += ck1;
            }
            buffer[19]   = ck1;
            buffer[20]   = ck2;

            pkg_length = 21;
        }
    }

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense[YesenseDriver::onStandardParamSetting]: "<<std::endl;
    for(int i=0;i<pkg_length;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), pkg_length);

        if(size == pkg_length)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//


}

// mode settings 0x4D
void YesenseDriver::onModeSettingQuery(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = MODE_SETTING;

    // id & length
    unsigned short id_length = 0;

    // set length to 1
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    if(msg->data == 1)
        buffer[5] = 0x02;
    else if(msg->data ==2)
        buffer[5] = 0x20;
    else if(msg->data ==3)
        buffer[5] = 0x4f;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<4;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[6] = ck1;
    buffer[7] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<8;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 8);

        if(size == 8)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

// mode settings write 0x4D
void YesenseDriver::onModeSettingSetting(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::array<uint8_t, 64> buffer{};

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = MODE_SETTING;

    // id & length
    unsigned short id_length = 0;

    // set id
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     // write to RAM,   id=0
    else
        id_length |= 1 << 1;     // write to flash, id=1

    // set length to 2
    id_length |= (1 << 4);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    uint8_t mode = msg->data & 0x0f;
    if(mode == 1)
    {
        //AHRS
        buffer[5] = 0x02;
        buffer[6] = 0x01;
    }
    else if(mode ==2)
    {
        //VRU
        buffer[5] = 0x02;
        buffer[6] = 0x02;
    }
    else if(mode ==3)
    {
        //IMU
        buffer[5] = 0x02;
        buffer[6] = 0x03;
    }
    else if(mode ==4)
    {
        //GenerPOS
        buffer[5] = 0x02;
        buffer[6] = 0x04;
    }
    else if(mode ==5)
    {
        //AutoMative
        buffer[5] = 0x02;
        buffer[6] = 0x05;
    }
    else if(mode ==6)
    {
        //Data Ready
        buffer[5] = 0x4f;
        buffer[6] = 0x01;
    }
    else if(mode ==7)
    {
        //PPS
        buffer[5] = 0x4f;
        buffer[6] = 0x02;
    }
    else if(mode == 8)
    {
        //general mode
        buffer[5] = 0x20;
        buffer[6] = 0x01;
    }
     else if(mode == 9)
    {
        //quadruped robot mode
        buffer[5] = 0x20;
        buffer[6] = 0x02;
    }
     else if(mode == 10)
    {
        //
        buffer[5] = 0x50;
        buffer[6] = 0x01;
    }
    else
        return;

    //crc check
    uint8_t ck1=0, ck2=0;
    for(int i=0;i<5;i++)
    {
        ck1 += buffer[2+i];
        ck2 += ck1;
    }

    buffer[7] = ck1;
    buffer[8] = ck2;

#if(ENBALE_DEBUG_OUTPUT)
    std::cout<<"Write data to yesense: "<<std::endl;
    for(int i=0;i<9;i++)
        printf("%02X ", buffer[i]);
    std::cout<<std::endl;
#endif//

#if(ENABLE_SERIAL_INPUT)
    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.data(), 9);
        if(size == 9)
        {
            {
                std::lock_guard<std::mutex> lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

// NMEA output settings 0x4E
void YesenseDriver::onNmeaQuery(std_msgs::msg::Empty::SharedPtr /*msg*/)
{

}


void YesenseDriver::onNmeaSetting(std_msgs::msg::UInt8::SharedPtr /*msg*/)
{

}


void YesenseDriver::on_gyro_bias_estimate(std_msgs::msg::String::SharedPtr msg)
{
#define BIAS_EST_ON  "\x59\x53\x4D\x11\x00\x51\x01\xB0\x68"
#define BIAS_EST_OFF "\x59\x53\x4D\x11\x00\x51\x02\xB1\x69"
#define BIAS_EST_GET "\x59\x53\x4D\x08\x00\x51\xA6\x9D"

    std::string cmd = msg->data;
    // convert to lower case
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
    std::string err_msg = "";

    const uint8_t *ys_cmd   = NULL;
    size_t ys_cmd_len = 0;

    param_prev_topic_id_  = "yesense/gyro_bias_estimate";
    param_prev_topic_cmd_ = cmd;

    if (cmd == "enable")
    {
        ys_cmd = (const uint8_t *)BIAS_EST_ON;
        ys_cmd_len = sizeof(BIAS_EST_ON) - 1;  // exclude null terminator
    }
    else if (cmd == "disable")
    {
        ys_cmd = (const uint8_t *)BIAS_EST_OFF;
        ys_cmd_len = sizeof(BIAS_EST_OFF) - 1;
    }
    else if (cmd == "query")
    {
        ys_cmd = (const uint8_t *)BIAS_EST_GET;
        ys_cmd_len = sizeof(BIAS_EST_GET) - 1;
    }
    else
    {
        err_msg = "Invalid command: '" + cmd + "'";
        goto fail;
    }

    if(!serial_.isOpen())
    {
        err_msg = "serial port is not opened !";
        goto fail;
    }

    //
    {
        int size = serial_.write(ys_cmd, ys_cmd_len);
        if ((size_t)size != ys_cmd_len)
        {
            err_msg = "serial write failed !, req_size != writed_size !";
            goto fail;
        }

        {
            std::lock_guard<std::mutex> lock(m_response_mutex_);
            wait_response_flag_ = true;
            param_class_ = ys_cmd[2];
            param_id_    = ys_cmd[3] & 0x07;
        }
    }

    return;

fail:
    yesense_imu::msg::YesenseImuCmdResp resp_msg;
    resp_msg.id  = param_prev_topic_id_;
    resp_msg.cmd = param_prev_topic_cmd_;
    resp_msg.msg = err_msg;
    resp_msg.success = false;
    pub_cmd_exec_resp_->publish(resp_msg);
}

void YesenseDriver::_spin()
{
    try
    {
        this->spin();
    }
    catch (std::exception &err)
    {
        RCLCPP_ERROR(node_->get_logger(), "error in 'spin', msg: %s", err.what());
    }
}

void YesenseDriver::spin()
{
    std::deque<char> local_buf;

    uint8_t data = 0x00;
    uint8_t prev_data = 0x00;

    uint16_t tid = 0x00;
    uint16_t prev_tid = 0x00;

    uint32_t gps_header_sum;

    while(configured_)
    {
        // Block until data is available, then swap the entire shared buffer at once.
        // This eliminates both the 100kHz busy-wait and per-byte mutex acquisitions.
        {
            std::unique_lock<std::mutex> lock(m_mutex_);
            cv_data_.wait(lock, [this]{ return !data_buffer_ptr_->empty() || !configured_; });
            std::swap(local_buf, *data_buffer_ptr_);
        }

        while(!local_buf.empty())
        {
            data = uint8_t(local_buf.front());
            local_buf.pop_front();

            if (mode_ == MODE_MESSAGE)            /* message data being recieved */
            {
                ck1_ += data;
                ck2_ += ck1_;

                prev_data = data; // save prev data

                if (index_ >= DATA_BUF_SIZE) {
                    RCLCPP_ERROR(node_->get_logger(), "'index_=%d' out of range !", index_);
                    break;
                }
                message_in_[index_++] = data;
                bytes_--;
                if (bytes_ == 0)                  /* is message complete? if so, checksum */
                    mode_ = MODE_CHECKSUM_L;
            }
            else if (mode_ == MODE_HEADER1)
            {
                if (data == 0x59)
                {
                    mode_++;
                }
                else if(data == '$') /* is GPS msg ? */
                {
                    mode_ = MODE_GPS_RAW;
                    gps_buf_index = 0;
                    gps_header_sum = 0;
                    gps_buf[gps_buf_index++] = data;
                }
            }
            else if(mode_ == MODE_GPS_RAW)
            {
                if (gps_buf_index >= DATA_BUF_SIZE) {
                    RCLCPP_ERROR(node_->get_logger(), "'gps_buf_index=%d' out of range !", gps_buf_index);
                    mode_ = MODE_HEADER1;
                    continue;
                }

                gps_buf[gps_buf_index++] = data;

                if (isgraph(data) || data == '\r' || data == '\n')
                {
                    if (gps_buf_index <= 6)
                    {
                        if (isalpha(data))
                            gps_header_sum += data;
                        else
                            mode_ = MODE_HEADER1;
                    }

                    if (data == '\r') /* frame end, exit */
                    {
                        mode_ = MODE_HEADER1;
                        gps_buf[gps_buf_index - 1] = '\0';
                        gsp_raw[gps_header_sum] = std::string((char *)gps_buf);
                    }
                }
                else
                {
                    mode_ = MODE_HEADER1;
                }
            }
            else if(mode_ == MODE_HEADER2)
            {
                if(data == 0x53)
                {
                    ck1_ = 0;
                    ck2_ = 0;
                    index_ = 0;

                    mode_++;
                }
                else
                {
                    mode_ = 0;
                }
            }
            else if(mode_ == MODE_TID_L)
            {
                ck1_ += data;
                ck2_ += ck1_;

                // set tid low
                tid = data;

                // check whether this is a parameter response
                {
                    std::lock_guard<std::mutex> lock(m_response_mutex_);
                    if(wait_response_flag_)
                    {
                        //maybe this byte is class id
                        if(param_class_ == data)
                        {
                            RCLCPP_DEBUG(node_->get_logger(), "Almost param response");
                            check_respose_flag_ = true;
                        }
                        else
                        {
                            RCLCPP_DEBUG(node_->get_logger(), "Not param response");

                            check_respose_flag_ = false;
                            error_respose_cnt_++;

                            if (error_respose_cnt_ > 10)
                            {
                                wait_response_flag_ = false;
                                error_respose_cnt_  = 0;
                            }
                        }
                    }
                }

                mode_++;
            }
            else if(mode_ == MODE_TID_H)
            {
                ck1_ += data;
                ck2_ += ck1_;

                tid |= ((uint16_t)data) << 8;

                if(prev_tid != 0 && tid > prev_tid && prev_tid != tid - 1)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "Frame lost: prev_TID: %d, cur_TID: %d", prev_tid, tid);
                }

                prev_tid = tid;

                // check whether this is a parameter response
                if(check_respose_flag_)
                {
                    std::lock_guard<std::mutex> lock(m_response_mutex_);
                    if(wait_response_flag_)
                    {
                        //maybe this byte is class id
                        uint8_t id = data & 0x07;
                        length_low_ = data;
                        if(param_id_ == id)
                        {
                            RCLCPP_DEBUG(node_->get_logger(), "Double check param response");
                            check_respose_flag_ = true;
                        }
                        else
                        {
                            RCLCPP_DEBUG(node_->get_logger(), "Double not param response");
                            check_respose_flag_ = false;
                        }
                    }
                }
                mode_++;
            }
            else if(mode_ == MODE_LENGTH)
            {
                ck1_ += data;
                ck2_ += ck1_;

                if(check_respose_flag_)
                {
                    // length is 13-bit
                    bytes_ = (length_low_ | data << 8) >> 3;
                    RCLCPP_DEBUG(node_->get_logger(), "package length: %d", bytes_);
                }
                else
                {
                    bytes_ = data;
                }

                if(bytes_ == 0) // package length is 0, reset all and exit loop
                {
                    ck1_ = 0;
                    ck2_ = 0;
                    index_ = 0;
                    mode_ = 0;
                    bytes_ = 0;
                    break;
                }

                mode_++;
            }
            else if(mode_ == MODE_CHECKSUM_L)
            {
                if(ck1_ == data)
                {
                    mode_++;
                }
                else
                {
                    //crc check error
                    ck1_ = 0;
                    ck2_ = 0;
                    index_ = 0;
                    mode_ = 0;
                    bytes_ = 0;
                    break;
                }
            }
            else if(mode_ == MODE_CHECKSUM_H)
            {
                // check whether this is a parameter-setting response
                if(wait_response_flag_ && check_respose_flag_)
                {
                    // log Response
                    {
                        std::cout<<"Response: "<<std::endl;

                        for (int i = 0; i < index_; i++)
                        {
                            printf("%02X ", message_in_[i]);
                        }

                        std::cout<<std::endl;
                    }

                    // publish response
                    {
                        yesense_imu::msg::YesenseImuCmdResp resp_msg;

                        resp_msg.id  = param_prev_topic_id_;
                        resp_msg.cmd = param_prev_topic_cmd_;

                        for (int i = 0; i < index_; i++)
                        {
                            resp_msg.data.push_back(message_in_[i]);
                        }

                        resp_msg.success = index_ > 0 ? (index_ == 1 ? (message_in_[0] == 0) : true) : false;
                        resp_msg.msg = resp_msg.success ? "ok" : "fail";

                        pub_cmd_exec_resp_->publish(resp_msg);
                    }

                    {
                        std::lock_guard<std::mutex> lock(m_response_mutex_);
                        wait_response_flag_ = false;
                        check_respose_flag_ = false;
                        error_respose_cnt_  = 0;
                    }

                    continue;
                }


                if(ck2_ == data)
                {
                    // parse data
                    unsigned short pos = 0;
                    int payload_len = index_;
                    payload_data_t *payload = NULL;
                    unsigned char ret = 0xff;

                    while(payload_len > 0)
                    {
                        payload = (payload_data_t *)(message_in_ + pos);

                        // payload is invalid
                        if (pos >= DATA_BUF_SIZE) {
                            RCLCPP_ERROR(node_->get_logger(), "message_in_ 'pos' out of range, payload size: %d !", payload_len);
                            break;
                        }

                        ret = parse_data_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + 2);

                        if((unsigned char)0x01 == ret) // check done !
                        {
                            pos += payload->data_len + sizeof(payload_data_t);
                            payload_len -= payload->data_len + sizeof(payload_data_t);
                        }
                        else // check failed
                        {
                            pos++;
                            payload_len--;
                        }
                    }

                    publish_imu(g_output_info);
                }
                else
                {
                    //crc check error
                    RCLCPP_WARN(node_->get_logger(), "Error checksum H !, TID: %d", tid);
                }

                ck1_ = 0;
                ck2_ = 0;
                index_ = 0;
                mode_ = 0;
                bytes_ = 0;
            }
        }
    }
}

#define EARTH_RADIUS 6378.137 // Earth radius (km)
#define Angle_To_Rad(x) (((x) * 3.141592653589793) / 180.0)

void YesenseDriver::update_position_by_gps(const protocol_info_t &imu_data, geometry_msgs::msg::PoseStamped &pose)
{
    // initial gps location
    static double initial_lon, initial_lat, initial_alt;

    // update initial gps location at first invoke
    if(initial_lon == 0 || initial_lat == 0 || initial_alt == 0)
    {
        RCLCPP_INFO(node_->get_logger(), "update initial gps location !");

        initial_lon = imu_data.longtidue;
        initial_lat = imu_data.latitude;
        initial_alt = imu_data.altidue;

        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        return; // exit
    }

    // calcu relative location
    double radLat1 ,radLat2, radLong1, radLong2, delta_lat, delta_long;

    radLat1 = Angle_To_Rad(initial_lat);
    radLong1 = Angle_To_Rad(initial_lon);
    radLat2 = Angle_To_Rad(imu_data.latitude);
    radLong2 = Angle_To_Rad(imu_data.longtidue);

    // calcu x
    delta_lat = radLat2 - radLat1;
    delta_long = 0;
    double x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
    x = x*EARTH_RADIUS*1000;

    // calcu y
    delta_lat = 0;
    delta_long = radLong1  - radLong2;
    double y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
    y = y*EARTH_RADIUS*1000;

    // calcu z
    double z = imu_data.altidue - initial_alt;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
}

void YesenseDriver::publish_imu(const protocol_info_t &imu_data)
{
    // Always publish the primary IMU topic
    g_imu_.header.stamp = node_->now();

    tf2::Quaternion q;
    q.setRPY(
        imu_data.roll / 180.0 * M_PI,
        imu_data.pitch / 180.0 * M_PI,
        imu_data.yaw / 180.0 * M_PI);

    g_imu_.orientation.x = q.x();
    g_imu_.orientation.y = q.y();
    g_imu_.orientation.z = q.z();
    g_imu_.orientation.w = q.w();

    g_imu_.angular_velocity.x = imu_data.angle_x / 180.0 * M_PI;
    g_imu_.angular_velocity.y = imu_data.angle_y / 180.0 * M_PI;
    g_imu_.angular_velocity.z = imu_data.angle_z / 180.0 * M_PI;

    g_imu_.linear_acceleration.x = imu_data.accel_x;
    g_imu_.linear_acceleration.y = imu_data.accel_y;
    g_imu_.linear_acceleration.z = imu_data.accel_z;

    imu_pub_->publish(g_imu_);

    // pose — cheap, publish if subscribed
    if (imu_pose_pub_->get_subscription_count() > 0)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "imu_frame";
        pose.header.stamp = g_imu_.header.stamp;
        pose.pose.orientation = g_imu_.orientation;
        imu_pose_pub_->publish(pose);

        // path — only if subscribed; capped to avoid ever-growing serialization cost
        if (imu_path_pub_->get_subscription_count() > 0)
        {
            static nav_msgs::msg::Path paths;
            static constexpr size_t kMaxPathPoses = 1000;
            paths.header.frame_id = "imu_frame";
            paths.header.stamp = g_imu_.header.stamp;
            paths.poses.push_back(pose);
            if (paths.poses.size() > kMaxPathPoses)
                paths.poses.erase(paths.poses.begin());
            imu_path_pub_->publish(paths);
        }
    }

    // marker — only if subscribed
    if (imu_marker_pub_->get_subscription_count() > 0)
    {
        visualization_msgs::msg::Marker marker_info;
        marker_info.header.frame_id = "imu_frame";
        marker_info.header.stamp = g_imu_.header.stamp;
        marker_info.ns = "basic_shapes";
        marker_info.id = 0;
        marker_info.type = visualization_msgs::msg::Marker::CUBE;
        marker_info.action = visualization_msgs::msg::Marker::ADD;
        marker_info.pose.orientation = g_imu_.orientation;
        marker_info.scale.x = 4.0;
        marker_info.scale.y = 4.0;
        marker_info.scale.z = 3.0;
        marker_info.color.r = 0.2f;
        marker_info.color.g = 0.2f;
        marker_info.color.b = 0.2f;
        marker_info.color.a = 1.0;
        marker_info.lifetime = rclcpp::Duration(0, 0);
        imu_marker_pub_->publish(marker_info);
    }

    // raw/all-data topics — build the message only if someone is listening
    const bool need_raw = imu_data_pub_->get_subscription_count() > 0
                       || imu_all_data_pub_->get_subscription_count() > 0
                       || imu_sensor_data_pub_->get_subscription_count() > 0
                       || imu_gps_data_pub_->get_subscription_count() > 0
                       || imu_gnss_data_pub_->get_subscription_count() > 0
                       || imu_status_pub_->get_subscription_count() > 0;

    if (need_raw)
    {
        yesense_imu::msg::YesenseImuAllData raw_data;

        raw_data.temperature = imu_data.imu_temp;

        raw_data.accel.linear.x = imu_data.accel_x;
        raw_data.accel.linear.y = imu_data.accel_y;
        raw_data.accel.linear.z = imu_data.accel_z;

        raw_data.accel.angular.x = imu_data.angle_x;
        raw_data.accel.angular.y = imu_data.angle_y;
        raw_data.accel.angular.z = imu_data.angle_z;

        raw_data.euler_angle.roll = imu_data.roll;
        raw_data.euler_angle.pitch = imu_data.pitch;
        raw_data.euler_angle.yaw = imu_data.yaw;

        raw_data.quaternion.q0 = imu_data.quaternion_data0;
        raw_data.quaternion.q1 = imu_data.quaternion_data1;
        raw_data.quaternion.q2 = imu_data.quaternion_data2;
        raw_data.quaternion.q3 = imu_data.quaternion_data3;

        raw_data.location.longtidue = imu_data.longtidue;
        raw_data.location.latitude = imu_data.latitude;
        raw_data.location.altidue = imu_data.altidue;

        raw_data.status.fusion_status = imu_data.status & 0x0f;
        raw_data.status.gnss_status = (imu_data.status >> 4) & 0x0f;

        raw_data.gnss.master.utc_time.year = imu_data.gnss.utc_time.year;
        raw_data.gnss.master.utc_time.month = imu_data.gnss.utc_time.month;
        raw_data.gnss.master.utc_time.date = imu_data.gnss.utc_time.date;
        raw_data.gnss.master.utc_time.hour = imu_data.gnss.utc_time.hour;
        raw_data.gnss.master.utc_time.min = imu_data.gnss.utc_time.min;
        raw_data.gnss.master.utc_time.sec = imu_data.gnss.utc_time.sec;
        raw_data.gnss.master.utc_time.ms = imu_data.gnss.utc_time.ms;

        raw_data.gnss.master.location.longtidue = imu_data.gnss.location.lon;
        raw_data.gnss.master.location.latitude = imu_data.gnss.location.lat;
        raw_data.gnss.master.location.altidue = imu_data.gnss.location.alt;

        raw_data.gnss.master.location_error.longtidue = imu_data.gnss.location_error.lon;
        raw_data.gnss.master.location_error.latitude = imu_data.gnss.location_error.lat;
        raw_data.gnss.master.location_error.altidue = imu_data.gnss.location_error.alt;

        raw_data.gnss.master.speed = imu_data.gnss.speed;
        raw_data.gnss.master.yaw = imu_data.gnss.yaw;
        raw_data.gnss.master.status = imu_data.gnss.status;
        raw_data.gnss.master.star_cnt = imu_data.gnss.star_cnt;
        raw_data.gnss.master.p_dop = imu_data.gnss.p_dop;
        raw_data.gnss.master.site_id = imu_data.gnss.site_id;

        raw_data.gnss.slave.dual_ant_yaw = imu_data.gnss_slave.dual_ant_yaw;
        raw_data.gnss.slave.dual_ant_yaw_error = imu_data.gnss_slave.dual_ant_yaw_error;
        raw_data.gnss.slave.dual_ant_baseline_len = imu_data.gnss_slave.dual_ant_baseline_len;

        for (auto it = gsp_raw.begin(); it != gsp_raw.end(); ++it)
            raw_data.gps.raw_data.push_back(it->second);

        if (imu_data_pub_->get_subscription_count() > 0)   imu_data_pub_->publish(raw_data);
        if (imu_all_data_pub_->get_subscription_count() > 0) imu_all_data_pub_->publish(raw_data);
        if (imu_status_pub_->get_subscription_count() > 0)  imu_status_pub_->publish(raw_data.status);
        if (imu_gps_data_pub_->get_subscription_count() > 0) imu_gps_data_pub_->publish(raw_data.gps);
        if (imu_gnss_data_pub_->get_subscription_count() > 0) imu_gnss_data_pub_->publish(raw_data.gnss);

        if (imu_sensor_data_pub_->get_subscription_count() > 0)
        {
            yesense_imu::msg::YesenseImuSensorData sensor_data;
            sensor_data.temperature = raw_data.temperature;
            sensor_data.sample_timestamp = raw_data.sample_timestamp;
            sensor_data.sync_timestamp = raw_data.sync_timestamp;
            sensor_data.accel = raw_data.accel;
            sensor_data.quaternion = raw_data.quaternion;
            sensor_data.euler_angle = raw_data.euler_angle;
            sensor_data.location = raw_data.location;
            imu_sensor_data_pub_->publish(sensor_data);
        }
    }
}

}
