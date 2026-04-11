#include <rclcpp/rclcpp.hpp>
#include <yesense_imu/yesense_parameters.hpp>
#include <serial/serial.h>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>

#include "yesense_imu/msg/yesense_imu_all_data.hpp"
#include "yesense_imu/msg/yesense_imu_status.hpp"
#include "yesense_imu/msg/yesense_imu_sensor_data.hpp"
#include "yesense_imu/msg/yesense_imu_cmd_resp.hpp"
#include "yesense_imu/msg/yesense_imu_gnss_data.hpp"
#include "yesense_imu/msg/yesense_imu_gps_data.hpp"

#include "analysis_data.h"
#include <iostream>

#define ENBALE_DEBUG_OUTPUT 1
#define ENABLE_SERIAL_INPUT 1

#define DATA_BUF_SIZE 1024

namespace yesense{

//IMU Data protocal
/*--------------------------------------------------------------------------------------------------------------
* Output protocol: header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
* CRC check starts from TID through the last byte of payload data
*/
const uint8_t MODE_HEADER1            = 0;
const uint8_t MODE_HEADER2            = 1;
const uint8_t MODE_TID_L              = 2;
const uint8_t MODE_TID_H              = 3;
const uint8_t MODE_LENGTH             = 4;
const uint8_t MODE_MESSAGE            = 5;
const uint8_t MODE_CHECKSUM_L         = 6;    // checksum for msg and topic id
const uint8_t MODE_CHECKSUM_H         = 7;    // checksum for msg and topic id

// gps raw data mode
const uint8_t MODE_GPS_RAW            = 10;

//IMU Param protocal
/*--------------------------------------------------------------------------------------------------------------
* Input protocol: header1(0x59) + header2(0x53) + class(1B) + id(3-bit) + len(13-bit) + message(Nbytes) + ck1(1B) + ck2(1B)
* CRC check starts from class through the last byte of message
*/
const uint8_t    PORDUCTION_INFO   = 0x00;
const uint8_t    RESET_ALL_PARAM   = 0x01;
const uint8_t    BAUDRATE          = 0x02;
const uint8_t    OUTPUT_FREEQUENCY = 0x03;
const uint8_t    OUTPUT_CONTENT    = 0x04;
const uint8_t    STANDARD_PARAM    = 0x05;
const uint8_t    MODE_SETTING      = 0x4D;
const uint8_t    NMEA_CONTENT      = 0x4E;



typedef enum _Id{
    QUERY_STATUS  = 0x00,
    CONFIG_MEMERY = 0x01,
    CONFIG_FLASH  = 0x02
}ID;


class YesenseDriver{
public:
    explicit YesenseDriver(rclcpp::Node::SharedPtr node);
    ~YesenseDriver();

    void run();
    void setParam();

    void publish_imu(const protocol_info_t &imu_data);
protected:
    void initSerial();
    void _spin();
    void spin();

    void update_position_by_gps(const protocol_info_t &imu_data, geometry_msgs::msg::PoseStamped &pose);

    // product information queries 0x00
    void onProductionInformationQuery(std_msgs::msg::Int8::SharedPtr msg);

    // reset all parameters
    void onResetAllParam(std_msgs::msg::UInt8::SharedPtr msg);

    // baud rate 0x02
    void onBaudrateQuery(std_msgs::msg::Empty::SharedPtr msg);
    void onBaudrateSetting(std_msgs::msg::UInt8::SharedPtr msg);

    // output frequency 0x03
    void onFrequencyQuery(std_msgs::msg::Empty::SharedPtr msg);
    void onFrequencySetting(std_msgs::msg::UInt8::SharedPtr msg);

    // output content 0x04
    void onOutputContentQuery(std_msgs::msg::Empty::SharedPtr msg);
    void onOutputContentSetting(std_msgs::msg::UInt8::SharedPtr msg);

    // standard parameter settings 0x05
    void onStandardParamQuery(std_msgs::msg::UInt8::SharedPtr msg);
    void onStandardParamSetting(std_msgs::msg::UInt8::SharedPtr msg);

    // mode settings 0x4D
    void onModeSettingQuery(std_msgs::msg::UInt8::SharedPtr msg);
    void onModeSettingSetting(std_msgs::msg::UInt8::SharedPtr msg);

    // NMEA output settings
    void onNmeaQuery(std_msgs::msg::Empty::SharedPtr msg);
    void onNmeaSetting(std_msgs::msg::UInt8::SharedPtr msg);

    void on_gyro_bias_estimate(std_msgs::msg::String::SharedPtr msg);

    int serial_pid_vid(const char *name);

private:
    serial::Serial serial_;                 // serial port instance
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_product_info_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_reset_param_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_baudrate_request_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_baudrate_setting_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_frequency_request_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_frequency_setting_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_output_content_request_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_output_content_setting_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_standard_request_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_standard_setting_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_request_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_setting_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_nmea_request_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_nmea_setting_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_gyro_bias_estimate_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr imu_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr imu_path_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuAllData>::SharedPtr imu_data_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuAllData>::SharedPtr imu_all_data_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuGpsData>::SharedPtr imu_gps_data_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuGnssData>::SharedPtr imu_gnss_data_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuStatus>::SharedPtr imu_status_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuSensorData>::SharedPtr imu_sensor_data_pub_;
    rclcpp::Publisher<yesense_imu::msg::YesenseImuCmdResp>::SharedPtr pub_cmd_exec_resp_;

    sensor_msgs::msg::Imu g_imu_;
    yesense_imu::Params params_;
    std::shared_ptr<yesense_imu::ParamListener> param_listener_;


    // Ring-buffer for storing data read from the serial port
    std::string data_;
    std::mutex m_mutex_;
    const int buffer_size_;
    std::unique_ptr<std::deque<char>> data_buffer_ptr_;

    // checksum bytes
    uint8_t ck1_;
    uint8_t ck2_;

    //State machine variables for spinOnce
    int mode_;
    int bytes_;
    int index_;
    int checksum_;

    bool configured_;

    /* used for syncing the time */
    uint32_t last_sync_time;
    uint32_t last_sync_receive_time;
    uint32_t last_msg_timeout_time;

    uint8_t  message_in_[DATA_BUF_SIZE];
    uint8_t  gps_buf[DATA_BUF_SIZE];
    uint32_t gps_buf_index;
    std::map<uint32_t, std::string> gsp_raw;

    // variables related to parameter query responses
    std::mutex m_response_mutex_;
    bool wait_response_flag_;
    bool check_respose_flag_;
    int error_respose_cnt_;
    uint8_t length_low_;

    uint8_t param_class_;
    uint8_t param_id_;

    std::string param_prev_topic_id_;
    std::string param_prev_topic_cmd_;

    // data processing thread
    std::thread deseralize_thread_;
};

}
