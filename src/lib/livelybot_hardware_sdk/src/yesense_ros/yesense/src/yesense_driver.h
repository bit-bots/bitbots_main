#include <ros/ros.h>
#include <ros/assert.h>
#include <serial/serial.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

// #include "yesense_imu/YesenseIMUSetting.h"
#include "yesense_imu/YesenseImuAllData.h"
#include "yesense_imu/YesenseImuStatus.h"
#include "yesense_imu/YesenseImuSensorData.h"
#include "yesense_imu/YesenseImuCmdResp.h"

#include "analysis_data.h"
#include <iostream>

#define ENBALE_DEBUG_OUTPUT 1
#define ENABLE_SERIAL_INPUT 1

#define DATA_BUF_SIZE 1024

namespace yesense{

//IMU Data protocal
/*--------------------------------------------------------------------------------------------------------------
* 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
* crc校验从TID开始到payload data的最后一个字节
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
* 输入协议为：header1(0x59) + header2(0x53) + class(1B) + id(3-bit) + len(13-bit) + message(Nbytes) + ck1(1B) + ck2(1B)
* crc校验从class开始到message的最后一个字节
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
    YesenseDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~YesenseDriver();

    void run();
    void setParam();

    void publish_imu(const protocol_info_t &imu_data);
protected:
    void initSerial();
    void _spin();
    void spin();
    
    void update_position_by_gps(const protocol_info_t &imu_data, geometry_msgs::PoseStamped &pose);

    // 产品信息相关0x00
    void onProductionInformationQuery(const std_msgs::Int8::ConstPtr& msg);
    
    //复位所有参数
    void onResetAllParam(const std_msgs::UInt8::ConstPtr& msg);

    // 波特率相关 0x02
    void onBaudrateQuery(const std_msgs::Empty::ConstPtr& msg);
    void onBaudrateSetting(const std_msgs::UInt8::ConstPtr& msg);

    // 频率相关 0x03
    void onFrequencyQuery(const std_msgs::Empty::ConstPtr& msg);
    void onFrequencySetting(const std_msgs::UInt8::ConstPtr& msg);

    // 输出内容相关 0x04
    void onOutputContentQuery(const std_msgs::Empty::ConstPtr& msg);
    void onOutputContentSetting(const std_msgs::UInt8::ConstPtr& msg);

    // 标准参数设置相关0x05
    void onStandardParamQuery(const std_msgs::UInt8::ConstPtr& msg);
    void onStandardParamSetting(const std_msgs::UInt8::ConstPtr& msg);
    
    // 模式设置相关0x4D
    void onModeSettingQuery(const std_msgs::UInt8::ConstPtr& msg);
    void onModeSettingSetting(const std_msgs::UInt8::ConstPtr& msg);

    // NMEA输出设置相关
    void onNmeaQuery(const std_msgs::Empty::ConstPtr& msg);
    void onNmeaSetting(const std_msgs::UInt8::ConstPtr& msg);

    //void onExecYesenseCmd(const std_msgs::String::ConstPtr& msg);

    void on_gyro_bias_estimate(const std_msgs::String::ConstPtr& msg);

    int serial_pid_vid(const char *name);

private:
    std::string port_;                      //串口端口
	int baudrate_;                          //波特率
    serial::Serial serial_;                 //串口实例
    ros::NodeHandle nh_,nh_private_;

    ros::Subscriber sub_product_info_;
    ros::Subscriber sub_reset_param_;
    ros::Subscriber sub_baudrate_request_, sub_baudrate_setting_;
    ros::Subscriber sub_frequency_request_, sub_frequency_setting_;
    ros::Subscriber sub_output_content_request_, sub_output_content_setting_;
    ros::Subscriber sub_standard_request_, sub_standard_setting_;
    ros::Subscriber sub_mode_request_, sub_mode_setting_;
    ros::Subscriber sub_nmea_request_, sub_nmea_setting_;
    //ros::Subscriber sub_cmd_exec_;

    ros::Subscriber sub_gyro_bias_estimate_;

    ros::Publisher imu_pub_, imu_pose_pub_, imu_marker_pub_, imu_path_pub_;
    ros::Publisher imu_data_pub_, imu_all_data_pub_,
        imu_gps_data_pub_, imu_gnss_data_pub_,
        imu_status_pub_, imu_sensor_data_pub_;
    ros::Publisher pub_cmd_exec_resp_;

    sensor_msgs::Imu g_imu_;
    std::string tf_parent_frame_id_;
	std::string tf_frame_id_;
	std::string frame_id_;
	double time_offset_in_seconds_;
	bool broadcast_tf_;
	double linear_acceleration_stddev_;
	double angular_velocity_stddev_;
	double orientation_stddev_;


    // RingBuffer环形存储区，用于存储从串口中读出的数据
    std::string data_;
    boost::mutex m_mutex_;  
    const int buffer_size_;
    boost::shared_ptr<boost::circular_buffer<char> > data_buffer_ptr_;

    //检校码
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

    //查询参数返回值相关
    boost::mutex m_response_mutex_;  
    bool wait_response_flag_;
    bool check_respose_flag_;
    int error_respose_cnt_;
    uint8_t length_low_;

    uint8_t param_class_;
    uint8_t param_id_;

    std::string param_prev_topic_id_;
    std::string param_prev_topic_cmd_;

    //数据处理线程
    boost::thread deseralize_thread_;
};

}
