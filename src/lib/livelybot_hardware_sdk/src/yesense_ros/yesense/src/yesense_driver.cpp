#include "yesense_driver.h"
#include <map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <libserialport.h>
#include <dirent.h>
#include <algorithm>

namespace yesense{

YesenseDriver::YesenseDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private)
    , port_("/dev/ttyS7")
    , baudrate_(460800)
    , buffer_size_(4096)
    , wait_response_flag_(false)
    , check_respose_flag_(false)
    , error_respose_cnt_(0)
    , mode_(0)
    , configured_(false)
{
    nh_private_.param("yesense_port",port_,port_);
    nh_private_.param("yesense_baudrate",baudrate_,baudrate_);

    nh_private_.param<std::string>("tf_parent_frame_id", tf_parent_frame_id_, "imu_base");
	nh_private_.param<std::string>("tf_frame_id", tf_frame_id_, "imu_link");
	nh_private_.param<std::string>("frame_id", frame_id_, "imu_link");
	nh_private_.param<double>("time_offset_in_seconds", time_offset_in_seconds_, 0.0);
	nh_private_.param<bool>("broadcast_tf", broadcast_tf_, true);
	nh_private_.param<double>("linear_acceleration_stddev", linear_acceleration_stddev_, 0.0);
	nh_private_.param<double>("angular_velocity_stddev", angular_velocity_stddev_, 0.0);
	nh_private_.param<double>("orientation_stddev", orientation_stddev_, 0.0);

    // sensor_msgs::Imu g_imu;
	g_imu_.header.frame_id = frame_id_;

	g_imu_.linear_acceleration_covariance[0] = linear_acceleration_stddev_;
	g_imu_.linear_acceleration_covariance[4] = linear_acceleration_stddev_;
	g_imu_.linear_acceleration_covariance[8] = linear_acceleration_stddev_;

	g_imu_.angular_velocity_covariance[0] = angular_velocity_stddev_;
	g_imu_.angular_velocity_covariance[4] = angular_velocity_stddev_;
	g_imu_.angular_velocity_covariance[8] = angular_velocity_stddev_;

	g_imu_.orientation_covariance[0] = orientation_stddev_;
	g_imu_.orientation_covariance[4] = orientation_stddev_;
	g_imu_.orientation_covariance[8] = orientation_stddev_;

    // 数据缓冲区
    data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char> >(new boost::circular_buffer<char>(buffer_size_));

    // 读取串口数据所需的变量
    index_    = 0;
    mode_     = 0;
    bytes_    = 0;
    checksum_ = 0;
    

    initSerial();


    /*********     参数设置   ********/

    //产品信息相关
    sub_product_info_ = nh_.subscribe<std_msgs::Int8>("production_query",1,&YesenseDriver::onProductionInformationQuery,this);

    //波特率相关
    sub_baudrate_request_ = nh_.subscribe<std_msgs::Empty>("baudrate_query",1,&YesenseDriver::onBaudrateQuery,this);
    sub_baudrate_setting_ = nh_.subscribe<std_msgs::UInt8>("baudrate_setting",1,&YesenseDriver::onBaudrateSetting,this);

    //频率相关
    sub_frequency_request_ = nh_.subscribe<std_msgs::Empty>("freequency_query",1,&YesenseDriver::onFrequencyQuery,this);
    sub_frequency_setting_ = nh_.subscribe<std_msgs::UInt8>("freequency_setting",1,&YesenseDriver::onFrequencySetting,this);

    //输出内容相关
    sub_output_content_request_ = nh_.subscribe<std_msgs::Empty>("output_content_query",1,&YesenseDriver::onOutputContentQuery,this);
    sub_output_content_setting_ = nh_.subscribe<std_msgs::UInt8>("output_content_setting",1,&YesenseDriver::onOutputContentSetting,this);

    sub_standard_request_ = nh_.subscribe<std_msgs::UInt8>("standard_param_query",1,&YesenseDriver::onStandardParamQuery,this);
    sub_standard_setting_ = nh_.subscribe<std_msgs::UInt8>("standard_param_setting",1,&YesenseDriver::onStandardParamSetting,this);

    sub_mode_request_ = nh_.subscribe<std_msgs::UInt8>("mode_query",1,&YesenseDriver::onModeSettingQuery,this);
    sub_mode_setting_ = nh_.subscribe<std_msgs::UInt8>("mode_setting",1,&YesenseDriver::onModeSettingSetting,this);

    sub_nmea_request_ = nh_.subscribe<std_msgs::Empty>("nmea_query",1,&YesenseDriver::onNmeaQuery,this);
    sub_nmea_setting_ = nh_.subscribe<std_msgs::UInt8>("nmea_setting",1,&YesenseDriver::onNmeaSetting,this);

    sub_gyro_bias_estimate_ = nh_.subscribe<std_msgs::String>("yesense/gyro_bias_estimate", 1, &YesenseDriver::on_gyro_bias_estimate,this);

    // data subscribe
    imu_pub_        = nh_.advertise<sensor_msgs::Imu>("imu/data", 2);
    // imu_pub_        = nh_.advertise<sensor_msgs::Imu>("imu/data", 100);

    imu_pose_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("pose", 100);
	imu_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("imu/marker", 100);
	imu_data_pub_ = nh_.advertise<yesense_imu::YesenseImuAllData>("imu/original_data", 100);
	imu_path_pub_ = nh_.advertise<nav_msgs::Path>("imu/paths", 100);

	imu_all_data_pub_ = nh_.advertise<yesense_imu::YesenseImuAllData>("yesense/all_data", 100);
	imu_gnss_data_pub_ = nh_.advertise<yesense_imu::YesenseImuGnssData>("yesense/gnss_data", 100);
	imu_gps_data_pub_ = nh_.advertise<yesense_imu::YesenseImuGpsData>("yesense/gps_data", 100);
    imu_status_pub_ = nh_.advertise<yesense_imu::YesenseImuStatus>("yesense/imu_status", 100);
    imu_sensor_data_pub_ = nh_.advertise<yesense_imu::YesenseImuSensorData>("yesense/sensor_data", 100);
    pub_cmd_exec_resp_ = nh_.advertise<yesense_imu::YesenseImuCmdResp>("yesense/command_resp", 100);

    // initSerial();

    deseralize_thread_ = boost::thread(boost::bind(&YesenseDriver::_spin,this));
}

YesenseDriver::~YesenseDriver()
{
    ROS_INFO("Close yesense device.");
    if(serial_.isOpen())
    {
        serial_.close();
    }
    data_buffer_ptr_.reset();

    configured_ = false;
    deseralize_thread_.join();
}

void YesenseDriver::run()
{   
    try 
    {
        ros::Rate rate(100000);

        while(ros::ok())
        {
            //read data from serial
            if (serial_.available())
            {
                data_ = serial_.read(serial_.available());
                // ROS_INFO("Read data size: %ld",data_.length());
                
                {
                    boost::mutex::scoped_lock lock(m_mutex_);  
                
                    for(int i=0;i<data_.length();i++)
                    {
                        data_buffer_ptr_->push_back(data_[i]);
                    }
                }
            
            }

            ros::spinOnce();  
            rate.sleep();
        }

        ROS_WARN("ROS Exited !");
    } 
    catch (std::exception &err) 
    {
        ROS_ERROR("error in 'run' function, msg: %s", err.what());
    }    
}

int YesenseDriver::serial_pid_vid(const char *name)
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
        if (pid == 0x5543 && vid == 0x5953)
        {
            r = 1;
        }
    }
    // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

    // 关闭端口
    sp_close(port);
    sp_free_port(port);

    return r;
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
            // bool flag = false;

            // std::vector<std::string> ports = list_serial_ports(port_);
            // for (const std::string& port : ports) 
            // {
            //     if (serial_pid_vid(port.c_str()) > 0)
            //     {
            //         ROS_INFO("IMU serial port:%s, rate:%d", port.c_str(), baudrate_);
            //         port_ = port;
            //         flag = true;
            //         break;
            //     }
            // }
            // if (flag == false)
            // {
            //     ROS_ERROR("Cannot find the IMU serial port number, please check if the USB connection is normal");
            //     exit(-1);
            // }

            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            ROS_INFO("Unable to open serial port: %s ,Trying again in 5 seconds.",serial_.getPort().c_str());
            ros::Duration(5).sleep();
        }
    }
    
    if (serial_.isOpen())
    {
        ROS_INFO("Serial port: %s initialized and opened.", serial_.getPort().c_str());

        configured_ = true;
    }
}

void YesenseDriver::onProductionInformationQuery(const std_msgs::Int8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);
    if(msg->data == 1) //查询软件版本信息
    {
        //header
        buffer[0] = 0x59;
        buffer[1] = 0x53;

        //class
        buffer[2]    = PORDUCTION_INFO;
        
        // id & length
        unsigned short id_length = 0;

        //设置id 为 0
        // id_length |= 0 << 0;

        //长度设置
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
        //查询产品信息
        //header
        buffer[0] = 0x59;
        buffer[1] = 0x53;

        //class
        buffer[2]    = PORDUCTION_INFO;

        // id & length
        unsigned short id_length = 0;

        //长度设置
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
        int size = serial_.write(buffer.get(),8);

        //暂时不考虑写入不成功的情况
        if(size == 8)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//
}

/*
 *查询当前波特率的值
*/
void YesenseDriver::onBaudrateQuery(const std_msgs::Empty::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = BAUDRATE;
    
    // id & length
    unsigned short id_length = 0;

    //设置id 为 0（查询）
    // id_length |= 0 << 0;

    //长度设置为0
    // id_length |= (1 << 3);

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
        int size = serial_.write(buffer.get(),7);
        
        if(size == 7)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//
}

/*
 *设置波特率
*/
void YesenseDriver::onBaudrateSetting(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = BAUDRATE;
    
    // id & length
    unsigned short id_length = 0;

    //设置id  (最高位表示写入到内存或者flash)
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     //设置到memery
    else
        id_length |= 1 << 1;     //设置到flash

    //长度设置为1
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
        int size = serial_.write(buffer.get(),8);
        if(size == 8)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

/*
 *查询输出频率
*/
void YesenseDriver::onFrequencyQuery(const std_msgs::Empty::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_FREEQUENCY;
    
    // id & length
    unsigned short id_length = 0;

    //设置id 为 0（查询）
    // id_length |= 0 << 0;

    //长度设置为0
    // id_length |= (1 << 3);

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
        int size = serial_.write(buffer.get(),7);
        if(size == 7)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//


}

/*
 *设置输出频率
*/
void YesenseDriver::onFrequencySetting(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_FREEQUENCY;
    
    // id & length
    unsigned short id_length = 0;

    //设置id 
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     //设置到memery    id设置为0
    else 
        id_length |= 1 << 1;     //设置到flash     id设置为1

    //长度设置为1
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    //低4位为参数值
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
        int size = serial_.write(buffer.get(),8);
        if(size == 8)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

/*
 *查询输出内容
*/
void YesenseDriver::onOutputContentQuery(const std_msgs::Empty::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_CONTENT;

    // id & length
    unsigned short id_length = 0;

    //设置id 为 0（查询）
    // id_length |= 0 << 0;

    //长度设置为0
    // id_length |= (1 << 3);

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
        int size = serial_.write(buffer.get(),7);

        if(size == 7)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

/*
 *设置输出内容
*/
void YesenseDriver::onOutputContentSetting(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = OUTPUT_CONTENT;

    // id & length
    unsigned short id_length = 0;

     //设置id 
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     //设置到memery    id设置为0
    else 
        id_length |= 1 << 1;     //设置到flash     id设置为1

    //长度设置为2
    id_length |= (1 << 4);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    //低4位为参数值
    uint8_t data_type = (msg->data) & 0x0f;
    if(data_type == 0x00)
    {
        //全部不输出
        buffer[5] = 0x00;
        buffer[6] = 0x00;
    }
    else if (data_type == 0x01)
    {
        //输出加计、陀螺、磁、欧拉、四元素
        buffer[5] = 0xf8;
        buffer[6] = 0x00;
    }
    else if(data_type == 0x02)
    {
        //输出位置、速度、UTC、加计、陀螺、磁、欧拉、四元素
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
        int size = serial_.write(buffer.get(),9);

        if(size == 9)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

 // 标准参数设置相关0x05
void YesenseDriver::onStandardParamQuery(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = STANDARD_PARAM;

    // id & length
    unsigned short id_length = 0;

    //设置id 为 0（查询）
    // id_length |= 0 << 0;

    //长度设置为1
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    if(msg->data == 1)
        buffer[5] = 0x03;    //陀螺用户零偏
    else if(msg->data ==2)
        buffer[5] = 0x81;    //读静态阈值

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
        int size = serial_.write(buffer.get(),8);

        if(size == 8)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

//标准参数设置0x05
void YesenseDriver::onStandardParamSetting(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    int pkg_length = 0;
    
    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = STANDARD_PARAM;

    // id & length
    unsigned short id_length = 0;

    //设置id 
    if(msg->data >> 7 & 0x01 )
    {
        //设置到memery 
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
        //设置到flash     id设置为2
        uint8_t mode = msg->data & 0x0f;
        if(mode == 1)
        {
            //姿态角设置为0
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
            //航向设置为0
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
            //陀螺用户零偏差置0
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
        int size = serial_.write(buffer.get(),pkg_length);
        
        if(size == pkg_length)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//


}

// 模式设置相关0x4D
void YesenseDriver::onModeSettingQuery(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = MODE_SETTING;

    // id & length
    unsigned short id_length = 0;

    //长度设置为1
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
        int size = serial_.write(buffer.get(),8);

        if(size == 8)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

//0x4D
void YesenseDriver::onModeSettingSetting(const std_msgs::UInt8::ConstPtr& msg)
{
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

    //header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    //class
    buffer[2]    = MODE_SETTING;

    // id & length
    unsigned short id_length = 0;
    
    //设置id 
    if(msg->data >> 7 & 0x01 )
        id_length |= 1 << 0;     //设置到memery    id设置为0
    else 
        id_length |= 1 << 1;     //设置到flash     id设置为1

    //长度设置为2
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
        int size = serial_.write(buffer.get(),9);
        if(size == 9)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
#endif//

}

// NMEA输出设置相关0x4E
void YesenseDriver::onNmeaQuery(const std_msgs::Empty::ConstPtr& msg)
{
    
}


void YesenseDriver::onNmeaSetting(const std_msgs::UInt8::ConstPtr& msg)
{
    
}


void YesenseDriver::on_gyro_bias_estimate(const std_msgs::String::ConstPtr& msg)
{
#define BIAS_EST_ON  "\x59\x53\x4D\x11\x00\x51\x01\xB0\x68"
#define BIAS_EST_OFF "\x59\x53\x4D\x11\x00\x51\x02\xB1\x69"
#define BIAS_EST_GET "\x59\x53\x4D\x08\x00\x51\xA6\x9D"

    std::string cmd = boost::to_lower_copy(msg.get()->data);
    std::string err_msg = "";

    uint8_t *ys_cmd   = NULL;
    size_t ys_cmd_len = 0;

    param_prev_topic_id_  = "yesense/gyro_bias_estimate";
    param_prev_topic_cmd_ = cmd;

    if (cmd == "enable") 
    {
        ys_cmd = (uint8_t *)BIAS_EST_ON;
        ys_cmd_len = sizeof(BIAS_EST_ON);
    } 
    else if (cmd == "disable") 
    {
        ys_cmd = (uint8_t *)BIAS_EST_OFF;
        ys_cmd_len = sizeof(BIAS_EST_OFF);
    }
    else if (cmd == "query") 
    {
        ys_cmd = (uint8_t *)BIAS_EST_GET;
        ys_cmd_len = sizeof(BIAS_EST_GET);
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
        if (size != ys_cmd_len)
        {
            err_msg = "serial write failed !, req_size != writed_size !";
            goto fail;
        }
        
        {
            boost::mutex::scoped_lock lock(m_response_mutex_);
            wait_response_flag_ = true;
            param_class_ = ys_cmd[2];
            param_id_    = ys_cmd[3] & 0x07;
        }
    }

    return;

fail:
    yesense_imu::YesenseImuCmdResp resp_msg;
    resp_msg.id  = param_prev_topic_id_;
    resp_msg.cmd = param_prev_topic_cmd_;
    resp_msg.msg = err_msg;
    resp_msg.success = false;
    pub_cmd_exec_resp_.publish(resp_msg);
}

// run any ys command
/*void YesenseDriver::onExecYesenseCmd(const std_msgs::String::ConstPtr& msg)
{
    std::vector<std::string> sList;

    boost::split(sList, msg.get()->data, boost::is_any_of(" "), boost::token_compress_on);

    if (sList.size() == 0) 
        return;

    size_t bufferSize = sList.size();
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[bufferSize]);

    for (int i = 0; i < bufferSize; i++)
    {
        const char *str = sList[i].c_str();
        char *endPtr = NULL;
        buffer[i] = (uint8_t)strtol(str, &endPtr, 16);
    }

    std::cout << "tx -> '";
    for(int i = 0; i < bufferSize; i++) printf("%02X ", buffer[i]);
    std::cout << "'" << std::endl;

    if(serial_.isOpen())
    {
        int size = serial_.write(buffer.get(), bufferSize);

        if (size == bufferSize)
        {
            {
                boost::mutex::scoped_lock lock(m_response_mutex_);
                wait_response_flag_ = true;

                param_class_ = buffer[2];
                param_id_    = (buffer[3] & 0x07);
            }
        }
    }
}*/

void YesenseDriver::_spin() 
{
    try 
    {
        this->spin();
    } 
    catch (std::exception &err)
    {
        ROS_ERROR("error in 'spin', msg: %s", err.what());
    }    
}

void YesenseDriver::spin()
{
    ros::Rate rate(100000);

    uint8_t data = 0x00;
    uint8_t prev_data = 0x00;

    uint16_t tid = 0x00;
    uint16_t prev_tid = 0x00;

    uint32_t gps_header_sum;

    while(configured_)
    {
        while(!data_buffer_ptr_->empty())
        {
            // ROS_INFO("Buffer remain size: %d",data_buffer_ptr_->size());
            
            {
                boost::mutex::scoped_lock lock(m_mutex_);  
                data = uint8_t(data_buffer_ptr_->front());
                data_buffer_ptr_->pop_front();
            }
            
            if (mode_ == MODE_MESSAGE)            /* message data being recieved */
            {
                ck1_ += data;
                ck2_ += ck1_;

                // ROS_INFO("ck_1: 0x%02x, ck_2: 0x%02x", ck1_, ck2_);
                /* if (prev_data == 0x59 && data == 0x53) 
                {
                    ROS_WARN("Found '0x59 0x53' header in message !, TID: %d", tid);
                } */

                prev_data = data; // save prev data
                
                ROS_ASSERT_MSG(index_ < DATA_BUF_SIZE, "'index_=%d' out of range !", index_);
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
                    // last_msg_timeout_time = c_time + SERIAL_MSG_TIMEOUT;
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
                ROS_ASSERT_MSG(gps_buf_index < DATA_BUF_SIZE, "'gps_buf_index=%d' out of range !", gps_buf_index);
                
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
                    // ROS_ERROR("*************           New frame begin            **************");
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

                //判断是否为参数的返回值
                {
                    boost::mutex::scoped_lock lock(m_response_mutex_);
                    if(wait_response_flag_)
                    {
                        //maybe this byte is class id
                        if(param_class_ == data)
                        {
                            ROS_INFO("Almost param response");
                            check_respose_flag_ = true;
                        }
                        else
                        {
                            ROS_INFO("Not param response");

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
                    ROS_INFO("Frame losed: prev_TID: %d, cur_TID: %d", prev_tid, tid);
                }

                prev_tid = tid;

                //判断是否为参数的返回值
                if(check_respose_flag_)
                {
                    boost::mutex::scoped_lock lock(m_response_mutex_);
                    if(wait_response_flag_)
                    {
                        //maybe this byte is class id
                        uint8_t id = data & 0x07;
                        length_low_ = data;
                        if(param_id_ == id)
                        {
                            ROS_INFO("Double check param response");
                            check_respose_flag_ = true;
                        }
                        else
                        {
                            ROS_INFO("Double not param response");
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
                    //长度为13-bit
                    bytes_ = (length_low_ | data << 8) >> 3;
                    ROS_INFO("package length: %d",bytes_);
                }
                else
                {
                    bytes_ = data;
                }
                
                // ROS_ASSERT_MSG(bytes_ != 0, "package size is 0 !");
                
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
                // ROS_INFO("*********    ck1: %02X, ck2: %02X   ************",ck1_,ck2_);
                if(ck1_ == data)
                {
                    mode_++;
                }
                else
                {
                    // ROS_WARN("Error checksum L, ck_1: 0x%02x, data: 0x%02x", ck1_, data);

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
                //检查是否为参数设置的返回值
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
                        yesense_imu::YesenseImuCmdResp msg;

                        msg.id  = param_prev_topic_id_;
                        msg.cmd = param_prev_topic_cmd_;
                        
                        for (int i = 0; i < index_; i++)
                        {
                            msg.data.push_back(message_in_[i]);
                        }
                        
                        msg.success = index_ > 0 ? (index_ == 1 ? (message_in_[0] == 0) : true) : false;
                        msg.msg = msg.success ? "ok" : "fail";

                        pub_cmd_exec_resp_.publish(msg);
                    }

                    {
                        boost::mutex::scoped_lock lock(m_response_mutex_);
                        wait_response_flag_ = false;
                        check_respose_flag_ = false;
                        error_respose_cnt_  = 0;
                    }
                    
                    continue;
                }


                if(ck2_ == data)
                {
                    // ROS_WARN("We Received A Vaild Data Pack !");

                    // 解析数据
                    unsigned short pos = 0;
                    int payload_len = index_;
                    payload_data_t *payload = NULL;
                    unsigned char ret = 0xff;

                    while(payload_len > 0)
                    {
                        payload = (payload_data_t *)(message_in_ + pos);
                        
                        // payload is invalid 
                        ROS_ASSERT_MSG(pos < DATA_BUF_SIZE, "message_in_ 'pos' out of range, payload size: %d !", payload_len);
                        
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
                    ROS_WARN("Error checksum H !, TID: %d", tid);
                }

                ck1_ = 0;
                ck2_ = 0;
                index_ = 0;
                mode_ = 0;
                bytes_ = 0;
            }   
        }
        
        rate.sleep();
    }
}

#define EARTH_RADIUS 6378.137 //地球半径
#define Angle_To_Rad(x) (((x) * 3.141592653589793) / 180.0)

void YesenseDriver::update_position_by_gps(const protocol_info_t &imu_data, geometry_msgs::PoseStamped &pose) 
{
    // initial gps location
    static double initial_lon, initial_lat, initial_alt;
    
    // update initial gps location at first invoke
    if(initial_lon == 0 || initial_lat == 0 || initial_alt == 0) 
    {
        ROS_INFO("update initial gps location !");
        
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
    // publish imu message
    g_imu_.header.stamp = ros::Time::now();

    g_imu_.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        imu_data.roll / 180.0 * M_PI, imu_data.pitch / 180.0 * M_PI,
        imu_data.yaw / 180.0 * M_PI);
    // std::cout << imu_data.roll << ", " << imu_data.pitch << ", " << imu_data.yaw << ", " << std::endl;

    g_imu_.angular_velocity.x = imu_data.angle_x / 180.0 * M_PI;
    g_imu_.angular_velocity.y = imu_data.angle_y / 180.0 * M_PI;
    g_imu_.angular_velocity.z = imu_data.angle_z / 180.0 * M_PI;

    g_imu_.linear_acceleration.x = imu_data.accel_x;
    g_imu_.linear_acceleration.y = imu_data.accel_y;
    g_imu_.linear_acceleration.z = imu_data.accel_z;

    imu_pub_.publish(g_imu_);
    
    // update imu pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "imu_link";
    pose.header.stamp = g_imu_.header.stamp;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = g_imu_.orientation.w;
    pose.pose.orientation.x = g_imu_.orientation.x;
    pose.pose.orientation.y = g_imu_.orientation.y;
    pose.pose.orientation.z = g_imu_.orientation.z;
    
    // update_position_by_gps(imu_data, pose);
    imu_pose_pub_.publish(pose);
    
    // update imu marker
    visualization_msgs::Marker marker_info;
    marker_info.header.frame_id = "imu_link";
    marker_info.header.stamp = ros::Time::now();

    // set namespace and id
    marker_info.ns = "basic_shapes";
    marker_info.id = 0;

    // set marker's shape
    marker_info.type = visualization_msgs::Marker::CUBE;

    // set action: ADD
    marker_info.action = visualization_msgs::Marker::ADD;

    // set imu pose
    marker_info.pose.position.x = pose.pose.position.x;
    marker_info.pose.position.y = pose.pose.position.y;
    marker_info.pose.position.z = pose.pose.position.z;
    marker_info.pose.orientation.x = g_imu_.orientation.x;
    marker_info.pose.orientation.y = g_imu_.orientation.y;
    marker_info.pose.orientation.z = g_imu_.orientation.z;
    marker_info.pose.orientation.w = g_imu_.orientation.w;

    // set scale, unit: m
    marker_info.scale.x = 4.0;
    marker_info.scale.y = 4.0;
    marker_info.scale.z = 3.0;

    // set color
    marker_info.color.r = 0.2f;
    marker_info.color.g = 0.2f;
    marker_info.color.b = 0.2f;
    marker_info.color.a = 1.0;

    marker_info.lifetime = ros::Duration();
    
    imu_marker_pub_.publish(marker_info);
    
    // publish original sensor data
    yesense_imu::YesenseImuAllData raw_data;

    raw_data.temperature = imu_data.imu_temp;
    
    raw_data.accel.linear.x = imu_data.accel_x;
    raw_data.accel.linear.y = imu_data.accel_y;
    raw_data.accel.linear.z = imu_data.accel_z;

    raw_data.accel.angular.x = imu_data.angle_x;
    raw_data.accel.angular.y = imu_data.angle_y;
    raw_data.accel.angular.z = imu_data.angle_z;
    
    raw_data.eulerAngle.roll = imu_data.roll;
    raw_data.eulerAngle.pitch = imu_data.pitch;
    raw_data.eulerAngle.yaw = imu_data.yaw;
    
    raw_data.quaternion.q0 = imu_data.quaternion_data0;
    raw_data.quaternion.q1 = imu_data.quaternion_data1;
    raw_data.quaternion.q2 = imu_data.quaternion_data2;
    raw_data.quaternion.q3 = imu_data.quaternion_data3;

    raw_data.location.longtidue = imu_data.longtidue;
    raw_data.location.latitude = imu_data.latitude;
    raw_data.location.altidue = imu_data.altidue;

    // raw_data.status.raw_code = imu_data.status;
    raw_data.status.fusion_status = imu_data.status & 0x0f;
    raw_data.status.gnss_status = (imu_data.status >> 4) & 0x0f;

    // gnss data
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

    for (std::map<uint32_t, std::string>::iterator inter = gsp_raw.begin();
         inter != gsp_raw.end(); 
         inter++)
    {
        raw_data.gps.raw_data.push_back(inter->second);
    }

    imu_data_pub_.publish(raw_data);
    imu_all_data_pub_.publish(raw_data);
    imu_sensor_data_pub_.publish((yesense_imu::YesenseImuSensorData&)raw_data);

    imu_gps_data_pub_.publish(raw_data.gps);
    imu_gnss_data_pub_.publish(raw_data.gnss);
    imu_status_pub_.publish(raw_data.status);
    
    // publish paths
    static nav_msgs::Path paths;
    paths.header.frame_id = "imu_link";
    paths.header.stamp = ros::Time::now();
    paths.poses.push_back(pose);
    imu_path_pub_.publish(paths);
}

}
