/*
odom -> baselink
walking (X, Y, Z, rZ)
imu (rX, rY)
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>
#include <std_msgs/Time.h>

class OdometryFuser
{
public:
    OdometryFuser();
private:
    sensor_msgs::Imu imu_data;
    nav_msgs::Odometry odom_data;
    ros::Time imu_update_time;
    ros::Time odom_update_time;

    void imuCallback(const sensor_msgs::Imu msg);
    void odomCallback(const nav_msgs::Odometry msg);
};

OdometryFuser::OdometryFuser()
{
    //setup tf listener and broadcaster as class members
    
    ros::NodeHandle n("~");

    tf2::Quaternion dummy_orientation;
    dummy_orientation.setRPY(0, 0, 0);
    odom_data.pose.pose.orientation = tf2::toMsg(dummy_orientation);
    imu_data.orientation = tf2::toMsg(dummy_orientation);

    ros::Subscriber imu_subscriber = n.subscribe("/imu/data", 1, &OdometryFuser::imuCallback, this);
    ros::Subscriber odom_subscriber = n.subscribe("/foo/bar", 1, &OdometryFuser::odomCallback, this);


    //static tf2_ros::TransformBroadcaster br;
    ros::Rate r(200.0);
    while(ros::ok())
    {
        ros::spinOnce();
        
        ros::Duration imu_delta_t = ros::Time::now() - imu_update_time;

        bool imu_active = true;

        int msg_rate = 1;

        if (imu_delta_t.toSec() > 0.01)
        {
            ROS_WARN_THROTTLE(msg_rate, "IMU message outdated!");
            imu_active = false;
        }

        ros::Duration odom_delta_t = ros::Time::now() - odom_update_time;

        bool odom_active = false;

        if (odom_delta_t.toSec() > 0.01)
        {
            ROS_WARN_THROTTLE(msg_rate, "Odom message outdated!");
            odom_active = false;
        }
        

        if (imu_active || odom_active)
        {
            double placeholder, imu_roll, imu_pitch, walking_yaw;

            nav_msgs::Odometry new_odom = odom_data;
            
            tf2::Quaternion imu_orientation;
            tf2::fromMsg(imu_data.orientation, imu_orientation);
            tf2::Matrix3x3 imu_rotation_matrix(imu_orientation);
            imu_rotation_matrix.getRPY(imu_roll, imu_pitch, placeholder);

            tf2::Quaternion odom_orientation;
            tf2::fromMsg(new_odom.pose.pose.orientation, odom_orientation);

            tf2::Matrix3x3 odom_rotation_matrix(odom_orientation);
            odom_rotation_matrix.getRPY(placeholder, placeholder, walking_yaw);

            std::cout << imu_roll << " " << imu_pitch << " " << walking_yaw << "\n\n";

            tf2::Quaternion new_orientation;
            new_orientation.setRPY(imu_roll, imu_pitch, walking_yaw);

            new_odom.pose.pose.orientation = tf2::toMsg(new_orientation);
            
            std::cout << new_odom;
        } else {
            ROS_WARN_THROTTLE(msg_rate, "No Data recived! Stop publishing...");
        }

        r.sleep();
    }
}

void OdometryFuser::imuCallback(const sensor_msgs::Imu msg)
{
    OdometryFuser::imu_data = msg;
    OdometryFuser::imu_update_time = ros::Time::now();
}

void OdometryFuser::odomCallback(const nav_msgs::Odometry msg)
{
    OdometryFuser::odom_data = msg;
    OdometryFuser::odom_update_time = ros::Time::now();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bitbots_odometry");

    OdometryFuser o;
}

