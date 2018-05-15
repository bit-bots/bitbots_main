
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


class BaseFootprintBroadcaster
{
public:
    BaseFootprintBroadcaster();
private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    ros::Subscriber imu_subscriber;
    geometry_msgs::TransformStamped transform;
};


BaseFootprintBroadcaster::BaseFootprintBroadcaster()
{
    //setup tf listener and broadcaster
    //register callback
    //
    ros::NodeHandle n;

    imu_subscriber = n.subscribe("/imu/data", 1, &BaseFootprintBroadcaster::imu_callback, this);
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = 0.0;
    
    /*self.tf = TransformStamped()
        self.tf.header.frame_id = "base_link"
        self.tf.child_frame_id = "base_footprint"
        self.tf.transform.translation.x = 0.0
        self.tf.transform.translation.y = 0.0*/
}

void BaseFootprintBroadcaster::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_footprint_publisher");

	BaseFootprintBroadcaster b;

	return 0;
}
