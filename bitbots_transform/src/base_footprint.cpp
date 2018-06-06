

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


class BaseFootprintBroadcaster
{
public:
    BaseFootprintBroadcaster();
private:
    //void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    //ros::Subscriber imu_subscriber;
    geometry_msgs::TransformStamped tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    //bool imu;
};

BaseFootprintBroadcaster::BaseFootprintBroadcaster() : tfBuffer(ros::Duration(10.0)),  tfListener(tfBuffer)
{
    //setup tf listener and broadcaster
    //register callback

    ros::NodeHandle n("~");
    //std::string imu_topic;
    //n.getParam("imu_topic", imu_topic);
    //imu_subscriber = n.subscribe(imu_topic, 1, &BaseFootprintBroadcaster::imu_callback, this);

    

    tf = geometry_msgs::TransformStamped();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = 0.0;

    static tf2_ros::TransformBroadcaster br;
    ros::Rate r(30.0);
    while(ros::ok())
    {
        /*ros::spinOnce();
        if(!imu)
        {
            tf.header.stamp = ros::Time::now();
        }*/

        geometry_msgs::TransformStamped tf_right, tf_left;
        try{
            tf_right = tfBuffer.lookupTransform("base_link", "r_sole", ros::Time::now(), ros::Duration(0.1));
            tf_left = tfBuffer.lookupTransform("base_link", "l_sole",  ros::Time::now(), ros::Duration(0.1));
        }catch(...){
            continue;
        }
        // check which foot is support foot (which foot is on the ground)
        if(tf_right.transform.translation.z < tf_left.transform.translation.z) {
            tf.transform.translation.z = tf_right.transform.translation.z;
            tf.transform.rotation = tf_right.transform.rotation;
            tf.header.stamp = tf_right.header.stamp;
        } else {
            tf.transform.translation.z = tf_left.transform.translation.z;
            tf.transform.rotation = tf_left.transform.rotation;
            tf.header.stamp = tf_left.header.stamp;
        }

        tf.transform.translation.y = (tf_left.transform.translation.y + tf_right.transform.translation.y)/2;

        br.sendTransform(tf);
        r.sleep();

    }
    ros::spin();
}

/*void BaseFootprintBroadcaster::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{

    imu = true;

    if((imu_msg->orientation.x + imu_msg->orientation.y + imu_msg->orientation.z + imu_msg->orientation.w) == 0.0)
    {
        ROS_WARN("imu message has no orientation set, use complementary filter to get one ");
        return;
    }
    tf.transform.rotation = imu_msg->orientation;


}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_footprint");

    BaseFootprintBroadcaster b;
}

