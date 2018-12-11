/*
taken from REP120

base_footprint

The base_footprint is the representation of the robot position on the floor. 
The floor is usually the level where the supporting leg rests, 
i.e. z = min(l_sole_z, r_sole_z) where l_sole_z and r_sole_z are the left and right sole height respecitvely. 
The translation component of the frame should be the barycenter of the feet projections on the floor. 
With respect to the odom frame, the roll and pitch angles should be zero and the yaw angle should correspond to the base_link yaw angle.

Rationale: base_footprint provides a fairly stable 2D planar representation of the humanoid even while walking and swaying with the base_link.
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include "bitbots_quintic_walk/WalkingDebug.h"


class BaseFootprintBroadcaster
{
public:
    BaseFootprintBroadcaster();
private:
    geometry_msgs::TransformStamped tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Subscriber support_foot_subscriber;
    void support_foot_callback(const bitbots_quintic_walk::WalkingDebug::ConstPtr& walk_msg);
    bool is_left_support, got_support_foot;
};

BaseFootprintBroadcaster::BaseFootprintBroadcaster() : tfBuffer(ros::Duration(10.0)),  tfListener(tfBuffer)
{
    //setup tf listener and broadcaster as class members
    
    ros::NodeHandle n("~");

    tf = geometry_msgs::TransformStamped();
    
    static tf2_ros::TransformBroadcaster br;
    ros::Rate r(30.0);
    while(ros::ok())
    {
        geometry_msgs::TransformStamped tf_right, 
                                        tf_left, 
                                        support_foot, 
                                        non_support_foot, 
                                        non_support_foot_in_support_foot_frame, 
                                        base_footprint_in_support_foot_frame;

        try{
            tf_right = tfBuffer.lookupTransform("base_link", "r_sole", ros::Time::now(), ros::Duration(0.1));
            tf_left = tfBuffer.lookupTransform("base_link", "l_sole",  ros::Time::now(), ros::Duration(0.1));
            
            // check which foot is support foot (which foot is on the ground)
            if(tf_right.transform.translation.z < tf_left.transform.translation.z) {
                support_foot = tf_right;
                non_support_foot = tf_left;
            } else {
                support_foot = tf_left;
                non_support_foot = tf_right;
            }

            // get the position of the non support foot in the support frame, used for computing the barycenter
            non_support_foot_in_support_foot_frame = tfBuffer.lookupTransform(support_foot.child_frame_id, 
                                                                              non_support_foot.child_frame_id,
                                                                              support_foot.header.stamp,
                                                                              ros::Duration(0.1));

            geometry_msgs::TransformStamped support_to_base_link = tfBuffer.lookupTransform(support_foot.header.frame_id,  
                                                                                            support_foot.child_frame_id, 
                                                                                            support_foot.header.stamp);

            geometry_msgs::PoseStamped base_footprint, base_footprint_in_base_link;

            // z at ground leven (support foot height)
            base_footprint.pose.position.z = 0;
            // x and y at barycenter of feet projections on the ground
            base_footprint.pose.position.x = non_support_foot_in_support_foot_frame.transform.translation.x/2;
            base_footprint.pose.position.y = non_support_foot_in_support_foot_frame.transform.translation.y/2;


            // get yaw from base link
            double yaw;
            yaw = tf2::getYaw(support_to_base_link.transform.rotation);
            
            // pitch and roll from support foot, yaw from base link
            tf2::Quaternion rotation;
            rotation.setRPY(0.0,0.0,yaw);
            base_footprint.pose.orientation = tf2::toMsg(rotation);

            // transform the position and orientation of the base footprint into the base_link frame
            tf2::doTransform(base_footprint, base_footprint_in_base_link, support_to_base_link);

            // set the broadcasted transform to the position and orientation of the base footprint
            tf.header.stamp = base_footprint_in_base_link.header.stamp;
            tf.header.frame_id = base_footprint_in_base_link.header.frame_id;
            tf.child_frame_id = "base_footprint";
            tf.transform.translation.x = base_footprint_in_base_link.pose.position.x;
            tf.transform.translation.y = base_footprint_in_base_link.pose.position.y;
            tf.transform.translation.z = base_footprint_in_base_link.pose.position.z;
            tf.transform.rotation = base_footprint_in_base_link.pose.orientation;
            br.sendTransform(tf);

        } catch(...){
            //ROS_WARN_THROTTLE(2, "Can not publish base_footprint, check your tf tree");
            continue;
        }

        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_footprint");

    BaseFootprintBroadcaster b;
}

