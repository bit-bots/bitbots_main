#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Char.h>
#include <std_msgs/Time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


class ConvenienceFramesBroadcaster
{
public:
    ConvenienceFramesBroadcaster();
private:
    geometry_msgs::TransformStamped tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    bool is_left_support, got_support_foot;
    void supportFootCallback(const std_msgs::Char msg);
};

ConvenienceFramesBroadcaster::ConvenienceFramesBroadcaster() : tfBuffer(ros::Duration(10.0)),  tfListener(tfBuffer)
{
    //setup tf listener and broadcaster as class members
    
    ros::NodeHandle n("~");
    got_support_foot = false;
    ros::Subscriber walking_support_foot_subscriber = n.subscribe("/walk_support_state", 1, &ConvenienceFramesBroadcaster::supportFootCallback, this, ros::TransportHints().tcpNoDelay());
    ros::Subscriber dynamic_kick_support_foot_subscriber = n.subscribe("/dynamic_kick_support_state", 1, &ConvenienceFramesBroadcaster::supportFootCallback, this, ros::TransportHints().tcpNoDelay());

    tf = geometry_msgs::TransformStamped();
    
    static tf2_ros::TransformBroadcaster br;
    ros::Rate r(30.0);
    while(ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::TransformStamped tf_right, 
                                        tf_left, 
                                        support_foot, 
                                        non_support_foot, 
                                        non_support_foot_in_support_foot_frame, 
                                        base_footprint_in_support_foot_frame,
                                        front_foot,
                                        back_foot;

        try{
            tf_right = tfBuffer.lookupTransform("base_link", "r_sole", ros::Time::now(), ros::Duration(0.1));
            tf_left = tfBuffer.lookupTransform("base_link", "l_sole",  ros::Time::now(), ros::Duration(0.1));

            // compute support foot
            if(got_support_foot)
            {
                if(is_left_support)
                {
                    support_foot = tf_left;
                    non_support_foot = tf_right;
                }
                else
                {
                    support_foot = tf_right;
                    non_support_foot = tf_left;
                }
            }
            else
            {
                // check which foot is support foot (which foot is on the ground)
                if(tf_right.transform.translation.z < tf_left.transform.translation.z) {
                    support_foot = tf_right;
                    non_support_foot = tf_left;
                } else {
                    support_foot = tf_left;
                    non_support_foot = tf_right;
                }
            }

            // check with foot is in front
            if(tf_right.transform.translation.x < tf_left.transform.translation.x){
                front_foot = tf_left;
                back_foot = tf_right;
            }else{
                front_foot = tf_right;
                back_foot = tf_left;
            }

            // get the position of the non support foot in the support frame, used for computing the barycenter
            non_support_foot_in_support_foot_frame = tfBuffer.lookupTransform(support_foot.child_frame_id, 
                                                                              non_support_foot.child_frame_id,
                                                                              support_foot.header.stamp,
                                                                              ros::Duration(0.1));

            geometry_msgs::TransformStamped support_to_base_link = tfBuffer.lookupTransform(support_foot.header.frame_id,  
                                                                                            support_foot.child_frame_id, 
                                                                                            support_foot.header.stamp);

            geometry_msgs::PoseStamped approach_frame;
            // x at front foot toes
            approach_frame.pose.position.x = front_foot.transform.translation.x;
            // y between foots
            approach_frame.pose.position.y = non_support_foot_in_support_foot_frame.transform.translation.y/2;
            // z at ground leven (support foot height)
            approach_frame.pose.position.z = support_foot.pose.z;

            // yaw of front foot
            double yaw;
            yaw = tf2::getYaw(front_foot.transform.rotation);
            
            // pitch and roll from support foot, yaw from base link
            tf2::Quaternion rotation;
            rotation.setRPY(0.0,0.0,yaw);
            approach_frame.pose.orientation = tf2::toMsg(rotation);

            // set the broadcasted transform to the position and orientation of the base footprint
            tf.header.stamp = ros::Time::now();
            tf.header.frame_id = "base_link";
            tf.child_frame_id = "approach_frame";
            tf.transform.translation.x = approach_frame.pose.position.x;
            tf.transform.translation.y = approach_frame.pose.position.y;
            tf.transform.translation.z = approach_frame.pose.position.z;
            tf.transform.rotation = approach_frame.pose.orientation;
            br.sendTransform(tf);

        } catch(...){
            //ROS_WARN_THROTTLE(2, "Can not publish base_footprint, check your tf tree");
            continue;
        }

        r.sleep();
    }
}

void ConvenienceFramesBroadcaster::supportFootCallback(const std_msgs::Char msg)
{
    got_support_foot = true;
    is_left_support = (msg.data == 'l');
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convenience_frames");

    ConvenienceFramesBroadcaster b;
}

