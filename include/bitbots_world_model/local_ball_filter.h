
#ifndef LOCAL_BALL_TRACKER
#define LOCAL_BALL_TRACKER

#include <vector>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <humanoid_league_msgs/BallRelative.h>
#include <dynamic_reconfigure/server.h>

#include <bitbots_world_model/LocalBallFilterConfig.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>



namespace hlm = humanoid_league_msgs;
namespace wm = bitbots_world_model;

class LocalTracker {
public:
    LocalTracker();

    // Needed services:
    // - reset all filters
    // - reset a specific filter?
    // - start/stop filtering
    // - start/stop sending?

    bool reset_filters_callback(
            std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void ball_callback(const hlm::BallRelative& msg);

    void init();

    void
    dynamic_reconfigure_callback(wm::LocalBallFilterConfig& config, uint32_t level);
    void filter_timer_callback(const ros::TimerEvent&);

private:
    ros::NodeHandle nh_;

    ros::Subscriber ball_subscriber_;

    ros::Publisher ball_publisher_;

    ros::ServiceServer reset_filter_service_;

    ros::Timer filter_timer_;

    tf2_ros::TransformBroadcaster transform_broadcaster_;
    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;

    hlm::BallRelative ball_measurement_;

    wm::LocalBallFilterConfig config_;

    bool valid_configuration_;


    void send_ball_transform();

    void publish_result();

    void exec_filter_step();

    bool reset_filter_callback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

    /**
     * reinitializes  filter
     */
    void reset_filter();
};

#endif
