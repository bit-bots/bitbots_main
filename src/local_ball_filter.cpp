#include "bitbots_world_model/local_ball_filter.h"

LocalTracker::LocalTracker() :
    nh_(),
    valid_configuration_(false),
    transform_listener_(transform_buffer_) {
  ROS_INFO("Created Bit-Bots world model");
}

void LocalTracker::ball_callback(const hlm::BallRelative &msg) {
  ball_measurement_ = msg;
}

bool LocalTracker::reset_filter_callback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // pretty self-explaining... it resets all the filters
  // TODO
  res.success = true;
  res.message = "resetted all filter";
  return true;
}

void LocalTracker::init() {
  if (!valid_configuration_) {
    ROS_ERROR(
        "You tried to initialize the world model with an invalid configuration!\n The dynamic_reconfigure_callback has to be called at least once with a valid configuration before initializing the world model!");
    return;
  }
}

void LocalTracker::exec_filter_step() {
  // TODO
}

void LocalTracker::filter_timer_callback(const ros::TimerEvent &) {
  // the content of this function is what happens in a single timestep
  //TODO
}

void LocalTracker::publish_result() {
  //TODO
}
void LocalTracker::dynamic_reconfigure_callback(wm::LocalBallFilterConfig &config, uint32_t level) {

  if (config.ball_subscribe_topic!=config_.ball_subscribe_topic) {
    ball_subscriber_ = nh_.subscribe(
        config.ball_subscribe_topic.c_str(), 1, &LocalTracker::ball_callback, this);
  }
  if (config.ball_publish_topic!=config_.ball_publish_topic) {
    ball_publisher_ =
        nh_.advertise<hlm::BallRelative>(config.ball_publish_topic.c_str(), 1);
  }

    // initializing timer
    filter_timer_ = nh_.createTimer(
            ros::Duration(
                    1.0 / static_cast<double>(config.filter_frequency)),
            &LocalTracker::filter_timer_callback, this);

  config_ = config;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "world_model");
  LocalTracker local_tracker;

  // dynamic reconfigure
  dynamic_reconfigure::Server<wm::LocalBallFilterConfig>
      dynamic_reconfigure_server;
  dynamic_reconfigure::Server<wm::LocalBallFilterConfig>::CallbackType f =
      boost::bind(&LocalTracker::dynamic_reconfigure_callback, &local_tracker,
                  _1, _2);
  dynamic_reconfigure_server.setCallback(
      f);  // automatically calls the callback once



  ros::spin();
  return 0;
}
