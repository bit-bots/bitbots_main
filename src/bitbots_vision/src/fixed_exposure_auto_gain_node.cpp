#include <bitbots_vision/auto_gain_controller.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace bitbots_vision {

class FixedExposureAutoGainNode : public rclcpp::Node {
 public:
  FixedExposureAutoGainNode() : Node("fixed_exposure_auto_gain") {
    const auto image_topic = declare_parameter<std::string>("image_topic", "zed/zed_node/rgb/image_rect_color");
    const auto camera_node = declare_parameter<std::string>("camera_node", "/zed/zed_node");
    exposure_ = static_cast<int>(declare_parameter<int>("exposure", 20));
    current_gain_ = static_cast<int>(declare_parameter<int>("initial_gain", 40));
    const double update_rate = declare_parameter<double>("update_rate", 2.0);
    if (update_rate <= 0.0) {
      throw std::invalid_argument("update rate must be positive");
    }
    update_period_ = rclcpp::Duration::from_seconds(1.0 / update_rate);

    AutoGainConfig config;
    config.target_brightness = declare_parameter<double>("target_brightness", 110.0);
    config.brightness_deadband = declare_parameter<double>("brightness_deadband", 8.0);
    config.gain_kp = declare_parameter<double>("gain_kp", 0.08);
    config.max_gain_step = static_cast<int>(declare_parameter<int>("max_gain_step", 4));
    config.min_gain = static_cast<int>(declare_parameter<int>("min_gain", 0));
    config.max_gain = static_cast<int>(declare_parameter<int>("max_gain", 100));
    controller_ = std::make_unique<AutoGainController>(config);

    if (exposure_ < 0 || exposure_ > 100 || current_gain_ < config.min_gain || current_gain_ > config.max_gain) {
      throw std::invalid_argument("exposure or initial gain is outside the configured range");
    }

    camera_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(this, camera_node);
    setup_timer_ =
        create_wall_timer(std::chrono::seconds(1), std::bind(&FixedExposureAutoGainNode::configure_camera, this));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        std::bind(&FixedExposureAutoGainNode::image_callback, this, std::placeholders::_1));
  }

 private:
  void configure_camera() {
    if (!camera_parameters_->service_is_ready() || parameter_update_pending_) {
      return;
    }

    parameter_update_pending_ = true;
    const std::vector<rclcpp::Parameter> parameters{
        rclcpp::Parameter("video.auto_exposure_gain", false),
        rclcpp::Parameter("video.exposure", exposure_),
        rclcpp::Parameter("video.gain", current_gain_),
    };
    camera_parameters_->set_parameters(parameters, [this](const auto future) {
      parameter_update_pending_ = false;
      const auto results = future.get();
      if (results.size() != 3 || !results[0].successful || !results[1].successful || !results[2].successful) {
        RCLCPP_ERROR(get_logger(), "Failed to configure fixed exposure and manual gain");
        return;
      }

      camera_configured_ = true;
      setup_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Fixed exposure at %d; software auto-gain started at %d", exposure_, current_gain_);
    });
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    const auto now = get_clock()->now();
    if (!camera_configured_ || parameter_update_pending_ || now - last_update_ < update_period_) {
      return;
    }
    last_update_ = now;

    cv::Mat gray;
    try {
      gray = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8)->image;
    } catch (const cv_bridge::Exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot measure image brightness: %s", error.what());
      return;
    }

    cv::Mat sampled;
    cv::resize(gray, sampled, cv::Size(), 0.125, 0.125, cv::INTER_AREA);
    const double brightness = cv::mean(sampled)[0];
    const auto requested_gain = controller_->update(brightness, current_gain_);
    if (!requested_gain.has_value()) {
      return;
    }

    parameter_update_pending_ = true;
    const int new_gain = requested_gain.value();
    camera_parameters_->set_parameters(
        {rclcpp::Parameter("video.gain", new_gain)}, [this, new_gain, brightness](const auto future) {
          parameter_update_pending_ = false;
          const auto results = future.get();
          if (results.size() != 1 || !results[0].successful) {
            RCLCPP_WARN(get_logger(), "Failed to set camera gain to %d", new_gain);
            return;
          }

          current_gain_ = new_gain;
          RCLCPP_DEBUG(get_logger(), "Brightness %.1f, camera gain %d", brightness, current_gain_);
        });
  }

  int exposure_;
  int current_gain_;
  rclcpp::Duration update_period_{0, 0};
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  bool camera_configured_{false};
  bool parameter_update_pending_{false};
  std::unique_ptr<AutoGainController> controller_;
  std::shared_ptr<rclcpp::AsyncParametersClient> camera_parameters_;
  rclcpp::TimerBase::SharedPtr setup_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace bitbots_vision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_vision::FixedExposureAutoGainNode>();
  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
