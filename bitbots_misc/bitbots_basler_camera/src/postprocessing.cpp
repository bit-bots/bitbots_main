#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using std::placeholders::_1, std::placeholders::_2;

namespace postprocessing {

class PostProcessor {
  std::shared_ptr<rclcpp::Node> node_;

  image_transport::TransportHints transport_hints_;
  std::unique_ptr<image_transport::CameraSubscriber> image_sub_;
  std::unique_ptr<image_transport::CameraPublisher> image_pub_;

  int binning_factor_x_ = 1;
  int binning_factor_y_ = 1;

 public:
  PostProcessor()
      : node_(std::make_shared<rclcpp::Node>("post_processor")),
        transport_hints_(node_.get()),
        image_sub_(std::make_unique<image_transport::CameraSubscriber>(image_transport::create_camera_subscription(
            node_.get(), "in/image_raw", std::bind(&PostProcessor::image_callback, this, _1, _2),
            transport_hints_.getTransport(), rclcpp::QoS(1).get_rmw_qos_profile()))),
          image_pub_(std::make_unique<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(node_.get(), "out/image_proc"))) {
    // Declare parameters
    node_->declare_parameter("binning_factor_x", 4);
    node_->declare_parameter("binning_factor_y", 4);

    // Get parameters
    node_->get_parameter("binning_factor_x", binning_factor_x_);
    node_->get_parameter("binning_factor_y", binning_factor_y_);
  }

  /**
   * @brief Callback used to update the head mode
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    // Convert to cv::Mat
    cv::Mat image = cv_bridge::toCvShare(image_msg)->image;

    // Do some processing
    int bit_depth = sensor_msgs::image_encodings::bitDepth(image_msg->encoding);

    int type = bit_depth == 8 ? CV_8U : CV_16U;

    // Create cv::Mat for color image
    cv::Mat color(image_msg->height, image_msg->width, CV_MAKETYPE(type, 3));

    // Create cv::Mat for
    const cv::Mat bayer(image_msg->height, image_msg->width, CV_MAKETYPE(type, 1),
                        const_cast<uint8_t*>(&image_msg->data[0]), image_msg->step);

    // Get the correct conversion code
    int code = -1;
    if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8 ||
        image_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB16) {
      code = cv::COLOR_BayerBG2BGR;
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||  // NOLINT
               image_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR16) {
      code = cv::COLOR_BayerRG2BGR;
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG8 ||  // NOLINT
               image_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG16) {
      code = cv::COLOR_BayerGR2BGR;
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG8 ||  // NOLINT
               image_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG16) {
      code = cv::COLOR_BayerGB2BGR;
    };

    // Debayer the image
    cv::cvtColor(bayer, color, code);

    // Perform binning by a given factor
    cv::Mat binned(image_msg->height / binning_factor_y_, image_msg->width / binning_factor_x_, CV_MAKETYPE(type, 3));
    cv::resize(color, binned, cv::Size(), 1.0 / binning_factor_x_, 1.0 / binning_factor_y_, cv::INTER_AREA);

    // Add the binning to the camera info
    auto new_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
    new_camera_info->binning_x = binning_factor_x_;
    new_camera_info->binning_y = binning_factor_y_;

    // Convert back to sensor_msgs::Image
    cv_bridge::CvImage color_msg;
    color_msg.header = image_msg->header;
    color_msg.encoding = sensor_msgs::image_encodings::BGR8;
    color_msg.image = binned;

    // Publish the image
    image_pub_->publish(color_msg.toImageMsg(), new_camera_info);
  }

  /**
   * @brief A getter that returns the node
   */
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }
};
}  // namespace postprocessing

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  auto post_processor = std::make_shared<postprocessing::PostProcessor>();
  exec.add_node(post_processor->get_node());
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
