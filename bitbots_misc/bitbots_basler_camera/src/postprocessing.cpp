#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <pylon/_InstantCameraParams.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
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

#include "pylon_camera_parameters.hpp"

using std::placeholders::_1, std::placeholders::_2;
using namespace Pylon;

namespace postprocessing {

class PostProcessor {
  std::shared_ptr<rclcpp::Node> node_;

  image_transport::TransportHints transport_hints_;
  std::unique_ptr<image_transport::CameraPublisher> image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> camera_;

  Pylon::PylonAutoInitTerm autoInitTerm;

  std::unique_ptr<pylon_camera_parameters::ParamListener> param_listener_;
  pylon_camera_parameters::Params config_;

 public:
  PostProcessor()
      : node_(std::make_shared<rclcpp::Node>("post_processor")),
        transport_hints_(node_.get()),
        image_pub_(std::make_unique<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(node_.get(), "camera/image_proc"))) {
    // Load parameters
    param_listener_ = std::make_unique<pylon_camera_parameters::ParamListener>(node_);
    config_ = param_listener_->get_params();

    // Set up camera info manager
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(node_.get(), config_.device_user_id,
                                                                                    config_.camera_info_url);

    try {
      // List all available devices
      Pylon::DeviceInfoList_t devices;
      CTlFactory::GetInstance().EnumerateDevices(devices);

      // Print all devices and their names
      for (size_t i = 0; i < devices.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Device %ld: %s (%s)", i, devices[i].GetFriendlyName().c_str(),
                    devices[i].GetModelName().c_str());
      }

      // Initialize the camera
      initilize_camera();

    } catch (GenICam::GenericException& e) {
      // Error handling.
      RCLCPP_ERROR(node_->get_logger(), "An exception occurred: %s", e.GetDescription());
      RCLCPP_ERROR(node_->get_logger(), "Could not initialize camera");
      exit(1);
    }

    // Setup timer for publishing
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / config_.fps),
                                      std::bind(&PostProcessor::timer_callback, this));
  }

  void initilize_camera() {
    // Set up the camera
    camera_ = std::make_unique<Pylon::CBaslerUniversalInstantCamera>(CTlFactory::GetInstance().CreateFirstDevice());

    // Wait for the camera to be ready
    camera_->Open();

    // Print the model name of the camera.
    RCLCPP_INFO(node_->get_logger(), "Using device '%s'", camera_->GetDeviceInfo().GetFriendlyName().c_str());

    // Set static camera parameters
    camera_->ShutterMode.SetValue(Basler_UniversalCameraParams::ShutterModeEnums::ShutterMode_Global);
    camera_->Width.SetToMaximum();
    camera_->Height.SetToMaximum();
    camera_->PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
    camera_->BalanceWhiteAuto.SetValue(
        Basler_UniversalCameraParams::BalanceWhiteAutoEnums::BalanceWhiteAuto_Continuous);

    // Set MTU and inter-package delay
    camera_->GevSCPSPacketSize.SetValue(config_.gige.mtu_size);
    camera_->GevSCPD.SetValue(config_.gige.inter_pkg_delay);

    // Set to manual acquisition mode
    camera_->AcquisitionMode.SetValue(Basler_UniversalCameraParams::AcquisitionModeEnums::AcquisitionMode_SingleFrame);

    // Set the camera parameters
    apply_camera_parameters();
  }

  void apply_camera_parameters() {
    // Set the camera parameters
    camera_->Gain.SetValue(config_.gain);
    camera_->ExposureTime.SetValue(config_.exposure);
    camera_->Gamma.SetValue(config_.gamma);
  }

  void timer_callback() {
    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;

    // Field to store the trigger time
    rclcpp::Time trigger_time;

    try {
      // Check if the config has changed
      if (param_listener_->is_old(config_)) {
        // Update the camera parameters
        config_ = param_listener_->get_params();
        // Apply the new camera parameters
        apply_camera_parameters();
      }

      // Try to reinitialize the camera if the connection is lost
      if (!camera_->IsOpen() or camera_->IsCameraDeviceRemoved()) {
        RCLCPP_WARN(node_->get_logger(), "Camera connection lost. Reinitializing camera");
        initilize_camera();
      }

      // Cancel all pending grabs
      camera_->StopGrabbing();

      // Start frame acquisition
      camera_->StartGrabbing(1, GrabStrategy_LatestImageOnly);

      // Store the current time
      trigger_time = node_->now();

      // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
      camera_->RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);

      // Image grabbed successfully?
      if (!ptrGrabResult->GrabSucceeded()) {
        RCLCPP_ERROR(node_->get_logger(), "Error: %x %s", ptrGrabResult->GetErrorCode(),
                     ptrGrabResult->GetErrorDescription().c_str());
        return;
      }

    } catch (GenICam::GenericException& e) {
      // Error handling.
      RCLCPP_ERROR(node_->get_logger(), "An exception occurred: %s", e.GetDescription());
      return;
    }

    // Convert to cv::Mat
    cv::Mat image(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*)ptrGrabResult->GetBuffer());

    // Create cv::Mat for color image
    cv::Mat color(image.size(), CV_MAKETYPE(CV_8U, 3));

    // Debayer the image
    cv::cvtColor(image, color, cv::COLOR_BayerBG2BGR);

    // Perform binning by a given factor
    cv::Mat binned(image.size().height / config_.binning_factor_y, image.size().width / config_.binning_factor_x,
                   CV_MAKETYPE(CV_8U, 3));
    cv::resize(color, binned, cv::Size(), 1.0 / config_.binning_factor_x, 1.0 / config_.binning_factor_y,
               cv::INTER_AREA);

    // Add the binning to the camera info
    auto camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());
    camera_info->binning_x = config_.binning_factor_x;
    camera_info->binning_y = config_.binning_factor_y;
    camera_info->header.frame_id = config_.camera_frame_id;
    camera_info->header.stamp = trigger_time;

    // Convert back to sensor_msgs::Image
    cv_bridge::CvImage color_msg;
    color_msg.header = camera_info->header;
    color_msg.encoding = sensor_msgs::image_encodings::BGR8;
    color_msg.image = binned;

    // Publish the image
    image_pub_->publish(color_msg.toImageMsg(), camera_info);
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
