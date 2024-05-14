#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <pylon/_InstantCameraParams.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
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

namespace basler_camera {

class BaslerCamera {
  std::shared_ptr<rclcpp::Node> node_;

  image_transport::TransportHints transport_hints_;
  image_transport::CameraPublisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> camera_;

  std::optional<double> camera_tick_frequency_;

  Pylon::PylonAutoInitTerm autoInitTerm;

  pylon_camera_parameters::ParamListener param_listener_;
  pylon_camera_parameters::Params config_;

 public:
  BaslerCamera()
      : node_(std::make_shared<rclcpp::Node>("post_processor")),
        transport_hints_(node_.get()),
        image_pub_(image_transport::create_camera_publisher(node_.get(), "camera/image_proc")),
        diagnostics_pub_(node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1)),
        param_listener_(node_) {
    // Load parameters
    config_ = param_listener_.get_params();

    // Set up camera info manager
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(node_.get(), config_.device_user_id,
                                                                                    config_.camera_info_url);

    try {
      // Initialize the camera
      initilize_camera();

    } catch (GenICam::GenericException& e) {
      // Error handling.
      RCLCPP_ERROR(node_->get_logger(), "An exception occurred: %s", e.GetDescription());
      RCLCPP_ERROR(node_->get_logger(), "Could not initialize camera");
      publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not initialize camera");
      exit(1);
    }
    // Setup timer for publishing
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / config_.fps),
                                      std::bind(&BaslerCamera::timer_callback, this));
  }

  void initilize_camera() {
    // List all available devices
    Pylon::DeviceInfoList_t devices;
    CTlFactory::GetInstance().EnumerateDevices(devices);

    // The device user id of the camera we want to use
    std::optional<Pylon::CDeviceInfo> our_device_info;

    // Print all devices and their names
    for (auto device : devices) {
      RCLCPP_INFO(node_->get_logger(), "Device: %s (%s)", device.GetFriendlyName().c_str(),
                  device.GetModelName().c_str());
      // Check if the device has a user defined name and if it is the one we want to use
      if (device.GetUserDefinedName().compare(config_.device_user_id.c_str()) == 0) {
        our_device_info = device;
      }
    }

    // Wait until the camera is found
    while (rclcpp::ok() && !our_device_info) {
      RCLCPP_ERROR(node_->get_logger(), "Could not find device with user id '%s'", config_.device_user_id.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Retrying in %f seconds", config_.reconnect_interval);
      publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::STALE,
                                "Could not find the device with id " + config_.device_user_id);
      // Wait
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(config_.reconnect_interval)));
      // List all available devices
      CTlFactory::GetInstance().EnumerateDevices(devices);
      // Get the matching device
      auto it = std::find_if(devices.begin(), devices.end(), [this](const auto& device) {
        return device.GetUserDefinedName().compare(config_.device_user_id.c_str()) == 0;
      });
      // Use the device if it was found
      if (it != devices.end()) {
        our_device_info = *it;
      }
    }

    // Check if the node is still ok, maybe it was shut down while waiting
    if (!rclcpp::ok()) {
      return;
    }

    // Set up the camera
    camera_ = std::make_unique<Pylon::CBaslerUniversalInstantCamera>(
        CTlFactory::GetInstance().CreateDevice(our_device_info.value()));
    RCLCPP_INFO(node_->get_logger(), "Using device user id '%s'", config_.device_user_id.c_str());

    // Wait for the camera to be ready
    camera_->Open();

    // Set MTU and inter-package delay
    camera_->GevSCPSPacketSize.SetValue(config_.gige.mtu_size);
    camera_->GevSCPD.SetValue(config_.gige.inter_pkg_delay);

    camera_->ClearBufferModeEnable();

    // Set the camera parameters
    apply_camera_parameters();

    // Set static camera parameters
    camera_->ShutterMode.SetValue(Basler_UniversalCameraParams::ShutterModeEnums::ShutterMode_Global);
    camera_->Width.SetToMaximum();
    camera_->Height.SetToMaximum();
    camera_->PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
    camera_->BalanceWhiteAuto.SetValue(
        Basler_UniversalCameraParams::BalanceWhiteAutoEnums::BalanceWhiteAuto_Continuous);

    // Set to manual acquisition mode
    camera_->AcquisitionMode.SetValue(Basler_UniversalCameraParams::AcquisitionModeEnums::AcquisitionMode_Continuous);

    // Get the camera frequency
    camera_tick_frequency_ = camera_->GevTimestampTickFrequency();

    // Start grabbing
    camera_->StartGrabbing(GrabStrategy_LatestImageOnly);
  }

  void apply_camera_parameters() {
    // Set the camera parameters
    camera_->GainAuto.SetValue(Basler_UniversalCameraParams::GainAutoEnums::GainAuto_Off);
    camera_->ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAutoEnums::ExposureAuto_Off);
    camera_->GainRaw.SetValue(config_.gain);
    camera_->ExposureTimeAbs.SetValue(config_.exposure);
  }

  void publish_basic_diagnostics(unsigned char severity, const std::string& message) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "CAMERA";
    status.hardware_id = config_.device_user_id;
    status.level = severity;
    status.message = message;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = node_->now();
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }

  void timer_callback() {
    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;

    // Field to store the trigger time
    rclcpp::Time trigger_time;

    try {
      // Check if the config has changed
      if (param_listener_.is_old(config_)) {
        // Update the camera parameters
        config_ = param_listener_.get_params();
        // Apply the new camera parameters
        apply_camera_parameters();
      }

      // Try to reinitialize the camera if the connection is lost
      if (!camera_->IsOpen() or camera_->IsCameraDeviceRemoved()) {
        auto message = "Camera connection lost. Reinitializing camera";
        RCLCPP_WARN(node_->get_logger(), message);
        publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::STALE, message);
        initilize_camera();
      }

      // Get current camera time
      camera_->GevTimestampControlLatch();
      auto camera_time_stamp_at_capture = (__int128_t)camera_->GevTimestampValue();
      camera_->GevTimestampControlLatchReset();
      auto ros_time_stamp_at_capture = node_->now();

      // Start frame acquisition
      camera_->ExecuteSoftwareTrigger();

      // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
      camera_->RetrieveResult(100, ptrGrabResult, TimeoutHandling_ThrowException);

      // Image grabbed successfully?
      if (!ptrGrabResult->GrabSucceeded()) {
        RCLCPP_ERROR(node_->get_logger(), "Error: %x %s", ptrGrabResult->GetErrorCode(),
                     ptrGrabResult->GetErrorDescription().c_str());
        publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error while grabbing image");
        return;
      }

      // Adjust the time stamp of the image
      auto image_camera_stamp = (__int128_t)ptrGrabResult->GetTimeStamp();
      auto image_age_in_seconds = (image_camera_stamp - camera_time_stamp_at_capture) / camera_tick_frequency_.value();
      trigger_time = ros_time_stamp_at_capture + rclcpp::Duration::from_seconds(image_age_in_seconds);
    } catch (GenICam::GenericException& e) {
      // Error handling.
      RCLCPP_ERROR(node_->get_logger(), "An exception occurred: %s", e.GetDescription());
      publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                                "An exception occurred during image acquisition");
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
    image_pub_.publish(color_msg.toImageMsg(), camera_info);

    // Check if image is too dark
    float luminance = 0;
    int sample_grid = 5;
    int step_height = binned.size().height / sample_grid;
    int step_width = binned.size().width / sample_grid;
    for (int i = 0; i < sample_grid; i++) {
      for (int j = 0; j < sample_grid; j++) {
        cv::Vec3b pixel = binned.at<cv::Vec3b>(i * step_height, j * step_width);
        luminance += (pixel[0] + pixel[1] + pixel[2]) / (3 * 255.0 * sample_grid * sample_grid);
      }
    }

    // Warn if the image is too dark
    if (luminance < config_.misc.darkness_threshold) {
      auto message = "Image is too dark. Did you forget the camera cover?";
      RCLCPP_WARN_ONCE(node_->get_logger(), message);
      publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    } else {
      // Everything is fine
      publish_basic_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera is running");
    }
  }

  /**
   * @brief A getter that returns the node
   */
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }
};
}  // namespace basler_camera

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec = rclcpp::experimental::executors::EventsExecutor(
      std::make_unique<rclcpp::experimental::executors::SimpleEventsQueue>(), true, rclcpp::ExecutorOptions());

  auto post_processor = std::make_shared<basler_camera::BaslerCamera>();
  exec.add_node(post_processor->get_node());
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
