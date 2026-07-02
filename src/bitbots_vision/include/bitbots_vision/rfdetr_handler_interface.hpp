#pragma once

#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/rfdetr_config.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace bitbots_vision {

/// Common interface implemented by every RF-DETR inference backend (ONNX
/// Runtime, TensorRT, ...) so `VisionNode` can use whichever one was
/// compiled in without caring which.
class RfdetrHandlerInterface {
 public:
  virtual ~RfdetrHandlerInterface() = default;

  /// Update thresholds without reloading the model.
  virtual void reconfigure(const RfdetrConfig& cfg) = 0;

  /// Set the image to be processed by the next call to predict().
  /// The image must be in BGR8 format (as delivered by cv_bridge).
  virtual void set_image(const cv::Mat& bgr_image) = 0;

  /// Run the network on the current image (no-op if already up to date).
  virtual void predict() = 0;

  virtual std::vector<Candidate> get_detection_candidates_for(const std::string& class_name) = 0;

  /// Line-segmentation mask (CV_8UC1, 0/255) at original image resolution.
  /// Empty if the loaded model has no line-segmentation output.
  virtual cv::Mat get_line_mask() = 0;

  virtual const std::vector<std::string>& detection_class_names() const = 0;
};

}  // namespace bitbots_vision
