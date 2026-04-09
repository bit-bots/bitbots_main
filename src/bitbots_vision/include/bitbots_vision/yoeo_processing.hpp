#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "bitbots_vision/candidate.hpp"

/// Pure preprocessing / postprocessing functions for YOEO inference.
///
/// Extracted as standalone free functions so they can be unit-tested
/// without an ONNX session.
namespace bitbots_vision::processing {

// ---------------------------------------------------------------------------
// Preprocessing
// ---------------------------------------------------------------------------

/// Metadata produced by preprocess_image() that is needed to invert the
/// padding / scaling transforms in the postprocessors.
struct PreprocessInfo {
  int orig_h{0};
  int orig_w{0};
  int max_dim{0};      ///< max(orig_h, orig_w) == side length of the padded square
  int pad_top{0};
  int pad_bottom{0};
  int pad_left{0};
  int pad_right{0};
  int net_h{0};
  int net_w{0};
};

/// Convert a BGR uint8 image to a normalised float32 CHW tensor suitable for
/// YOEO inference.
///
/// Steps:
///   1. BGR → RGB, normalise to [0, 1]
///   2. Pad shorter axis with zeros to make the image square (max_dim × max_dim)
///   3. Resize to (net_h, net_w)
///   4. Rearrange axes HWC → CHW
///
/// @param bgr      Input image (CV_8UC3, BGR colour order)
/// @param net_h    Network input height
/// @param net_w    Network input width
/// @param info     [out] Padding / sizing metadata for use in postprocessors
/// @return         Flattened CHW float32 tensor of size 3 * net_h * net_w
std::vector<float> preprocess_image(
  const cv::Mat & bgr, int net_h, int net_w, PreprocessInfo & info);

// ---------------------------------------------------------------------------
// Non-maximum suppression
// ---------------------------------------------------------------------------

/// NMS with cross-class suppression for robot classes.
///
/// All robot class IDs share the same class-offset so they are suppressed
/// against each other (treating them as a single super-class). Non-robot
/// classes are suppressed independently.
///
/// @param boxes            Boxes in xywh format (center x, center y, w, h)
/// @param scores           Confidence score per box
/// @param class_ids        Class id per box
/// @param robot_class_ids  IDs of all "robot" classes that share suppression
/// @param nms_threshold    IoU threshold for suppression
/// @param max_detections   Upper limit on kept boxes
/// @return                 Indices of kept boxes
std::vector<int> nms_boxes(
  const std::vector<cv::Rect2d> & boxes,
  const std::vector<float> & scores,
  const std::vector<int> & class_ids,
  const std::vector<int> & robot_class_ids,
  float nms_threshold,
  int max_detections = 30);

// ---------------------------------------------------------------------------
// Detection postprocessing
// ---------------------------------------------------------------------------

/// Postprocess raw YOEO detection output.
///
/// Raw output layout: [1, N, 5 + num_classes]
///   per box: [x_c, y_c, w, h, obj_conf, class_prob_0, …]
///   all coordinates are in network-input-image space
///
/// @param data             Pointer to the first element of the output tensor
/// @param num_boxes        N (number of raw anchor boxes)
/// @param stride           5 + num_classes
/// @param class_names      Detection class names (index == class id)
/// @param robot_class_ids  IDs of robot classes (for cross-class NMS)
/// @param conf_thresh      Objectness × class-prob threshold
/// @param nms_thresh       NMS IoU threshold
/// @param info             Preprocessing metadata for coordinate rescaling
/// @return                 Map of class_name → kept Candidates
std::unordered_map<std::string, std::vector<Candidate>> postprocess_detections(
  const float * data,
  int64_t num_boxes,
  int64_t stride,
  const std::vector<std::string> & class_names,
  const std::vector<int> & robot_class_ids,
  float conf_thresh,
  float nms_thresh,
  const PreprocessInfo & info);

// ---------------------------------------------------------------------------
// Segmentation postprocessing
// ---------------------------------------------------------------------------

/// Postprocess raw YOEO segmentation output.
///
/// Raw output layout: [1, H_seg, W_seg]  (argmax class-index map, float)
///
/// Steps:
///   1. Resize to the padded original image size (max_dim × max_dim)
///   2. Unpad to original image size
///   3. For each class index i build a binary CV_8UC1 mask (255 = active)
///
/// @param data         Pointer to the first element of the output tensor
/// @param seg_h        H_seg
/// @param seg_w        W_seg
/// @param class_names  Segmentation class names (index == class id)
/// @param info         Preprocessing metadata for unpadding
/// @return             Map of class_name → CV_8UC1 binary mask
std::unordered_map<std::string, cv::Mat> postprocess_segmentation(
  const float * data,
  int seg_h,
  int seg_w,
  const std::vector<std::string> & class_names,
  const PreprocessInfo & info);

}  // namespace bitbots_vision::processing
