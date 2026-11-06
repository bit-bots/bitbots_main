#pragma once

#include <bitbots_vision/candidate.hpp>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

/// Pure preprocessing / postprocessing functions for RF-DETR inference.
///
/// Extracted as standalone free functions so they can be unit-tested
/// without an ONNX session. Mirrors the reference Python implementation
/// (rf-detr-webgpu/demo.py) exactly.
namespace bitbots_vision::processing {

// ---------------------------------------------------------------------------
// Preprocessing
// ---------------------------------------------------------------------------

/// Convert a BGR uint8 image to a normalised float32 CHW tensor suitable for
/// RF-DETR inference.
///
/// Steps (matching demo.py's preprocess()):
///   1. Direct resize to (net_h, net_w) — NOT aspect-ratio preserving, no
///      padding. Normalized box coordinates from the model survive this
///      independent-per-axis resize, so postprocessing can decode boxes
///      directly against the original image dimensions.
///   2. BGR -> RGB, normalise to [0, 1]
///   3. ImageNet standardisation: (x - mean) / std per RGB channel
///   4. Rearrange axes HWC -> CHW
///
/// @param bgr    Input image (CV_8UC3, BGR colour order)
/// @param net_h  Network input height
/// @param net_w  Network input width
/// @return       Flattened CHW float32 tensor of size 3 * net_h * net_w
std::vector<float> preprocess_image_rfdetr(const cv::Mat& bgr, int net_h, int net_w);

// ---------------------------------------------------------------------------
// Detection postprocessing
// ---------------------------------------------------------------------------

/// Postprocess raw RF-DETR detection output. RF-DETR is NMS-free (a fixed
/// number of object queries, used as-is), so unlike YOEO there is no NMS
/// step here.
///
/// The confidence threshold is per-class: different classes (e.g. ball vs.
/// robot) benefit from different cutoffs, since RF-DETR's raw NMS-free
/// output otherwise includes many overlapping low-confidence duplicate boxes
/// per real object. Classes not present in `class_conf_thresholds` fall back
/// to `default_conf_thresh`.
///
/// @param dets                  Pointer to [num_dets, 4] normalized (cx, cy, w, h)
/// @param logits                Pointer to [num_dets, num_classes] raw logits
/// @param num_dets              Number of object queries (e.g. 300)
/// @param num_classes           Number of classes (index == class id)
/// @param class_names           Detection class names (index == class id)
/// @param class_conf_thresholds Per-class-name softmax-confidence thresholds
/// @param default_conf_thresh   Threshold used for any class not in `class_conf_thresholds`
/// @param orig_h                Original image height (pixels)
/// @param orig_w                Original image width (pixels)
/// @return                       Map of class_name -> kept Candidates
std::unordered_map<std::string, std::vector<Candidate>> postprocess_detections_rfdetr(
    const float* dets, const float* logits, int64_t num_dets, int64_t num_classes,
    const std::vector<std::string>& class_names, const std::unordered_map<std::string, float>& class_conf_thresholds,
    float default_conf_thresh, int orig_h, int orig_w);

// ---------------------------------------------------------------------------
// Line-mask postprocessing
// ---------------------------------------------------------------------------

/// Decode RF-DETR's learned line-segmentation head output: sigmoid over the
/// raw logits, resize to the original image resolution, then threshold.
///
/// @param line_mask_logits  Pointer to [mask_h, mask_w] raw logits (single batch/channel)
/// @param mask_h            Segmentation output height (e.g. 96)
/// @param mask_w            Segmentation output width (e.g. 96)
/// @param threshold         Post-sigmoid probability threshold to keep a pixel
/// @param orig_h            Original image height (pixels)
/// @param orig_w            Original image width (pixels)
/// @return                  CV_8UC1 binary mask (0 / 255) at original image resolution
cv::Mat postprocess_line_mask_rfdetr(const float* line_mask_logits, int mask_h, int mask_w, float threshold, int orig_h,
                                     int orig_w);

// ---------------------------------------------------------------------------
// Candidate suppression
// ---------------------------------------------------------------------------

/// Paint out (set to 0) the regions covered by the given candidates, each
/// inflated by `margin_px` on all sides and clamped to the mask bounds.
/// Modifies `mask` in place.
void suppress_candidates(cv::Mat& mask, const std::vector<Candidate>& candidates, int margin_px);

}  // namespace bitbots_vision::processing
