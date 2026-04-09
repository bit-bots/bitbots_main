#include "bitbots_vision/yoeo_processing.hpp"

#include <algorithm>
#include <cstring>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace bitbots_vision::processing {

// ---------------------------------------------------------------------------
// preprocess_image
// ---------------------------------------------------------------------------

std::vector<float> preprocess_image(
  const cv::Mat & bgr, int net_h, int net_w, PreprocessInfo & info)
{
  info.orig_h = bgr.rows;
  info.orig_w = bgr.cols;
  info.max_dim = std::max(info.orig_h, info.orig_w);
  info.net_h = net_h;
  info.net_w = net_w;

  info.pad_top = (info.orig_w > info.orig_h) ? (info.orig_w - info.orig_h) / 2 : 0;
  info.pad_bottom =
    (info.orig_w > info.orig_h) ? (info.orig_w - info.orig_h) - info.pad_top : 0;
  info.pad_left = (info.orig_h > info.orig_w) ? (info.orig_h - info.orig_w) / 2 : 0;
  info.pad_right =
    (info.orig_h > info.orig_w) ? (info.orig_h - info.orig_w) - info.pad_left : 0;

  cv::Mat rgb_float;
  cv::cvtColor(bgr, rgb_float, cv::COLOR_BGR2RGB);
  rgb_float.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);

  cv::Mat padded;
  cv::copyMakeBorder(
    rgb_float, padded,
    info.pad_top, info.pad_bottom, info.pad_left, info.pad_right,
    cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  cv::Mat resized;
  cv::resize(padded, resized, cv::Size(net_w, net_h));

  std::vector<float> tensor(3 * net_h * net_w);
  std::vector<cv::Mat> channels(3);
  cv::split(resized, channels);
  for (int c = 0; c < 3; ++c) {
    std::memcpy(
      tensor.data() + c * net_h * net_w, channels[c].data,
      static_cast<size_t>(net_h * net_w) * sizeof(float));
  }
  return tensor;
}

// ---------------------------------------------------------------------------
// nms_boxes
// ---------------------------------------------------------------------------

std::vector<int> nms_boxes(
  const std::vector<cv::Rect2d> & boxes,
  const std::vector<float> & scores,
  const std::vector<int> & class_ids,
  const std::vector<int> & robot_class_ids,
  float nms_threshold,
  int max_detections)
{
  constexpr int kMaxWH = 4096;

  std::vector<cv::Rect2d> shifted(boxes);
  for (size_t i = 0; i < boxes.size(); ++i) {
    int eff = class_ids[i];
    if (!robot_class_ids.empty()) {
      bool is_robot =
        std::find(robot_class_ids.begin(), robot_class_ids.end(), eff) !=
        robot_class_ids.end();
      if (is_robot) {
        eff = robot_class_ids[0];
      }
    }
    const double offset = static_cast<double>(eff) * kMaxWH;
    shifted[i].x += offset;
    shifted[i].y += offset;
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(shifted, scores, 0.0f, nms_threshold, indices, 1.0f, max_detections);
  return indices;
}

// ---------------------------------------------------------------------------
// postprocess_detections
// ---------------------------------------------------------------------------

std::unordered_map<std::string, std::vector<Candidate>> postprocess_detections(
  const float * data,
  int64_t num_boxes,
  int64_t stride,
  const std::vector<std::string> & class_names,
  const std::vector<int> & robot_class_ids,
  float conf_thresh,
  float nms_thresh,
  const PreprocessInfo & info)
{
  const int num_classes = static_cast<int>(class_names.size());
  const float scale =
    static_cast<float>(info.max_dim) / static_cast<float>(info.net_h);

  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int64_t i = 0; i < num_boxes; ++i) {
    const float * row = data + i * stride;
    const float obj_conf = row[4];
    if (obj_conf < conf_thresh) {
      continue;
    }

    int best_class = 0;
    float best_score = 0.f;
    for (int c = 0; c < num_classes; ++c) {
      const float s = obj_conf * row[5 + c];
      if (s > best_score) {
        best_score = s;
        best_class = c;
      }
    }

    if (best_score < conf_thresh) {
      continue;
    }

    boxes.push_back({row[0], row[1], row[2], row[3]});
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  std::unordered_map<std::string, std::vector<Candidate>> result;
  if (boxes.empty()) {
    return result;
  }

  const auto keep = nms_boxes(boxes, scores, class_ids, robot_class_ids, nms_thresh);

  for (int idx : keep) {
    const auto & b = boxes[idx];
    const float x1 = (b.x - b.width / 2.f) * scale - static_cast<float>(info.pad_left);
    const float y1 = (b.y - b.height / 2.f) * scale - static_cast<float>(info.pad_top);
    const float x2 = (b.x + b.width / 2.f) * scale - static_cast<float>(info.pad_left);
    const float y2 = (b.y + b.height / 2.f) * scale - static_cast<float>(info.pad_top);

    result[class_names[class_ids[idx]]].push_back(
      Candidate::from_x1y1x2y2(
        static_cast<int>(x1), static_cast<int>(y1),
        static_cast<int>(x2), static_cast<int>(y2),
        scores[idx]));
  }
  return result;
}

// ---------------------------------------------------------------------------
// postprocess_segmentation
// ---------------------------------------------------------------------------

std::unordered_map<std::string, cv::Mat> postprocess_segmentation(
  const float * data,
  int seg_h,
  int seg_w,
  const std::vector<std::string> & class_names,
  const PreprocessInfo & info)
{
  cv::Mat seg_float(seg_h, seg_w, CV_32FC1, const_cast<float *>(data));

  cv::Mat seg_padded;
  cv::resize(
    seg_float, seg_padded, cv::Size(info.max_dim, info.max_dim), 0, 0, cv::INTER_NEAREST);

  cv::Mat seg_cropped = seg_padded(
    cv::Range(info.pad_top, info.max_dim - info.pad_bottom),
    cv::Range(info.pad_left, info.max_dim - info.pad_right));

  cv::Mat seg_int;
  seg_cropped.convertTo(seg_int, CV_32SC1);

  std::unordered_map<std::string, cv::Mat> result;
  for (int i = 0; i < static_cast<int>(class_names.size()); ++i) {
    result[class_names[i]] = (seg_int == i);
  }
  return result;
}

}  // namespace bitbots_vision::processing
