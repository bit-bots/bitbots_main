#include <algorithm>
#include <bitbots_vision/rfdetr_processing.hpp>
#include <cmath>
#include <cstring>
#include <opencv2/imgproc.hpp>

namespace bitbots_vision::processing {

// ---------------------------------------------------------------------------
// preprocess_image_rfdetr
// ---------------------------------------------------------------------------

std::vector<float> preprocess_image_rfdetr(const cv::Mat& bgr, int net_h, int net_w) {
  // ImageNet normalization constants, matching demo.py.
  static constexpr float kMean[3] = {0.485f, 0.456f, 0.406f};
  static constexpr float kStd[3] = {0.229f, 0.224f, 0.225f};

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(net_w, net_h));

  cv::Mat rgb_float;
  cv::cvtColor(resized, rgb_float, cv::COLOR_BGR2RGB);
  rgb_float.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);

  std::vector<cv::Mat> channels(3);
  cv::split(rgb_float, channels);

  std::vector<float> tensor(3 * net_h * net_w);
  for (int c = 0; c < 3; ++c) {
    channels[c] = (channels[c] - kMean[c]) / kStd[c];
    std::memcpy(tensor.data() + c * net_h * net_w, channels[c].data,
                static_cast<size_t>(net_h * net_w) * sizeof(float));
  }
  return tensor;
}

// ---------------------------------------------------------------------------
// postprocess_detections_rfdetr
// ---------------------------------------------------------------------------

std::unordered_map<std::string, std::vector<Candidate>> postprocess_detections_rfdetr(
    const float* dets, const float* logits, int64_t num_dets, int64_t num_classes,
    const std::vector<std::string>& class_names, const std::unordered_map<std::string, float>& class_conf_thresholds,
    float default_conf_thresh, int orig_h, int orig_w) {
  std::unordered_map<std::string, std::vector<Candidate>> result;

  for (int64_t i = 0; i < num_dets; ++i) {
    const float* logit_row = logits + i * num_classes;

    // Numerically stable softmax over this row.
    float max_logit = logit_row[0];
    for (int64_t c = 1; c < num_classes; ++c) {
      max_logit = std::max(max_logit, logit_row[c]);
    }
    float sum_exp = 0.f;
    int best_class = 0;
    float best_exp = -1.f;
    std::vector<float> exps(static_cast<size_t>(num_classes));
    for (int64_t c = 0; c < num_classes; ++c) {
      const float e = std::exp(logit_row[c] - max_logit);
      exps[static_cast<size_t>(c)] = e;
      sum_exp += e;
      if (e > best_exp) {
        best_exp = e;
        best_class = static_cast<int>(c);
      }
    }
    const float confidence = best_exp / sum_exp;

    if (best_class < 0 || best_class >= static_cast<int>(class_names.size())) {
      continue;
    }
    const std::string& class_name = class_names[static_cast<size_t>(best_class)];
    const auto thresh_it = class_conf_thresholds.find(class_name);
    const float conf_thresh = (thresh_it != class_conf_thresholds.end()) ? thresh_it->second : default_conf_thresh;
    if (confidence < conf_thresh) {
      continue;
    }

    const float* box = dets + i * 4;
    const float cx = box[0];
    const float cy = box[1];
    const float w = box[2];
    const float h = box[3];

    const float x1 = (cx - w / 2.f) * static_cast<float>(orig_w);
    const float y1 = (cy - h / 2.f) * static_cast<float>(orig_h);
    const float x2 = (cx + w / 2.f) * static_cast<float>(orig_w);
    const float y2 = (cy + h / 2.f) * static_cast<float>(orig_h);

    result[class_name].push_back(Candidate::from_x1y1x2y2(static_cast<int>(x1), static_cast<int>(y1),
                                                          static_cast<int>(x2), static_cast<int>(y2), confidence));
  }

  return result;
}

}  // namespace bitbots_vision::processing
