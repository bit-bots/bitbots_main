#include "bitbots_vision/debug_image.hpp"

#include <opencv2/imgproc.hpp>

namespace bitbots_vision {

void DebugImage::set_image(const cv::Mat& image) { image_ = image.clone(); }

void DebugImage::draw_ball_candidates(const std::vector<Candidate>& candidates, const cv::Scalar& color,
                                      int thickness) {
  if (!active_) {
    return;
  }
  for (const auto& c : candidates) {
    cv::circle(image_, {c.center_x(), c.center_y()}, c.radius(), color, thickness);
  }
}

void DebugImage::draw_box_candidates(const std::vector<Candidate>& candidates, const cv::Scalar& color, int thickness) {
  if (!active_) {
    return;
  }
  for (const auto& c : candidates) {
    cv::rectangle(image_, {c.x1, c.y1}, {c.x2(), c.y2()}, color, thickness);
  }
}

void DebugImage::draw_mask(const cv::Mat& mask, const cv::Scalar& color, double opacity) {
  if (!active_ || mask.empty()) {
    return;
  }

  cv::Mat color_layer(image_.size(), image_.type(), color);
  cv::Mat blend;
  cv::addWeighted(image_, 1.0 - opacity, color_layer, opacity, 0.0, blend);

  // Only apply blend where mask is non-zero
  cv::Mat mask_bool;
  cv::compare(mask, 0, mask_bool, cv::CMP_NE);

  blend.copyTo(image_, mask_bool);
}

}  // namespace bitbots_vision
