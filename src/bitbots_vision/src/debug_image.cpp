#include <bitbots_vision/debug_image.hpp>
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

void DebugImage::draw_mask_hatched(const cv::Mat& mask, const cv::Scalar& color, int stripe_period, int stripe_width,
                                   double opacity) {
  if (!active_ || mask.empty()) {
    return;
  }

  // Diagonal stripes drawn as a handful of thick lines (not a per-pixel
  // loop), then intersected with the mask so only the striped sub-region
  // gets tinted.
  cv::Mat hatch = cv::Mat::zeros(mask.size(), CV_8UC1);
  for (int offset = -mask.rows; offset < mask.cols; offset += stripe_period) {
    cv::line(hatch, cv::Point(offset, 0), cv::Point(offset + mask.rows, mask.rows), cv::Scalar(255), stripe_width);
  }

  cv::Mat hatched_mask;
  cv::bitwise_and(mask, hatch, hatched_mask);

  draw_mask(hatched_mask, color, opacity);
}

}  // namespace bitbots_vision
