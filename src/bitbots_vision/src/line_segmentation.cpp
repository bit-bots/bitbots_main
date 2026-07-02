#include <algorithm>
#include <bitbots_vision/line_segmentation.hpp>
#include <opencv2/imgproc.hpp>

namespace bitbots_vision::processing {

// ---------------------------------------------------------------------------
// compute_white_green_masks
// ---------------------------------------------------------------------------

void compute_white_green_masks(const cv::Mat& bgr, const LineSegmentationParams& params, cv::Mat& white_mask,
                                cv::Mat& green_mask) {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::inRange(hsv, params.white.low, params.white.high, white_mask);
  cv::inRange(hsv, params.green.low, params.green.high, green_mask);
}

// ---------------------------------------------------------------------------
// compute_field_hull_mask
// ---------------------------------------------------------------------------

cv::Mat compute_field_hull_mask(const cv::Mat& green_mask, int border_margin_px) {
  cv::Mat hull_mask = cv::Mat::zeros(green_mask.size(), CV_8UC1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    return hull_mask;
  }

  double max_area = 0.0;
  for (const auto& c : contours) {
    max_area = std::max(max_area, cv::contourArea(c));
  }
  if (max_area <= 0.0) {
    return hull_mask;
  }

  // A field line (or the field boundary) that runs all the way across the
  // visible green region can split it into multiple disconnected pieces of
  // comparable size -- e.g. the halfway line seen head-on. Using only the
  // single largest contour would then drop the far side of the field
  // entirely. Instead, merge every contour that's a substantial fraction of
  // the largest one, so genuinely separate field pieces get bridged by the
  // hull, while small, unrelated green specks elsewhere in the image (e.g.
  // background color drift) stay excluded.
  constexpr double kRelativeAreaThreshold = 0.2;
  std::vector<cv::Point> points;
  for (const auto& c : contours) {
    if (cv::contourArea(c) >= kRelativeAreaThreshold * max_area) {
      points.insert(points.end(), c.begin(), c.end());
    }
  }

  std::vector<cv::Point> hull;
  cv::convexHull(points, hull);
  cv::fillConvexPoly(hull_mask, hull, cv::Scalar(255));

  if (border_margin_px > 0) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * border_margin_px + 1, 2 * border_margin_px + 1));
    cv::erode(hull_mask, hull_mask, kernel);
  }

  return hull_mask;
}

// ---------------------------------------------------------------------------
// compute_line_mask
// ---------------------------------------------------------------------------

cv::Mat compute_line_mask(const cv::Mat& white_mask, const cv::Mat& field_mask, const cv::Mat& green_mask,
                          int morph_kernel_size, int min_contour_area) {
  cv::Mat line_mask;
  cv::bitwise_and(white_mask, field_mask, line_mask);

  cv::Mat not_green;
  cv::bitwise_not(green_mask, not_green);
  cv::bitwise_and(line_mask, not_green, line_mask);

  if (morph_kernel_size > 1) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size, morph_kernel_size));
    cv::morphologyEx(line_mask, line_mask, cv::MORPH_OPEN, kernel);
  }

  if (min_contour_area > 0) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(line_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filtered = cv::Mat::zeros(line_mask.size(), CV_8UC1);
    for (const auto& contour : contours) {
      if (cv::contourArea(contour) >= static_cast<double>(min_contour_area)) {
        cv::drawContours(filtered, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
      }
    }
    line_mask = filtered;
  }

  return line_mask;
}

// ---------------------------------------------------------------------------
// suppress_candidates
// ---------------------------------------------------------------------------

void suppress_candidates(cv::Mat& line_mask, const std::vector<Candidate>& candidates, int margin_px) {
  const cv::Rect image_bounds(0, 0, line_mask.cols, line_mask.rows);
  for (const auto& c : candidates) {
    cv::Rect box(c.x1 - margin_px, c.y1 - margin_px, c.width + 2 * margin_px, c.height + 2 * margin_px);
    box &= image_bounds;
    if (box.width > 0 && box.height > 0) {
      cv::rectangle(line_mask, box, cv::Scalar(0), cv::FILLED);
    }
  }
}

// ---------------------------------------------------------------------------
// segment_lines
// ---------------------------------------------------------------------------

cv::Mat segment_lines(const cv::Mat& bgr, const LineSegmentationParams& params,
                      const std::vector<Candidate>& suppress_candidates_list, int suppress_margin_px) {
  cv::Mat white_mask, green_mask;
  compute_white_green_masks(bgr, params, white_mask, green_mask);

  cv::Mat field_mask = compute_field_hull_mask(green_mask, params.field_border_margin_px);
  cv::Mat line_mask =
      compute_line_mask(white_mask, field_mask, green_mask, params.morph_kernel_size, params.min_contour_area);

  suppress_candidates(line_mask, suppress_candidates_list, suppress_margin_px);

  return line_mask;
}

}  // namespace bitbots_vision::processing
