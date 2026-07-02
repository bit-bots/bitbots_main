#pragma once

#include <bitbots_vision/candidate.hpp>
#include <opencv2/core.hpp>

namespace bitbots_vision {

/// Draws detection and segmentation results on a copy of the input image.
class DebugImage {
 public:
  // BGR color constants (cv::Scalar is not a literal type, so inline static const)
  inline static const cv::Scalar kBall{0, 255, 0};                // green
  inline static const cv::Scalar kRobotTeamMates{255, 255, 102};  // cyan
  inline static const cv::Scalar kRobotOpponents{153, 51, 255};   // magenta
  inline static const cv::Scalar kRobotUnknown{160, 160, 160};    // grey
  inline static const cv::Scalar kGoalposts{255, 255, 255};       // white
  inline static const cv::Scalar kLines{255, 0, 0};               // blue
  inline static const cv::Scalar kField{0, 140, 255};              // orange -- distinct from the green turf itself

  explicit DebugImage(bool active = false) : active_(active) {}

  void set_image(const cv::Mat& image);

  /// Draw a circle around each candidate (suitable for ball-shaped objects).
  void draw_ball_candidates(const std::vector<Candidate>& candidates, const cv::Scalar& color, int thickness = 1);

  /// Draw a bounding rectangle for each candidate (suitable for robots, goalposts).
  void draw_box_candidates(const std::vector<Candidate>& candidates, const cv::Scalar& color, int thickness = 1);

  /// Blend a segmentation mask over the debug image.
  /// @param mask  CV_8UC1 binary mask (non-zero = active)
  void draw_mask(const cv::Mat& mask, const cv::Scalar& color, double opacity = 0.5);

  /// Blend a segmentation mask over the debug image using a diagonal hatch
  /// pattern instead of a solid fill -- useful for large-area masks (e.g.
  /// the field) where a solid tint would obscure the underlying image or be
  /// hard to distinguish from a similarly-colored real object.
  /// @param mask          CV_8UC1 binary mask (non-zero = active)
  /// @param stripe_period Spacing in pixels between diagonal stripes
  /// @param stripe_width  Thickness in pixels of each stripe
  void draw_mask_hatched(const cv::Mat& mask, const cv::Scalar& color, int stripe_period = 16, int stripe_width = 4,
                        double opacity = 0.6);

  const cv::Mat& get_image() const { return image_; }

 private:
  bool active_{false};
  cv::Mat image_;
};

}  // namespace bitbots_vision
