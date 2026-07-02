#pragma once

#include <bitbots_vision/candidate.hpp>
#include <opencv2/core.hpp>
#include <vector>

/// Classical HSV-based field-line segmentation.
///
/// A naive white-color threshold is not enough to find field lines: the
/// background (crowd, advertising boards, other robots' white parts, ...)
/// often contains white too. Instead, field lines are treated geometrically:
/// the field is the (roughly convex, from a downward/forward-facing robot
/// camera) region enclosed by the green turf's outer boundary, and anything
/// inside that region that isn't green -- lines, the ball, robots standing
/// on the field -- is a "hole". A candidate white pixel only counts as a
/// line pixel if it falls inside that filled-in field region. Detected
/// objects (ball/goalpost/robot) are then painted out of the result so they
/// cannot register as line pixels.
///
/// This assumes the visible field boundary is roughly convex and that
/// background clutter (crowd, banners, ...) lies outside it rather than
/// being enclosed by it -- generally true for a flat carpet viewed from a
/// downward/forward-facing robot camera, but can over-include background if
/// the visible green region is heavily occluded or non-convex in a given
/// frame.
namespace bitbots_vision::processing {

struct HsvBounds {
  cv::Scalar low;   ///< H, S, V lower bound (inclusive)
  cv::Scalar high;  ///< H, S, V upper bound (inclusive)
};

struct LineSegmentationParams {
  HsvBounds white;
  HsvBounds green;
  int morph_kernel_size{3};       ///< opening kernel size (removes salt/pepper noise)
  int min_contour_area{20};       ///< minimum contour area (px^2) to keep; 0 disables
  int field_border_margin_px{5};  ///< pixels to erode the field hull inward by; 0 disables
};

/// Threshold a BGR image in HSV space for white and green pixels.
void compute_white_green_masks(const cv::Mat& bgr, const LineSegmentationParams& params, cv::Mat& white_mask,
                                cv::Mat& green_mask);

/// Fill in the field's outer boundary: compute the convex hull over every
/// connected component of `green_mask` that is a substantial fraction of the
/// largest one, and rasterize that hull as a solid mask. Merging comparably
/// large components (rather than using only the single largest) bridges a
/// field that a full-width line or the field boundary has split into
/// disconnected pieces, while small, unrelated green specks elsewhere in the
/// image (e.g. background color drift) stay excluded. Returns an all-zero
/// mask if no green contour is found (nothing looks like a field, so no
/// lines can be found either).
///
/// The hull is then eroded inward by `border_margin_px`: a convex hull can
/// bulge slightly past the true (possibly non-convex) field boundary at
/// concave points -- e.g. where the boundary is occluded or the field edge
/// itself curves inward -- so background right outside the real edge (an
/// advertising board, boundary tape, ...) can end up just inside the hull.
/// Shrinking the hull's border trades a small strip of near-boundary line
/// detection for suppressing those artifacts.
cv::Mat compute_field_hull_mask(const cv::Mat& green_mask, int border_margin_px);

/// Combine white_mask & field_mask & ~green_mask, then apply basic
/// filtering: morphological opening (removes salt/pepper noise) followed by
/// optional small-contour-area removal.
///
/// The explicit `& ~green_mask` subtraction guards against overlapping or
/// loosely-configured HSV bounds: `field_mask` (the filled hull) includes
/// the actual green turf pixels as well as the holes in it, so without this
/// subtraction a pixel that happens to pass both the white and green
/// thresholds (e.g. a bright turf highlight) would be misclassified as a
/// line pixel.
cv::Mat compute_line_mask(const cv::Mat& white_mask, const cv::Mat& field_mask, const cv::Mat& green_mask,
                          int morph_kernel_size, int min_contour_area);

/// Paint out (set to 0) the regions covered by the given candidates, each
/// inflated by `margin_px` on all sides and clamped to the mask bounds.
/// Modifies `line_mask` in place.
void suppress_candidates(cv::Mat& line_mask, const std::vector<Candidate>& candidates, int margin_px);

/// Full line-segmentation pipeline: HSV thresholding, green-flanking test,
/// basic filtering, and object-detection suppression.
///
/// @param bgr                    Input image (CV_8UC3, BGR)
/// @param params                 HSV bounds / filtering parameters
/// @param suppress_candidates_list  Detected ball/goalpost/robot candidates to paint out
/// @param suppress_margin_px     Pixel margin to inflate each candidate's bbox by
/// @return                       CV_8UC1 binary mask (0 / 255)
cv::Mat segment_lines(const cv::Mat& bgr, const LineSegmentationParams& params,
                      const std::vector<Candidate>& suppress_candidates_list, int suppress_margin_px);

}  // namespace bitbots_vision::processing
