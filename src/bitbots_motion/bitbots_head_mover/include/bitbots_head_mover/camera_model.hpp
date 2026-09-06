#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>

namespace image_geometry {
class PinholeCameraModel;
}

/// Projection of world points into the camera image.
namespace bitbots_head_mover {

/// How a point's position inside the image is turned into a score.
struct VisibilityWeighting {
  /// Fraction of the image, measured from its center, that counts as perfectly
  /// visible. A value of one half means the central 50% of the image.
  double center_fraction = 0.5;
  /// Score of a point sitting exactly on the image border. Points outside the
  /// image always score zero, so this is the score a barely visible point gets.
  double border_score = 0.3;
};

/// Tests whether points are visible to the camera and how well they are framed.
///
/// Wraps image_geometry::PinholeCameraModel so the rest of the head mover works
/// with Eigen vectors and does not have to know about OpenCV. All points passed
/// in are expected to be expressed in the camera's optical frame, i.e. with x to
/// the right, y down and z forward.
class CameraModel {
 public:
  CameraModel();
  ~CameraModel();
  CameraModel(CameraModel&&) noexcept;
  CameraModel& operator=(CameraModel&&) noexcept;

  /// Adopt new intrinsics.
  ///
  /// Returns false and leaves the model unchanged if the message does not carry
  /// usable intrinsics, which is the case before the camera driver is calibrated.
  bool update(const sensor_msgs::msg::CameraInfo& info);

  /// Whether usable intrinsics were received.
  bool valid() const { return valid_; }

  /// The image dimensions in pixels.
  double width() const { return width_; }
  double height() const { return height_; }

  /// Project a point in the optical frame to pixel coordinates.
  ///
  /// Returns nullopt if the point is behind the camera or lands outside the
  /// image bounds.
  std::optional<Eigen::Vector2d> project(const Eigen::Vector3d& point) const;

  /// Score how well a point in the optical frame is framed, in [0, 1].
  ///
  /// Points that are not visible score zero. Visible points score one inside the
  /// central region and fall off linearly towards the border score at the image
  /// edge, so that centering an object is preferred over merely catching it.
  double visibility(const Eigen::Vector3d& point, const VisibilityWeighting& weighting = {}) const;

 private:
  /// Project a point in the optical frame, without bounds checking.
  ///
  /// Returns false if the point is behind the camera. Kept inline and free of
  /// OpenCV types because the scoring calls it for every coverage cell of every
  /// evaluated point of every candidate, which made it the single hottest path
  /// in a planning cycle.
  bool projectRaw(const Eigen::Vector3d& point, double& u, double& v) const {
    if (point.z() <= 0.0) {
      return false;
    }
    const double inverse_z = 1.0 / point.z();
    u = (fx_ * point.x() + tx_) * inverse_z + cx_;
    v = (fy_ * point.y() + ty_) * inverse_z + cy_;
    return true;
  }

  /// The rectified pinhole model, which stays the authority on the intrinsics.
  std::unique_ptr<image_geometry::PinholeCameraModel> model_;
  bool valid_ = false;
  double width_ = 0.0;
  double height_ = 0.0;

  // The model's rectified parameters, cached for the hot path above. They are
  // taken from the model rather than from the message so that binning and a
  // region of interest stay accounted for.
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  double tx_ = 0.0;
  double ty_ = 0.0;
  double inverse_half_width_ = 0.0;
  double inverse_half_height_ = 0.0;
};

}  // namespace bitbots_head_mover
