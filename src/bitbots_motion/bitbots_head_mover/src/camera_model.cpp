#include <algorithm>
#include <bitbots_head_mover/camera_model.hpp>
#include <cmath>
#include <image_geometry/pinhole_camera_model.hpp>

namespace bitbots_head_mover {

CameraModel::CameraModel() : model_(std::make_unique<image_geometry::PinholeCameraModel>()) {}
CameraModel::~CameraModel() = default;
CameraModel::CameraModel(CameraModel&&) noexcept = default;
CameraModel& CameraModel::operator=(CameraModel&&) noexcept = default;

bool CameraModel::update(const sensor_msgs::msg::CameraInfo& info) {
  // An uncalibrated driver publishes zeroed intrinsics, which would make every
  // projection collapse onto the principal point
  if (info.width == 0 || info.height == 0 || info.k[0] == 0.0 || info.k[4] == 0.0) {
    return false;
  }

  // The return value reports whether the intrinsics differ from the previous
  // ones, not whether they were accepted, so it is deliberately ignored here.
  // The validation above is what decides whether the model is usable.
  model_->fromCameraInfo(info);

  // Cache the model's rectified parameters for the hot projection path. Reading
  // them through the model's accessors keeps binning and a region of interest
  // accounted for, which the raw message fields would not.
  fx_ = model_->fx();
  fy_ = model_->fy();
  cx_ = model_->cx();
  cy_ = model_->cy();
  tx_ = model_->Tx();
  ty_ = model_->Ty();

  // A rectified model without focal lengths would project everything onto the
  // principal point, which the check above cannot catch for all message layouts
  if (fx_ == 0.0 || fy_ == 0.0) {
    valid_ = false;
    return false;
  }

  width_ = static_cast<double>(info.width);
  height_ = static_cast<double>(info.height);
  inverse_half_width_ = 2.0 / width_;
  inverse_half_height_ = 2.0 / height_;
  valid_ = true;
  return true;
}

std::optional<Eigen::Vector2d> CameraModel::project(const Eigen::Vector3d& point) const {
  double u = 0.0;
  double v = 0.0;
  // Points at or behind the image plane have no meaningful projection
  if (!valid_ || !projectRaw(point, u, v)) {
    return std::nullopt;
  }

  if (!std::isfinite(u) || !std::isfinite(v)) {
    return std::nullopt;
  }

  if (u < 0.0 || u > width_ || v < 0.0 || v > height_) {
    return std::nullopt;
  }

  return Eigen::Vector2d(u, v);
}

double CameraModel::visibility(const Eigen::Vector3d& point, const VisibilityWeighting& weighting) const {
  double u = 0.0;
  double v = 0.0;
  if (!valid_ || !projectRaw(point, u, v)) {
    return 0.0;
  }

  // Offset from the image center, normalized so that the image border sits at
  // one. Anything beyond that is off the image and therefore not visible, which
  // also covers the non finite case.
  const double offset_x = std::abs(u * inverse_half_width_ - 1.0);
  const double offset_y = std::abs(v * inverse_half_height_ - 1.0);
  // The larger of the two axes decides, so the perfectly framed region is a box
  // rather than an ellipse and a point is only "centered" if it is centered in
  // both directions
  const double offset = std::max(offset_x, offset_y);
  if (!(offset <= 1.0)) {
    return 0.0;
  }

  const double center_fraction = std::clamp(weighting.center_fraction, 0.0, 1.0);
  if (offset <= center_fraction) {
    return 1.0;
  }

  // Fall off linearly from the edge of the central region to the image border.
  // The central region covering the whole image would make this a zero division.
  if (center_fraction >= 1.0) {
    return 1.0;
  }
  const double falloff = (offset - center_fraction) / (1.0 - center_fraction);
  return 1.0 + falloff * (weighting.border_score - 1.0);
}

}  // namespace bitbots_head_mover
