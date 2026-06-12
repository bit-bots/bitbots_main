#include <algorithm>
#include <bitbots_vision/auto_gain_controller.hpp>
#include <cmath>
#include <stdexcept>

namespace bitbots_vision {

AutoGainController::AutoGainController(AutoGainConfig config) : config_(config) {
  if (config_.target_brightness < 0.0 || config_.target_brightness > 255.0) {
    throw std::invalid_argument("target brightness must be between 0 and 255");
  }
  if (config_.brightness_deadband < 0.0 || config_.gain_kp <= 0.0 || config_.max_gain_step <= 0 ||
      config_.min_gain < 0 || config_.max_gain > 100 || config_.min_gain > config_.max_gain) {
    throw std::invalid_argument("invalid auto gain configuration");
  }
}

std::optional<int> AutoGainController::update(double brightness, int current_gain) const {
  const double error = config_.target_brightness - brightness;
  if (std::abs(error) <= config_.brightness_deadband) {
    return std::nullopt;
  }

  int step = static_cast<int>(std::round(config_.gain_kp * error));
  if (step == 0) {
    step = error > 0.0 ? 1 : -1;
  }
  step = std::clamp(step, -config_.max_gain_step, config_.max_gain_step);

  const int new_gain = std::clamp(current_gain + step, config_.min_gain, config_.max_gain);
  if (new_gain == current_gain) {
    return std::nullopt;
  }
  return new_gain;
}

}  // namespace bitbots_vision
