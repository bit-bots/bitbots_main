#pragma once

#include <optional>

namespace bitbots_vision {

struct AutoGainConfig {
  double target_brightness{110.0};
  double brightness_deadband{8.0};
  double gain_kp{0.08};
  int max_gain_step{4};
  int min_gain{0};
  int max_gain{100};
};

class AutoGainController {
 public:
  explicit AutoGainController(AutoGainConfig config);

  /// Returns a new gain when the brightness error requires an adjustment.
  std::optional<int> update(double brightness, int current_gain) const;

 private:
  AutoGainConfig config_;
};

}  // namespace bitbots_vision
