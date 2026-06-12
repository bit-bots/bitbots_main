#include <gtest/gtest.h>

#include <bitbots_vision/auto_gain_controller.hpp>
#include <stdexcept>

using bitbots_vision::AutoGainConfig;
using bitbots_vision::AutoGainController;

TEST(AutoGainController, HoldsGainInsideDeadband) {
  AutoGainController controller(AutoGainConfig{});
  EXPECT_FALSE(controller.update(105.0, 40).has_value());
  EXPECT_FALSE(controller.update(118.0, 40).has_value());
}

TEST(AutoGainController, RaisesGainForDarkImage) {
  AutoGainController controller(AutoGainConfig{});
  EXPECT_EQ(controller.update(50.0, 40), 44);
}

TEST(AutoGainController, LowersGainForBrightImage) {
  AutoGainController controller(AutoGainConfig{});
  EXPECT_EQ(controller.update(180.0, 40), 36);
}

TEST(AutoGainController, UsesAtLeastOneGainStepOutsideDeadband) {
  AutoGainConfig config;
  config.brightness_deadband = 0.0;
  config.gain_kp = 0.001;
  AutoGainController controller(config);

  EXPECT_EQ(controller.update(109.0, 40), 41);
  EXPECT_EQ(controller.update(111.0, 40), 39);
}

TEST(AutoGainController, RespectsGainLimits) {
  AutoGainController controller(AutoGainConfig{});
  EXPECT_FALSE(controller.update(0.0, 100).has_value());
  EXPECT_FALSE(controller.update(255.0, 0).has_value());
}

TEST(AutoGainController, RejectsInvalidConfiguration) {
  AutoGainConfig config;
  config.min_gain = 80;
  config.max_gain = 20;
  EXPECT_THROW(AutoGainController controller(config), std::invalid_argument);
}
