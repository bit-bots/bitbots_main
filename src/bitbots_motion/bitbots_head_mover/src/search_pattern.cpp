#include <algorithm>
#include <bitbots_head_mover/search_pattern.hpp>
#include <cmath>

namespace bitbots_head_mover {

double lineAngle(int line, int line_count, double min_angle, double max_angle) {
  // A single scanline has no span to distribute, so it sits at the first angle.
  // Without this the step size below would divide by zero and yield a NaN angle.
  if (line_count <= 1) {
    return min_angle;
  }
  // Get the angular delta that is covered by the scanlines in the pitch axis
  double delta = std::abs(max_angle - min_angle);
  // Calculate the angular step size between two scanlines
  double steps = delta / (line_count - 1);
  // Calculate the pitch angle of the given scanline
  return steps * line + min_angle;
}

std::vector<HeadPosition> interpolatedSteps(int steps, double pitch, double min_yaw, double max_yaw) {
  // Handle edge case where we do not need to interpolate
  if (steps == 0) {
    return {};
  }
  // Add one to the step count as we need to include the min and max yaw values
  steps += 1;
  // Create a vector that stores the interpolated steps
  std::vector<HeadPosition> output_points;
  // Calculate the delta between the min and max yaw values
  double delta = std::abs(max_yaw - min_yaw);
  // Calculate the step size between two interpolated steps
  double step_size = delta / steps;
  // Iterate over all steps and calculate the interpolated yaw values
  for (int i = 1; i <= steps; i++) {
    double yaw = min_yaw + step_size * i;
    output_points.push_back({yaw, pitch});
  }
  return output_points;
}

std::vector<HeadPosition> generatePattern(int line_count, double max_horizontal_angle_left,
                                          double max_horizontal_angle_right, double max_vertical_angle_up,
                                          double max_vertical_angle_down, double reduce_last_scanline,
                                          int interpolation_steps) {
  // Store the keyframes of the search pattern
  std::vector<HeadPosition> keyframes;
  // Store the state of the generation process
  bool down_direction = true;   // true = decreasing line (toward top), false = increasing line (toward bottom)
  bool right_side = false;      // true = right, false = left
  bool right_direction = true;  // true = moving right, false = moving left; alternates per scan line
  int line = line_count - 1;
  // Calculate the number of iterations that are needed to generate the search pattern
  int iterations = std::max(line_count * 4 - 4, 2);
  // Iterate over all iterations and generate the search pattern
  for (int i = 0; i < iterations; i++) {
    // Get the maximum yaw values (left and right) for the current yaw position
    // Select the relevant one based on the current side we are on
    double current_yaw;
    if (right_side) {
      current_yaw = max_horizontal_angle_right;
    } else {
      current_yaw = max_horizontal_angle_left;
    }

    // Get the current pitch angle based on the current line we are on
    double current_pitch = lineAngle(line, line_count, max_vertical_angle_up, max_vertical_angle_down);

    // Store the keyframe
    keyframes.push_back({current_yaw, current_pitch});

    // Check if we move horizontally or vertically in the pattern
    if (right_side != right_direction) {
      // We move horizontally, so we might need to interpolate between the current and the next keyframe
      std::vector<HeadPosition> interpolated_points =
          interpolatedSteps(interpolation_steps, current_pitch, max_horizontal_angle_right, max_horizontal_angle_left);
      // Reverse the order of the interpolated points if we are moving to the right
      if (right_direction) {
        std::reverse(interpolated_points.begin(), interpolated_points.end());
      }
      // Add the interpolated points to the keyframes
      keyframes.insert(keyframes.end(), interpolated_points.begin(), interpolated_points.end());
      // Change the direction we are moving in
      right_side = right_direction;

    } else {
      // Flip the scan direction so the next line scans the opposite way (boustrophedon)
      right_direction = !right_direction;
      // Advance to the next scan line
      if (down_direction) {
        line -= 1;
      } else {
        line += 1;
      }
      // Flip vertical direction when we reach either edge
      if (line <= 0 || line >= line_count - 1) {
        down_direction = !down_direction;
      }
    }
  }

  // Reduce the last scanline by a given factor
  for (auto& keyframe : keyframes) {
    if (std::abs(keyframe.pitch - max_vertical_angle_down) < 1e-6) {
      keyframe = {keyframe.yaw * reduce_last_scanline, max_vertical_angle_down};
    }
  }
  return keyframes;
}

}  // namespace bitbots_head_mover
