#pragma once

#include <algorithm>
#include <vector>

namespace bitbots_vision {

/// Axis-aligned bounding box with a confidence rating, in pixel coordinates.
struct Candidate {
  int x1{0};      ///< Upper-left x
  int y1{0};      ///< Upper-left y
  int width{0};   ///< Box width
  int height{0};  ///< Box height
  float rating{0.f};

  static Candidate from_x1y1x2y2(int x1, int y1, int x2, int y2, float rating)
  {
    return {std::min(x1, x2), std::min(y1, y2), std::abs(x2 - x1), std::abs(y2 - y1), rating};
  }

  int center_x() const { return x1 + width / 2; }
  int center_y() const { return y1 + height / 2; }
  int x2() const { return x1 + width; }
  int y2() const { return y1 + height; }
  int radius() const { return (width + height) / 4; }

  static std::vector<Candidate> sort_by_rating(std::vector<Candidate> candidates)
  {
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate & a, const Candidate & b) { return a.rating > b.rating; });
    return candidates;
  }

  static std::vector<Candidate> filter_by_rating(
    const std::vector<Candidate> & candidates, float threshold)
  {
    std::vector<Candidate> result;
    for (const auto & c : candidates) {
      if (c.rating > threshold) {
        result.push_back(c);
      }
    }
    return result;
  }
};

}  // namespace bitbots_vision
