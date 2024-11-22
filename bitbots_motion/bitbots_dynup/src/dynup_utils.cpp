#include "bitbots_dynup/dynup_utils.hpp"

namespace bitbots_dynup {

DynupDirection getDynupDirection(const std::string& direction) {
  std::map<const std::string, DynupDirection> mapping = {
      {"front", FRONT}, {"back", BACK},       {"front_only", FRONT_ONLY}, {"back_only", BACK_ONLY},
      {"rise", RISE},   {"descend", DESCEND}, {"walkready", WALKREADY},
  };
  try {
    return mapping.at(direction);
  } catch (const std::out_of_range& e) {
    throw std::invalid_argument("Invalid direction: '" + direction + "'");
  }
}

}  // namespace bitbots_dynup
