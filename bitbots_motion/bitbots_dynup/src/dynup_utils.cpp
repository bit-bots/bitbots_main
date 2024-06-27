#include "bitbots_dynup/dynup_utils.hpp"

namespace bitbots_dynup {

DynupDirection getDynupDirection(const std::string & direction) {
  std::map<const char*, DynupDirection> mapping = {
      {"front", FRONT}, {"back", BACK},       {"front_only", FRONT_ONLY}, {"back_only", BACK_ONLY},
      {"rise", RISE},   {"descend", DESCEND}, {"walkready", WALKREADY},
  };
  return mapping.at(direction.c_str());
}

}  // namespace bitbots_dynup
