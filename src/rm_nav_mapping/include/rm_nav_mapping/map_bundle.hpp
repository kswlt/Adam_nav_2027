#pragma once

#include <string>

namespace rm_nav_mapping {

struct MapBundle {
  std::string version;
  std::string localization_map;
  std::string base_occupancy;
  std::string terrain_reference;
  std::string semantic_terrain;
  std::string metadata;

  bool valid() const
  {
    return !version.empty() && !localization_map.empty() &&
           !base_occupancy.empty() && !metadata.empty();
  }
};

}  // namespace rm_nav_mapping
