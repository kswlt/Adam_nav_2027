#include "rm_nav_mapping/map_bundle.hpp"

#include <cassert>

int main()
{
  rm_nav_mapping::MapBundle bundle;
  assert(!bundle.valid());
  bundle.version = "v001";
  bundle.localization_map = "localization_map.pcd";
  bundle.base_occupancy = "base_occupancy.pgm";
  bundle.metadata = "metadata.yaml";
  assert(bundle.valid());
  return 0;
}
