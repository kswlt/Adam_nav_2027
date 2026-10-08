#include "rm_nav_sensors/observation_batch.hpp"

#include <cassert>

int main()
{
  rm_nav_sensors::ObservationBatch batch;
  assert(!batch.valid());
  rm_nav_sensors::ObservationFrame frame;
  frame.source_id = "front_mid360";
  frame.frame_id = "front_mid360";
  frame.healthy = true;
  batch.frames.push_back(frame);
  assert(batch.valid());
  return 0;
}
