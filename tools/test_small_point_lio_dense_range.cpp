#include "small_point_lio/preprocess.h"
#include <iostream>
#include <stdexcept>

// Calls the actual upstream preprocessing implementation, not a copied model.
int main() {
  small_point_lio::Parameters parameters;
  parameters.point_filter_num = 1;
  parameters.space_downsample = false;
  parameters.min_distance_squared = .25;
  parameters.max_distance_squared = 900.;
  small_point_lio::Preprocess preprocessing;
  preprocessing.parameters = &parameters;
  preprocessing.reset();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const double invalid_time = std::numeric_limits<double>::quiet_NaN();
  std::vector<common::Point> input = {
      {1., {0.f, 0.f, 0.f}}, {1.1, {.1f, 0.f, 0.f}},
      {2., {2.f, 0.f, 0.f}}, {1.5, {31.f, 0.f, 0.f}},
      {1.7, {nan, 0.f, 0.f}}, {invalid_time, {3.f, 0.f, 0.f}},
      {1.8, {1.f, 0.f, 0.f}}};
  preprocessing.on_point_cloud_callback(input);
  for (const auto * queue : {&preprocessing.point_deque, &preprocessing.dense_point_deque}) {
    if (queue->size() != 2 || queue->front().timestamp != 1.8 || queue->back().timestamp != 2.)
      throw std::runtime_error("Invalid return leaked, valid return lost, or source time changed");
    if (queue->front().position.x() != 1.f || queue->back().position.x() != 2.f)
      throw std::runtime_error("Valid point coordinates changed");
  }
  // All-invalid scans may not inject a moving copy of the sensor origin into maps.
  preprocessing.on_point_cloud_callback({{3., {0.f, 0.f, 0.f}}});
  if (preprocessing.dense_point_deque.size() != 2)
    throw std::runtime_error("No-return scan added map points");
  std::cout << "PASS actual upstream: dense/update range, finite values, source times, no-return scan\n";
}
