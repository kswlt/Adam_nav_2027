#include "small_point_lio/dense_projection.h"
#include "util/voxelgrid_sampling.h"
#include <iostream>
#include <stdexcept>

void check(bool condition, const char * reason) {
  if (!condition) throw std::runtime_error(reason);
}

int main() {
  small_point_lio::state source;
  source.position << 1., 2., 3.;
  source.velocity << .6, -.2, .1;
  source.omg << 0., 0., 8.;
  source.rotation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto original = source;
  const double dt = .004;
  const Eigen::Vector3d world_point(5., -3., 1.);
  const Eigen::Matrix3d future_rotation = source.rotation *
      Eigen::AngleAxisd(8.*dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d acquired_point = future_rotation.transpose() *
      (world_point-source.position-source.velocity*dt);
  const auto projected = small_point_lio::project_dense_point(source, acquired_point, dt);
  check((projected-world_point).norm() < 1e-10, "Rotating/translating static wall projection failed");
  check((source.position+source.rotation*acquired_point-world_point).norm() > .1,
        "Fixture did not expose use of stale pose");
  check((source.position-original.position).norm() == 0 &&
        (source.rotation-original.rotation).norm() == 0 &&
        (source.velocity-original.velocity).norm() == 0, "Publication changed filter state");
  check((small_point_lio::project_dense_point(source, acquired_point, 0.)-
        (source.position+source.rotation*acquired_point)).norm() < 1e-12, "Zero-time projection changed");
  bool rejected = false;
  try { small_point_lio::project_dense_point(source, acquired_point, -.001); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, "Negative dense time was accepted");

  const std::vector<common::Point> input = {
      {1., {.02f, .03f, 0.f}}, {2., {.07f, .03f, 0.f}},
      {3., {.18f, .04f, 0.f}}, {4., {1.f, .03f, 0.f}}};
  util::VoxelgridSampling sampler;
  std::vector<common::Point> result;
  sampler.voxelgrid_sampling(input, result, .2);
  check(result.size() == 2, "Voxel occupancy changed");
  for (const auto & point : result) {
    bool real_return = false;
    for (const auto & original_point : input) {
      if ((point.position-original_point.position).norm() == 0 &&
          point.timestamp == original_point.timestamp) real_return = true;
    }
    check(real_return, "Sample mixed averaged coordinates with an unrelated acquisition time");
  }
  const auto selected = result;
  sampler.voxelgrid_sampling(input, result, .2);
  check(result.size() == selected.size(), "Sampling count is nondeterministic");
  for (size_t i = 0; i < result.size(); ++i)
    check(result[i].timestamp == selected[i].timestamp &&
          (result[i].position-selected[i].position).norm() == 0, "Sampling is nondeterministic");
  sampler.voxelgrid_sampling({}, result, .2);
  check(result.empty(), "Empty input retained previous samples");
  std::cout << "PASS exact-time wall projection, immutable filter, real paired voxel returns\n";
}
