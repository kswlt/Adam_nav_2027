#include "small_point_lio/dense_projection.h"
#include <iostream>
#include <stdexcept>

int main() {
  small_point_lio::state source;
  source.position << 1., 2., 3.;
  source.velocity << .6, -.2, .1;
  source.omg << 0., 0., 8.;
  source.rotation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto original = source;
  const double dt = .004;
  const Eigen::Vector3d wall(5., -3., 1.);
  const Eigen::Matrix3d rotation_at_sample = source.rotation *
      Eigen::AngleAxisd(8.*dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d measured = rotation_at_sample.transpose() *
      (wall-source.position-source.velocity*dt);
  if ((small_point_lio::project_dense_point(source, measured, dt)-wall).norm() > 1e-10)
    throw std::runtime_error("Known static wall moved under constant-rate sensor motion");
  if ((source.position+source.rotation*measured-wall).norm() < .1)
    throw std::runtime_error("Fixture does not distinguish use of stale pose");
  if ((source.position-original.position).norm() != 0 ||
      (source.rotation-original.rotation).norm() != 0 ||
      (source.velocity-original.velocity).norm() != 0)
    throw std::runtime_error("Publication projection mutated the filter");
  if ((small_point_lio::project_dense_point(source, measured, 0.)-
       (source.position+source.rotation*measured)).norm() > 1e-12)
    throw std::runtime_error("Zero-time projection changed");
  for (const double invalid : {-.001, std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { small_point_lio::project_dense_point(source, measured, invalid); }
    catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) throw std::runtime_error("Invalid time accepted");
  }
  std::cout << "PASS actual point-time projection: static wall, immutable filter, invalid times\n";
}
