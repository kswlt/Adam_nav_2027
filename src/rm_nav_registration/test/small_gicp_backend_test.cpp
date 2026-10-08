#include "rm_nav_registration/small_gicp_backend.hpp"
#include "rm_nav_registration/localization_validator.hpp"
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

int main()
{
  using namespace rm_nav_registration;
  SmallGicpConfig config;
  config.num_threads = 1;  // deterministic voxel preprocessing for regression
  SmallGicpBackend backend(config);
  RegistrationRequest request;
  std::mt19937 rng(2027);
  std::uniform_real_distribution<double> coordinate(-2, 2);
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() = Eigen::Vector3d(0.12, -0.10, 0.03);
  expected.linear() = (Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY())).toRotationMatrix();
  for (int i = 0; i < 2000; ++i) {
    Eigen::Vector3d point(coordinate(rng), coordinate(rng), coordinate(rng));
    request.source_points.push_back(point);
    request.target_points.push_back(expected * point);
  }
  const auto result = backend.register_clouds(request);
  const Eigen::Isometry3d error = expected.inverse() * result.target_T_source;
  const double position_error = error.translation().norm();
  const double angle_error = Eigen::AngleAxisd(error.rotation()).angle();
  std::cout << "translation_error_m=" << position_error << " rotation_error_rad=" << angle_error
            << " rmse_m=" << result.residual << " inlier_ratio=" << result.inlier_ratio
            << " condition_score=" << result.condition_score << " runtime_ms=" << result.runtime_ms << '\n';
  require(result.converged, "GICP did not converge");
  require(position_error < 0.01 && angle_error < 0.01, "GICP recovered wrong transform direction or pose");
  require(result.source_point_count > 1000 && result.target_point_count > 1000, "Missing cloud metrics");
  require(result.inlier_ratio > 0.9 && result.residual < 0.02, "Incorrect residual/inliers");
  require(result.hessian.norm() > 0 && result.condition_score > 0, "Missing Hessian quality");
  require(validate(result, {}).state == ValidationState::ACCEPTED, "Recovered result not accepted");
  require(!backend.register_clouds({}).converged, "Empty clouds accepted");
  auto invalid = request;
  invalid.source_points[0].x() = std::numeric_limits<double>::quiet_NaN();
  require(!backend.register_clouds(invalid).converged, "NaN cloud accepted");
  invalid = request;
  invalid.initial_target_T_source.linear()(0, 0) = 2;
  require(!backend.register_clouds(invalid).converged, "Nonrigid seed accepted");
  invalid = request;
  for (auto & point : invalid.source_points) point.x() += 100;
  const auto nonoverlap = backend.register_clouds(invalid);
  require(validate(nonoverlap, {}).state == ValidationState::REJECTED, "Nonoverlap accepted");
  invalid = request;
  for (auto & point : invalid.source_points) point.setZero();
  require(!backend.register_clouds(invalid).converged, "Collapsed cloud accepted");
  bool threw = false;
  try {
    config.voxel_resolution = -1;
    SmallGicpBackend bad(config);
  } catch (const std::invalid_argument &) { threw = true; }
  require(threw, "Invalid GICP configuration accepted");
  std::cout << "PASS real small_gicp alignment and rejection checks\n";
}
