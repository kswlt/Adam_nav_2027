#include "rm_nav_registration/kiss_gicp_backend.hpp"
#include "rm_nav_registration/localization_validator.hpp"
#include <iostream>
#include <random>
#include <limits>
#include <stdexcept>

void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

int main()
{
  using namespace rm_nav_registration;
  KissGicpConfig config;
  config.refinement.num_threads = 2;
  config.refinement.voxel_resolution = 0.08;
  config.refinement.max_correspondence_distance = 0.5;
  KissGicpBackend backend(config);
  RegistrationRequest request;
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = (Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY())).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(3.0, -1.0, 0.4);
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uniform(-5, 5);
  for (int i = 0; i < 5000; ++i) {
    const Eigen::Vector3d point(uniform(rng), uniform(rng), uniform(rng));
    request.source_points.push_back(point);
    request.target_points.push_back(expected * point);
  }
  const auto result = backend.register_clouds(request);
  const auto error = Eigen::Isometry3d(expected.inverse() * result.target_T_source);
  std::cout << "KISS_GICP translation_error_m=" << error.translation().norm()
            << " angle_error_rad=" << Eigen::AngleAxisd(error.rotation()).angle()
            << " rmse_m=" << result.residual << " inlier_ratio=" << result.inlier_ratio
            << " runtime_ms=" << result.runtime_ms << '\n';
  require(result.method == RegistrationMethod::KISS_GICP && result.converged, "KISS/GICP did not converge");
  require(error.translation().norm() < 0.02 && Eigen::AngleAxisd(error.rotation()).angle() < 0.01,
          "KISS/GICP recovered wrong transform");
  require(result.inlier_ratio > 0.9 && result.residual < 0.03, "KISS/GICP has insufficient quality");
  require(result.translation_delta > 3 && result.rotation_delta > 1, "Correction delta lost during refinement");
  require(validate(result, {}).state == ValidationState::CANDIDATE, "Large recovery automatically accepted");
  require(!backend.register_clouds({}).converged, "Empty KISS cloud accepted");
  auto invalid = request;
  invalid.source_points[0].z() = std::numeric_limits<double>::quiet_NaN();
  require(!backend.register_clouds(invalid).converged, "NaN KISS cloud accepted");
  invalid = request;
  invalid.initial_target_T_source.linear()(0, 0) = 2;
  require(!backend.register_clouds(invalid).converged, "Invalid prior accepted");
  std::cout << "PASS real KISS coarse registration, GICP refinement and large-correction gate\n";
}
