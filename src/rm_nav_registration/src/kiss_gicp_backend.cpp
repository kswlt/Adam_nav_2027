#include "rm_nav_registration/kiss_gicp_backend.hpp"
#include <kiss_matcher/KISSMatcher.hpp>
#include <tbb/global_control.h>
#include <omp.h>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace rm_nav_registration {
KissGicpBackend::KissGicpBackend(const KissGicpConfig & config)
: config_(config), refiner_(config.refinement)
{
  if (!std::isfinite(config.feature_resolution) || config.feature_resolution < 0.005 ||
      config.num_threads < 1 || config.min_final_inliers < 3 || config.max_input_points < 20) {
    throw std::invalid_argument("Invalid KISS registration limits");
  }
}

RegistrationResult KissGicpBackend::register_clouds(const RegistrationRequest & request)
{
  const auto started = std::chrono::steady_clock::now();
  RegistrationResult result;
  result.method = RegistrationMethod::KISS_GICP;
  result.fitness = result.residual = std::numeric_limits<double>::infinity();
  const auto finish = [&]() {
    result.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return result;
  };
  if (!valid_rigid_transform(request.initial_target_T_source)) return finish();
  const auto valid_cloud = [&](const auto & cloud) {
    if (cloud.size() < 20 || cloud.size() > config_.max_input_points) return false;
    for (const auto & point : cloud) {
      if (!point.allFinite() || point.cwiseAbs().maxCoeff() > 10000) return false;
    }
    return true;
  };
  if (!valid_cloud(request.source_points) || !valid_cloud(request.target_points)) return finish();
  std::vector<Eigen::Vector3f> source, target;
  source.reserve(request.source_points.size()); target.reserve(request.target_points.size());
  for (const auto & point : request.source_points) source.push_back(point.template cast<float>());
  for (const auto & point : request.target_points) target.push_back(point.template cast<float>());
  struct OpenMpScope {
    int previous{omp_get_max_threads()};
    ~OpenMpScope() { omp_set_num_threads(previous); }
  } omp_scope;
  omp_set_num_threads(config_.num_threads);
  tbb::global_control tbb_scope(tbb::global_control::max_allowed_parallelism, config_.num_threads);
  try {
    kiss_matcher::KISSMatcher matcher(static_cast<float>(config_.feature_resolution));
    // estimate(src, tgt) returns tgt_T_src; never invert it here.
    const auto coarse = matcher.estimate(source, target);
    if (!coarse.valid || matcher.getNumFinalInliers() < config_.min_final_inliers ||
        matcher.getNumRotationInliers() < config_.min_final_inliers) return finish();
    Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();
    seed.linear() = coarse.rotation; seed.translation() = coarse.translation;
    if (!valid_rigid_transform(seed)) return finish();
    RegistrationRequest refined_request = request;
    refined_request.initial_target_T_source = seed;
    result = refiner_.register_clouds(refined_request);
    result.method = RegistrationMethod::KISS_GICP;
    // Report correction from the caller's prior, not only GICP's small refinement.
    const Eigen::Isometry3d delta = request.initial_target_T_source.inverse() * result.target_T_source;
    result.translation_delta = delta.translation().norm();
    result.rotation_delta = Eigen::AngleAxisd(delta.rotation()).angle();
  } catch (const std::exception &) {
    // Missing features or an upstream solver failure must not yield a valid pose.
    result = RegistrationResult{};
    result.method = RegistrationMethod::KISS_GICP;
    result.fitness = result.residual = std::numeric_limits<double>::infinity();
  }
  return finish();
}
}  // namespace rm_nav_registration
