#include "rm_nav_registration/small_gicp_backend.hpp"
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace rm_nav_registration {

SmallGicpBackend::SmallGicpBackend(const SmallGicpConfig & config) : config_(config)
{
  for (double value : {config.voxel_resolution, config.max_correspondence_distance,
                       config.translation_epsilon, config.rotation_epsilon}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("GICP limits must be finite and positive");
  }
  if (config.num_threads < 1 || config.max_iterations < 1 || config.min_points < 10 ||
      config.max_input_points < config.min_points) throw std::invalid_argument("Invalid GICP point/thread limits");
}

RegistrationResult SmallGicpBackend::register_clouds(const RegistrationRequest & request)
{
  RegistrationResult result;
  result.method = RegistrationMethod::LOCAL_GICP;
  result.residual = result.fitness = std::numeric_limits<double>::infinity();
  const auto started = std::chrono::steady_clock::now();
  const auto finish = [&]() {
    result.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return result;
  };
  if (!valid_rigid_transform(request.initial_target_T_source)) return finish();
  const auto valid_cloud = [&](const auto & cloud) {
    if (cloud.size() < config_.min_points || cloud.size() > config_.max_input_points) return false;
    for (const auto & point : cloud) if (!point.allFinite()) return false;
    return true;
  };
  if (!valid_cloud(request.target_points) || !valid_cloud(request.source_points)) return finish();
  auto [target, tree] = small_gicp::preprocess_points(
      request.target_points, config_.voxel_resolution, 10, config_.num_threads);
  auto [source, source_tree] = small_gicp::preprocess_points(
      request.source_points, config_.voxel_resolution, 10, config_.num_threads);
  result.target_point_count = target->size();
  result.source_point_count = source->size();
  if (target->size() < config_.min_points || source->size() < config_.min_points) return finish();
  small_gicp::RegistrationSetting settings;
  settings.type = small_gicp::RegistrationSetting::GICP;
  settings.num_threads = config_.num_threads;
  settings.max_iterations = config_.max_iterations;
  settings.max_correspondence_distance = config_.max_correspondence_distance;
  settings.translation_eps = config_.translation_epsilon;
  settings.rotation_eps = config_.rotation_epsilon;
  auto raw = small_gicp::align(*target, *source, *tree,
                                    request.initial_target_T_source, settings);
  // LM can reject every trial at an already stationary warm start because
  // roundoff makes new_e > e. Never infer convergence from low RMSE alone.
  // Only for a finite, observable stationary system, run one genuine upstream
  // Gauss-Newton step with the same points, correspondences and tolerances.
  if (!raw.converged && valid_rigid_transform(raw.T_target_source) &&
      raw.H.allFinite() && raw.b.allFinite() && std::isfinite(raw.error)) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig((raw.H + raw.H.transpose()) * 0.5);
    if (eig.info() == Eigen::Success && eig.eigenvalues().maxCoeff() > 0 &&
        eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() >= 1e-6) {
      const Eigen::Matrix<double, 6, 1> step = raw.H.ldlt().solve(-raw.b);
      if (step.allFinite() && step.head<3>().norm() <= config_.rotation_epsilon &&
          step.tail<3>().norm() <= config_.translation_epsilon) {
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP,
          small_gicp::NullFactor, small_gicp::DistanceRejector, small_gicp::GaussNewtonOptimizer> check;
        check.reduction.num_threads = config_.num_threads;
        check.rejector.max_dist_sq = settings.max_correspondence_distance * settings.max_correspondence_distance;
        check.criteria.rotation_eps = config_.rotation_epsilon;
        check.criteria.translation_eps = config_.translation_epsilon;
        check.optimizer.max_iterations = 1;
        const auto checked = check.align(*target, *source, *tree, raw.T_target_source);
        if (checked.converged && std::isfinite(checked.error) &&
            checked.error <= raw.error + 1e-9 * std::max(1.0, std::abs(raw.error))) raw = checked;
      }
    }
  }
  result.target_T_source = raw.T_target_source;
  result.hessian = raw.H;
  result.iterations = raw.iterations;
  if (!valid_rigid_transform(raw.T_target_source) || !raw.H.allFinite()) return finish();
  // Report Euclidean nearest-neighbor RMSE in meters, rather than labeling
  // the upstream Mahalanobis objective as a metric distance.
  double squared_sum = 0;
  const double max_squared = config_.max_correspondence_distance * config_.max_correspondence_distance;
  for (std::size_t i = 0; i < source->size(); ++i) {
    const Eigen::Vector4d transformed = raw.T_target_source * source->point(i);
    std::size_t index = 0;
    double distance = 0;
    if (tree->knn_search(transformed, 1, &index, &distance) && distance <= max_squared) {
      ++result.inlier_count;
      squared_sum += distance;
    }
  }
  if (result.inlier_count == 0) return finish();
  result.residual = std::sqrt(squared_sum / result.inlier_count);
  result.fitness = squared_sum / result.inlier_count;  // mean squared distance, m^2
  result.inlier_ratio = static_cast<double>(result.inlier_count) / source->size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig((raw.H + raw.H.transpose()) * 0.5);
  if (eig.info() == Eigen::Success && eig.eigenvalues().maxCoeff() > 0) {
    result.condition_score = std::max(0.0, eig.eigenvalues().minCoeff()) / eig.eigenvalues().maxCoeff();
  }
  const Eigen::Isometry3d correction = request.initial_target_T_source.inverse() * raw.T_target_source;
  result.translation_delta = correction.translation().norm();
  result.rotation_delta = Eigen::AngleAxisd(correction.rotation()).angle();
  result.confidence = result.inlier_ratio;  // quality proxy, not a calibrated probability
  result.converged = raw.converged;
  return finish();
}

}  // namespace rm_nav_registration
