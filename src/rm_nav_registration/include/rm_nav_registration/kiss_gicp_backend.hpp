#pragma once
#include "rm_nav_registration/small_gicp_backend.hpp"

namespace rm_nav_registration {
struct KissGicpConfig {
  double feature_resolution{0.3};
  int num_threads{2};
  std::size_t max_input_points{50000};
  std::size_t min_final_inliers{5};
  SmallGicpConfig refinement{};
};

// Caller supplies the policy-restricted candidate target. No whole-field search
// or map->odom mutation is performed by this library.
class KissGicpBackend final : public RegistrationBackend {
public:
  explicit KissGicpBackend(const KissGicpConfig & config = {});
  RegistrationResult register_clouds(const RegistrationRequest & request) override;
private:
  KissGicpConfig config_;
  SmallGicpBackend refiner_;
};
}  // namespace rm_nav_registration
