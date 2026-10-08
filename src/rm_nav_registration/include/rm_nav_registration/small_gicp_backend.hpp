#pragma once
#include "rm_nav_registration/registration_backend.hpp"

namespace rm_nav_registration {

struct SmallGicpConfig {
  double voxel_resolution{0.05};
  double max_correspondence_distance{0.5};
  double translation_epsilon{0.001};
  double rotation_epsilon{0.001};
  int num_threads{2};
  int max_iterations{40};
  std::size_t min_points{20};
  std::size_t max_input_points{1000000};
};

class SmallGicpBackend final : public RegistrationBackend {
public:
  explicit SmallGicpBackend(const SmallGicpConfig & config = {});
  RegistrationResult register_clouds(const RegistrationRequest & request) override;
private:
  SmallGicpConfig config_;
};

}  // namespace rm_nav_registration
