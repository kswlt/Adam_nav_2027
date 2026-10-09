#include "rm_nav_mapping/loop_validator.hpp"
#include <cassert>
using namespace rm_nav_mapping;
rm_nav_registration::RegistrationResult good() {
  rm_nav_registration::RegistrationResult r; r.converged=true;r.inlier_count=500;r.inlier_ratio=.8;
  r.residual=.05;r.condition_score=.1;r.confidence=.9;return r;
}
int main() {
  LoopValidationInput in;in.forward=good();in.reverse=good();in.overlap_forward=in.overlap_reverse=.8;in.geometry_ratio=.1;
  auto ok=validate_loop(in);assert(ok.accepted);
  in.forward.residual=.3;assert(!validate_loop(in).accepted);in.forward=good();
  in.overlap_reverse=.1;assert(!validate_loop(in).accepted);in.overlap_reverse=.8;
  in.geometry_ratio=1e-8;assert(!validate_loop(in).accepted);in.geometry_ratio=.1;
  in.reverse.target_T_source.translation().x()=.2;assert(!validate_loop(in).accepted);
  in.reverse=good();in.forward.converged=false;assert(!validate_loop(in).accepted);
}
