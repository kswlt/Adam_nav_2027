#include "rm_nav_registration/small_gicp_backend.hpp"
#include <random>
#include <iostream>
#include <stdexcept>
#include <limits>
void require(bool value,const char * message){if(!value)throw std::runtime_error(message);}
int main()
{
  using namespace rm_nav_registration;
  SmallGicpConfig config;config.use_voxelized_target=true;config.num_threads=1;
  SmallGicpBackend backend(config);RegistrationRequest request;
  std::mt19937 rng(2027);std::uniform_real_distribution<double> coordinate(-2,2);
  auto expected=Eigen::Isometry3d::Identity();expected.translation()=Eigen::Vector3d(.12,-.1,.03);
  expected.linear()=Eigen::AngleAxisd(.04,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for(int i=0;i<5000;++i) {Eigen::Vector3d point(coordinate(rng),coordinate(rng),coordinate(rng));
    request.source_points.push_back(point);request.target_points.push_back(expected*point);}
  const auto result=backend.register_clouds(request);const auto error=expected.inverse()*result.target_T_source;
  std::cout<<"VGICP error_m="<<error.translation().norm()<<" error_rad="<<Eigen::AngleAxisd(error.rotation()).angle()
    <<" rmse="<<result.residual<<" condition="<<result.condition_score<<'\n';
  require(result.method==RegistrationMethod::MAPPING_VGICP && result.converged,"Real VGICP did not converge");
  require(error.translation().norm()<.03 && Eigen::AngleAxisd(error.rotation()).angle()<.02,"Wrong VGICP transform direction/pose");
  require(result.inlier_ratio>.9 && result.residual<.05 && result.condition_score>1e-6,"VGICP quality metrics missing");
  request.initial_target_T_source=result.target_T_source;
  require(backend.register_clouds(request).converged,"VGICP stationary warm start failed");
  request.source_points[0].x()=std::numeric_limits<double>::quiet_NaN();
  require(!backend.register_clouds(request).converged,"VGICP accepted nonfinite cloud");
  require(!backend.register_clouds({}).converged,"VGICP accepted empty cloud");
  std::cout<<"PASS genuine voxel-map VGICP alignment and invalid-input rejection\n";
}
