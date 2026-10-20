#include "rm_nav_mapping/online_mapping.hpp"
#include "rm_nav_mapping/loop_validator.hpp"
#include "rm_nav_mapping/overlap.hpp"
#include "rm_nav_registration/kiss_gicp_backend.hpp"
#include <chrono>
#include <stdexcept>

namespace rm_nav_mapping {
namespace {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point start) {return std::chrono::duration<double>(Clock::now()-start).count();}
double geometry(const Cloud & points) {
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  for(const auto & p:points)mean+=p;
  mean/=static_cast<double>(points.size());
  Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
  for(const auto & p:points){const Eigen::Vector3d d=p-mean;cov+=d*d.transpose();}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if(solver.info()!=Eigen::Success || solver.eigenvalues().maxCoeff()<=0)return 0;
  return solver.eigenvalues().minCoeff()/solver.eigenvalues().maxCoeff();
}
}
OnlineMapping::OnlineMapping(OnlineMappingConfig config):config_(config),candidates_(config.candidates) {
  if(config.max_keyframes<1 || config.max_keyframes>500 || config.max_cloud_points<100 ||
     config.max_cloud_points>50000 || config.max_map_points<100 || config.max_map_points>1000000 ||
     !std::isfinite(config.map_voxel_m) || config.map_voxel_m<.05 || config.map_voxel_m>1 ||
     !std::isfinite(config.loop_budget_s) || config.loop_budget_s<=0 || config.loop_budget_s>30)
    throw std::invalid_argument("Invalid online mapping resource limits");
}
void OnlineMapping::insert(Voxels & map,const Cloud & points,const Eigen::Isometry3d & pose) const {
  for(const auto & point:points){
    const Eigen::Vector3d p=pose*point;
    map[std::make_tuple(long(std::floor(p.x()/config_.map_voxel_m)),long(std::floor(p.y()/config_.map_voxel_m)),
                       long(std::floor(p.z()/config_.map_voxel_m)))]=p;
    if(map.size()>config_.max_map_points)throw std::runtime_error("Online map voxel capacity exceeded; last snapshot retained, originals intact");
  }
}
void OnlineMapping::append(const Keyframe & frame,const Cloud & points) {
  const auto start=Clock::now();
  if(frames_.size()>=config_.max_keyframes)throw std::runtime_error("Online session keyframe limit reached; start a new segment");
  if(frame.id!=frames_.size() || frame.stamp_ns<=0 || !rm_nav_registration::valid_rigid_transform(frame.odom_T_chassis) ||
     points.size()<100 || points.size()>200000)throw std::invalid_argument("Invalid online frame/order/geometry");
  for(const auto & p:points)if(!p.allFinite() || p.cwiseAbs().maxCoeff()>10000)throw std::invalid_argument("Invalid online point");
  auto adjacent=adjacent_;
  Eigen::Isometry3d pose=frame.odom_T_chassis;
  if(!frames_.empty()){
    if(frame.stamp_ns<=frames_.back().stamp_ns)throw std::invalid_argument("Nonmonotonic online keyframe time");
    GraphEdge edge;edge.from=frame.id-1;edge.to=frame.id;
    edge.from_T_to=frames_.back().odom_T_chassis.inverse()*frame.odom_T_chassis;
    if(edge.from_T_to.translation().norm()>2 || rotation_angle(edge.from_T_to.linear())>1)
      throw std::invalid_argument("LIO adjacent jump exceeds online mapping limits");
    adjacent.push_back(edge);pose=poses_.back()*edge.from_T_to;
  }
  Cloud cloud;const auto stride=(points.size()+config_.max_cloud_points-1)/config_.max_cloud_points;
  for(std::size_t i=0;i<points.size();i+=stride)cloud.push_back(points[i]);
  auto next_poses=poses_;next_poses.push_back(pose);
  auto next_loops=loops_;bool corrected=false;
  last_graph_ms_=0;
  reason_="LIO odometry factors; no new validated loop";
  const auto stage=Clock::now();
  rm_nav_registration::KissGicpConfig kc;kc.num_threads=1;kc.max_input_points=config_.max_cloud_points;
  kc.refinement.num_threads=1;
  rm_nav_registration::KissGicpBackend matcher(kc);
  for(const auto id:candidates_.candidates(frame,frames_)){
    if(seconds(stage)>config_.loop_budget_s){reason_="loop stage soft budget exhausted";break;}
    ++candidates_tested_;
    const double pair_geometry=std::min(geometry(clouds_[id]),geometry(cloud));
    if(pair_geometry<LoopValidationConfig{}.min_geometry_ratio){
      ++loops_rejected_;reason_="degenerate scan geometry";continue;
    }
    rm_nav_registration::RegistrationRequest request;
    request.target_points=clouds_[id];request.source_points=cloud;
    request.initial_target_T_source=frames_[id].odom_T_chassis.inverse()*frame.odom_T_chassis;
    const auto forward=matcher.register_clouds(request);
    std::swap(request.target_points,request.source_points);
    const Eigen::Isometry3d reverse_seed=request.initial_target_T_source.inverse();
    request.initial_target_T_source=reverse_seed;
    const auto reverse=matcher.register_clouds(request);
    if(seconds(stage)>config_.loop_budget_s){++loops_rejected_;reason_="loop stage exceeded soft deadline; result discarded";break;}
    const auto verdict=validate_loop({forward,reverse,
      overlap_fraction(clouds_[id],cloud,forward.target_T_source),
      overlap_fraction(cloud,clouds_[id],reverse.target_T_source),
      std::max(forward.translation_delta,reverse.translation_delta),std::max(forward.rotation_delta,reverse.rotation_delta),
      pair_geometry});
    if(seconds(stage)>config_.loop_budget_s){++loops_rejected_;reason_="loop validation exceeded soft deadline";break;}
    if(!verdict.accepted){++loops_rejected_;reason_=verdict.reason;continue;}
    GraphEdge loop;loop.from=id;loop.to=frame.id;loop.loop=true;loop.validated=true;
    loop.from_T_to=forward.target_T_source;loop.sigmas<<.05,.05,.05,.10,.10,.10;
    auto trial=next_loops;trial.push_back(loop);
    std::vector<Eigen::Isometry3d> raw;
    for(const auto & f:frames_)raw.push_back(f.odom_T_chassis);
    raw.push_back(frame.odom_T_chassis);
    const auto graph_start=Clock::now();
    try {
      const auto solution=optimize_pose_graph(raw,adjacent,trial);
      next_poses=solution.poses;next_loops=std::move(trial);corrected=true;
      reason_="bidirectional loop and GTSAM post-audit accepted; historical map rebuilt";
    }catch(const std::exception & e){++loops_rejected_;reason_=std::string("graph rejected: ")+e.what();}
    last_graph_ms_=seconds(graph_start)*1000;
    if(corrected)break; // At most one accepted loop for the new frame.
  }
  // Build a complete replacement before committing poses/edges/clouds together.
  // Failure never exposes new poses over the old map, or mixed old/new points.
  Voxels map=corrected?Voxels{}:voxels_;
  if(corrected)for(std::size_t i=0;i<clouds_.size();++i)insert(map,clouds_[i],next_poses[i]);
  insert(map,cloud,next_poses.back());
  frames_.push_back(frame);clouds_.push_back(std::move(cloud));
  adjacent_=std::move(adjacent);loops_=std::move(next_loops);poses_=std::move(next_poses);voxels_=std::move(map);
  ++revision_;last_work_ms_=seconds(start)*1000;
}
Cloud OnlineMapping::map_points() const {
  Cloud out;out.reserve(voxels_.size());for(const auto & item:voxels_)out.push_back(item.second);return out;
}
}
