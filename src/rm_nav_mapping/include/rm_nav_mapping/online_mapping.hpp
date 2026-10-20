#pragma once
#include "rm_nav_mapping/pose_graph.hpp"
#include "rm_nav_mapping/loop_candidate_manager.hpp"
#include <functional>
#include <map>
#include <tuple>
#include <string>

namespace rm_nav_mapping {
using Cloud = std::vector<Eigen::Vector3d>;
struct OnlineMappingConfig {
  std::size_t max_keyframes{500}, max_cloud_points{20000}, max_map_points{500000};
  double map_voxel_m{.15}, loop_budget_s{3.};
  LoopCandidateConfig candidates{10'000'000'000, 10, 3., 2};
};
// One worker owns this engine. LIO odometry provides adjacent factors; only
// bidirectional KISS/GICP + graph post-audit can change historical poses.
class OnlineMapping {
public:
  explicit OnlineMapping(OnlineMappingConfig config = {});
  void append(const Keyframe & frame, const Cloud & local_points);
  const std::vector<Keyframe> & frames() const { return frames_; }
  const std::vector<Eigen::Isometry3d> & poses() const { return poses_; }
  const std::vector<GraphEdge> & loops() const { return loops_; }
  Cloud map_points() const;
  std::size_t revision() const { return revision_; }
  std::size_t candidates_tested() const { return candidates_tested_; }
  std::size_t loops_rejected() const { return loops_rejected_; }
  const std::string & reason() const { return reason_; }
  double last_work_ms() const { return last_work_ms_; }
  double last_graph_ms() const { return last_graph_ms_; }
private:
  using Voxels = std::map<std::tuple<long,long,long>, Eigen::Vector3d>;
  void insert(Voxels &, const Cloud &, const Eigen::Isometry3d &) const;
  OnlineMappingConfig config_;
  LoopCandidateManager candidates_;
  std::vector<Keyframe> frames_;
  std::vector<Cloud> clouds_;
  std::vector<Eigen::Isometry3d> poses_;
  std::vector<GraphEdge> adjacent_, loops_;
  Voxels voxels_;
  std::size_t revision_{0}, candidates_tested_{0}, loops_rejected_{0};
  std::string reason_{"waiting for committed keyframes"};
  double last_work_ms_{0}, last_graph_ms_{0};
};
}
