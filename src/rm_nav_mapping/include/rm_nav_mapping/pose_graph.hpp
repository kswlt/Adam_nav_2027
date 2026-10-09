#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <cstdint>

namespace rm_nav_mapping {
struct GraphEdge {
  std::uint64_t from{0},to{0};
  Eigen::Isometry3d from_T_to{Eigen::Isometry3d::Identity()};
  // GTSAM Pose3 tangent order: rotation xyz, translation xyz. Explicit conservative tuning,
  // not an invented covariance obtained by treating the registration Hessian as a probability.
  Eigen::Matrix<double,6,1> sigmas{(Eigen::Matrix<double,6,1>()<<.03,.03,.03,.05,.05,.05).finished()};
  // Loop edges are admitted only from the independent bidirectional validator.
  bool loop{false};
  bool validated{false};
  double huber_k{1.0};
};
struct GraphSolution {
  std::vector<Eigen::Isometry3d> poses;
  double initial_error{0},final_error{0};
  int iterations{0};
  std::size_t loop_count{0};
};
GraphSolution optimize_pose_graph(const std::vector<Eigen::Isometry3d> & initial,
                                  const std::vector<GraphEdge> & adjacent);
GraphSolution optimize_pose_graph(const std::vector<Eigen::Isometry3d> & initial,
                                  const std::vector<GraphEdge> & adjacent,
                                  const std::vector<GraphEdge> & validated_loops);
}
