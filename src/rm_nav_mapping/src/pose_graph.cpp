#include "rm_nav_mapping/pose_graph.hpp"
#include "rm_nav_registration/registration_types.hpp"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <stdexcept>

namespace rm_nav_mapping {
namespace {
void validate_edge(const rm_nav_mapping::GraphEdge & edge, std::size_t n, bool loop)
{
  if (edge.from >= n || edge.to >= n || edge.from == edge.to ||
      !rm_nav_registration::valid_rigid_transform(edge.from_T_to) ||
      !edge.sigmas.allFinite() || edge.sigmas.minCoeff()<=0 || edge.sigmas.maxCoeff()>1)
    throw std::invalid_argument("Invalid graph edge or noise tuning");
  if (loop) {
    if (!edge.loop || !edge.validated || !std::isfinite(edge.huber_k) || edge.huber_k <= 0 || edge.huber_k > 100)
      throw std::invalid_argument("Unvalidated loop edge refused");
  } else if (edge.loop || edge.validated) {
    throw std::invalid_argument("Adjacent graph edge marked as loop");
  }
}
}

GraphSolution optimize_pose_graph(const std::vector<Eigen::Isometry3d> & initial,
                                  const std::vector<GraphEdge> & adjacent)
{
  return optimize_pose_graph(initial, adjacent, {});
}

GraphSolution optimize_pose_graph(const std::vector<Eigen::Isometry3d> & initial,
                                  const std::vector<GraphEdge> & adjacent,
                                  const std::vector<GraphEdge> & validated_loops)
{
  if(initial.empty() || initial.size()>500 || adjacent.size()!=initial.size()-1)
    throw std::invalid_argument("Bounded, connected adjacent graph required (1..500 poses)");
  gtsam::NonlinearFactorGraph graph;gtsam::Values values;
  for(std::size_t i=0;i<initial.size();++i) {
    if(!rm_nav_registration::valid_rigid_transform(initial[i]))throw std::invalid_argument("Invalid initial graph pose");
    values.insert(i,gtsam::Pose3(initial[i].matrix()));
  }
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,gtsam::Pose3(initial[0].matrix()),gtsam::noiseModel::Isotropic::Sigma(6,.001)));
  for(std::size_t i=0;i<adjacent.size();++i) {
    const auto & edge=adjacent[i];
    if(edge.from!=i || edge.to!=i+1) throw std::invalid_argument("Invalid/disconnected adjacent graph edge");
    validate_edge(edge,initial.size(),false);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(edge.from,edge.to,gtsam::Pose3(edge.from_T_to.matrix()),
                                               gtsam::noiseModel::Diagonal::Sigmas(edge.sigmas)));
  }
  if (validated_loops.size() > initial.size()*2)
    throw std::invalid_argument("Loop edge budget exceeded");
  for (const auto & edge : validated_loops) {
    validate_edge(edge,initial.size(),true);
    auto base=gtsam::noiseModel::Diagonal::Sigmas(edge.sigmas);
    auto robust=gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(edge.huber_k),base);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(edge.from,edge.to,
      gtsam::Pose3(edge.from_T_to.matrix()),robust));
  }
  gtsam::LevenbergMarquardtParams settings;settings.maxIterations=50;
  settings.relativeErrorTol=1e-6;settings.absoluteErrorTol=1e-8;
  GraphSolution solution;solution.initial_error=graph.error(values);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph,values,settings);
  const auto result=optimizer.optimize();solution.final_error=graph.error(result);solution.iterations=optimizer.iterations();
  solution.loop_count=validated_loops.size();
  if(!std::isfinite(solution.initial_error) || !std::isfinite(solution.final_error) ||
     solution.final_error>solution.initial_error+1e-8*std::max(1.,solution.initial_error) ||
     (!validated_loops.empty() && solution.final_error>solution.initial_error))
    throw std::runtime_error("GTSAM objective is nonfinite or increased after graph optimization");
  if (validated_loops.empty() && solution.final_error>1e-4*initial.size())
    throw std::runtime_error("Adjacent graph residual exceeds convergence limit");
  for(std::size_t i=0;i<initial.size();++i) {
    Eigen::Isometry3d pose(result.at<gtsam::Pose3>(i).matrix());
    if(!rm_nav_registration::valid_rigid_transform(pose))throw std::runtime_error("Invalid optimized graph pose");
    solution.poses.push_back(pose);
  }
  return solution;
}
}
