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
  const auto angle=[](const Eigen::Matrix3d & r) {
    return std::acos(std::clamp((r.trace()-1.)*.5,-1.,1.));
  };
  const auto residual=[&](const GraphEdge & edge,const std::vector<Eigen::Isometry3d> & poses) {
    const Eigen::Isometry3d error=edge.from_T_to.inverse()*poses[edge.from].inverse()*poses[edge.to];
    const auto tangent=gtsam::Pose3::Logmap(gtsam::Pose3(error.matrix()));
    return GraphSolution::Residual{error.translation().norm(),angle(error.linear()),
      tangent.cwiseQuotient(edge.sigmas).norm()};
  };
  for(std::size_t i=0;i<initial.size();++i) {
    const Eigen::Isometry3d correction=initial[i].inverse()*solution.poses[i];
    solution.translation_corrections_m.push_back(correction.translation().norm());
    solution.rotation_corrections_rad.push_back(angle(correction.linear()));
    if(i) {
      const Eigen::Isometry3d before=initial[i-1].inverse()*initial[i];
      const Eigen::Isometry3d after=solution.poses[i-1].inverse()*solution.poses[i];
      const Eigen::Isometry3d change=before.inverse()*after;
      solution.max_local_translation_change_m=std::max(solution.max_local_translation_change_m,change.translation().norm());
      solution.max_local_rotation_change_rad=std::max(solution.max_local_rotation_change_rad,angle(change.linear()));
    }
  }
  for(const auto & edge:adjacent) {
    solution.adjacent_before.push_back(residual(edge,initial));
    const auto r=residual(edge,solution.poses);solution.adjacent_after.push_back(r);
    if(!validated_loops.empty()&&(r.translation_m>.20||r.rotation_rad>.15||r.whitened_norm>9))
      throw std::runtime_error("Optimized adjacent factor exceeds physical/normalized residual limits");
  }
  std::size_t worst=0;
  for(std::size_t i=0;i<validated_loops.size();++i) {
    const auto r=residual(validated_loops[i],solution.poses);solution.loop_after.push_back(r);
    if(r.translation_m>.25||r.rotation_rad>.15||r.whitened_norm>9)
      throw std::runtime_error("Optimized loop factor residual rejected despite objective decrease");
    if(r.whitened_norm>solution.loop_after[worst].whitened_norm)worst=i;
  }
  if(!validated_loops.empty()) {
    if(solution.max_local_translation_change_m>.5||solution.max_local_rotation_change_rad>.35||
       *std::max_element(solution.translation_corrections_m.begin(),solution.translation_corrections_m.end())>5||
       *std::max_element(solution.rotation_corrections_rad.begin(),solution.rotation_corrections_rad.end())>1)
      throw std::runtime_error("Optimized pose correction/local deformation budget exceeded");
    // One bounded counterfactual solve: remove the worst residual loop, retaining
    // the identical anchor/adjacent factors. Never use decreased robust cost alone.
    auto counterfactual=graph;
    counterfactual.erase(counterfactual.begin()+1+adjacent.size()+worst);
    const auto alternate=gtsam::LevenbergMarquardtOptimizer(counterfactual,result,settings).optimize();
    solution.removal_test_loop=static_cast<std::int64_t>(worst);
    for(std::size_t i=0;i<initial.size();++i) {
      const Eigen::Isometry3d pose(alternate.at<gtsam::Pose3>(i).matrix());
      const Eigen::Isometry3d change=solution.poses[i].inverse()*pose;
      solution.removal_max_translation_change_m=std::max(solution.removal_max_translation_change_m,change.translation().norm());
      solution.removal_max_rotation_change_rad=std::max(solution.removal_max_rotation_change_rad,angle(change.linear()));
    }
    if(solution.removal_max_translation_change_m>.5||solution.removal_max_rotation_change_rad>.35)
      throw std::runtime_error("Single-loop removal reveals excessive graph sensitivity");
  }
  return solution;
}
}
