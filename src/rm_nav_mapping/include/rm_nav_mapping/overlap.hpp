#pragma once
#include <Eigen/Geometry>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <vector>

namespace rm_nav_mapping {
// Exact legacy sampled-set metric: fraction of sampled source points within
// 0.2 m of a sampled target. No voxel centroids or covariance preprocessing.
inline double overlap_fraction(const std::vector<Eigen::Vector3d> & target,
    const std::vector<Eigen::Vector3d> & source,const Eigen::Isometry3d & transform) {
  if(target.empty()||source.empty())return 0;
  const auto ts=std::max<std::size_t>(1,target.size()/5000);
  const auto ss=std::max<std::size_t>(1,source.size()/2000);
  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve((target.size()+ts-1)/ts);
  for(std::size_t i=0;i<target.size();i+=ts)sampled.push_back(target[i]);
  const auto cloud=std::make_shared<small_gicp::PointCloud>(sampled);
  const small_gicp::KdTree<small_gicp::PointCloud> tree(cloud);
  std::size_t accepted=0,total=0;
  for(std::size_t i=0;i<source.size();i+=ss) {
    Eigen::Vector4d point;point.head<3>()=transform*source[i];point.w()=1;
    std::size_t index=0;double distance=0;
    if(tree.knn_search(point,1,&index,&distance)&&distance<=.04)++accepted;
    ++total;
  }
  return static_cast<double>(accepted)/total;
}
}
