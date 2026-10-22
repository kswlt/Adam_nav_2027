#include "rm_nav_mapping/adjacent_submap.hpp"
#include <cassert>
#include <iostream>

int main() {
  using rm_nav_mapping::AdjacentSubmap;
  AdjacentSubmap history;
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
  b.translation() = Eigen::Vector3d(1., 0., 0.);
  b.linear() = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d wall(2., 3., 1.);
  history.push(0, 1000000000LL, a, {a.inverse()*wall});
  history.push(1, 2000000000LL, b, {b.inverse()*wall});
  auto target = history.target();
  assert(target.frame_ids == std::vector<std::uint64_t>({0, 1}));
  assert((target.points[0]-b.inverse()*wall).norm() < 1e-12);
  assert((target.points[1]-b.inverse()*wall).norm() < 1e-12);
  history.push(2, 3000000000LL, a, {wall});
  history.push(3, 4000000000LL, a, {wall});
  assert(history.target().frame_ids == std::vector<std::uint64_t>({1, 2, 3}));
  history.push(4, 10000000000LL, a, {wall});
  assert(history.target().frame_ids == std::vector<std::uint64_t>({4}));
  auto far = a;
  far.translation().x() = 10.;
  history.push(5, 11000000000LL, far, {wall});
  assert(history.target().frame_ids == std::vector<std::uint64_t>({5}));
  bool refused = false;
  try { history.push(7, 12000000000LL, a, {wall}); }
  catch (const std::invalid_argument &) { refused = true; }
  assert(refused);
  refused = false;
  try { history.push(6, 11000000000LL, a, {wall}); }
  catch (const std::invalid_argument &) { refused = true; }
  assert(refused);
  refused = false;
  try { history.push(6, 12000000000LL, a, std::vector<Eigen::Vector3d>(200001, wall)); }
  catch (const std::invalid_argument &) { refused = true; }
  assert(refused);
  AdjacentSubmap capacity;
  for(std::uint64_t i=0;i<4;++i)
    capacity.push(i,1000000000LL+static_cast<std::int64_t>(i)*1000000000LL,a,
                  std::vector<Eigen::Vector3d>(AdjacentSubmap::max_frame_points,wall));
  const auto full=capacity.target();
  assert(full.points.size()==AdjacentSubmap::max_target_points);
  assert(full.frame_ids==std::vector<std::uint64_t>({1,2,3}));
  std::cout << "PASS submap reference, rotation, age/distance/count bounds and provenance\n";
}
