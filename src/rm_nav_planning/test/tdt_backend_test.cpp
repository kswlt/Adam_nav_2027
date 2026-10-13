#include "rm_nav_planning/tdt_backend.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace rm_nav_planning;
  TdtRequest request;
  request.grid.width=request.grid.height=100; request.grid.resolution=.05;
  request.grid.cells.assign(10000,0);
  request.footprint={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
  request.path={{1,1,3.1},{1.5,1.3,3.1},{2,1,-3.1}};
  request.initial_velocity={.1,0,.05};
  request.trajectory_id=1; request.map_version="m"; request.environment_version="e";
  TdtOptions options; options.solve_budget_s=3;
  TdtMinimumJerkBackend backend(options);
  TdtReport report; TimedTrajectory output;
  const bool success=backend.generate(request,output,report);
  std::cout<<report.reason<<" solves="<<report.solves<<" time="<<report.wall_time_s<<std::endl;
  assert(success);
  assert(output.valid_at(0,"m","e"));
  assert((output.points.front().velocity-request.initial_velocity).norm()<1e-3);
  assert(output.points.back().velocity.norm()<1e-3);
  assert(std::abs(std::remainder(output.points.back().pose.z()+3.1,6.283185307179586))<1e-3);
  assert(report.velocity_bound<=options.limits.max_velocity*1.000001);
  assert(report.acceleration_bound<=options.limits.max_acceleration*1.000001);
  assert(report.jerk_bound<=options.limits.max_jerk*1.000001);
  const auto saved=output.trajectory_id;
  request.grid.cells[20*100+20]=100;
  assert(!backend.generate(request,output,report) && output.trajectory_id==saved);
  request.grid.cells[20*100+20]=0;
  request.path={{2,2,0},{2,2,1}}; request.initial_velocity.setZero(); request.trajectory_id++;
  assert(backend.generate(request,output,report)); // yaw-only, no artificial XY motion
  for (const auto & point:output.points) assert((point.pose.head<2>()-Eigen::Vector2d(2,2)).norm()<1e-3);
  request.grid.origin={-2,1,.3};
  request.path={{0,3,0},{.3,3,.2}}; request.trajectory_id++;
  assert(backend.generate(request,output,report)); // rotated/nonzero map origin
  request.path.front().x()=1e300;
  assert(!backend.generate(request,output,report));
  request.path={{1,1,0},{2,1,0}}; request.initial_velocity={.1,0,0};
  request.grid.origin.setZero();
  TdtOptions slow=options; slow.limits.max_jerk=.02;
  const bool retimed=TdtMinimumJerkBackend(slow).generate(request,output,report);
  std::cout<<"retiming: "<<report.reason<<" solves="<<report.solves<<std::endl;
  assert(retimed);
  assert(report.solves>2 && report.jerk_bound<=.020001);
  assert((output.points.front().velocity-request.initial_velocity).norm()<1e-3);
  slow.solve_budget_s=1e-9;
  assert(!TdtMinimumJerkBackend(slow).generate(request,output,report));
  return 0;
}
