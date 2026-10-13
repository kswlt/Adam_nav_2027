#include "rm_nav_planning/tdt_backend.hpp"
#include "minimumSnap.hpp"
#include <chrono>
#include <limits>
#include <numeric>

namespace rm_nav_planning {
namespace {
constexpr double kTwoPi=6.28318530717958647692;
double wrap(double value) { return std::remainder(value,kTwoPi); }
double choose(int n,int k) {
  double result=1;
  for (int j=1;j<=k;++j) result=result*(n-j+1)/j;
  return result;
}
// Derivative polynomial in normalized u, as Bernstein control vectors. The curve
// lies in their convex hull, so the largest control-vector norm is a continuous
// upper bound, including between output samples (degree is only five).
double derivative_bound(const Eigen::MatrixXd & x,const Eigen::MatrixXd & y,
    const std::vector<double> & times,int derivative) {
  double bound=0;
  const int degree=x.cols()-1, n=degree-derivative;
  for (int row=0;row<x.rows();++row) {
    for (int i=0;i<=n;++i) {
      Eigen::Vector2d control=Eigen::Vector2d::Zero();
      for (int k=0;k<=i;++k) {
        const int power=k+derivative;
        double factor=choose(i,k)/choose(n,k)/std::pow(times[row],derivative);
        for (int j=0;j<derivative;++j) factor*=power-j;
        control+=factor*Eigen::Vector2d(x(row,degree-power),y(row,degree-power));
      }
      const double norm=control.stableNorm();
      if (!std::isfinite(norm)) return std::numeric_limits<double>::infinity();
      bound=std::max(bound,norm);
    }
  }
  return bound;
}
Eigen::Vector2d evaluate(const MinimumSnap::SolveOutput & result,int segment,double u,int derivative) {
  Eigen::Vector2d value=Eigen::Vector2d::Zero();
  const int degree=result.coefficients_x.cols()-1;
  for (int power=degree;power>=derivative;--power) {
    double factor=1;
    for (int j=0;j<derivative;++j) factor*=power-j;
    const Eigen::Vector2d coefficient(result.coefficients_x(segment,degree-power),
      result.coefficients_y(segment,degree-power));
    value=value*u+coefficient*factor;
  }
  return value/std::pow(result.segment_times[segment],derivative);
}
bool valid_result(const MinimumSnap::SolveOutput & r) {
  return r.success && r.normalized_time && !r.segment_times.empty() &&
    r.coefficients_x.rows()==static_cast<int>(r.segment_times.size()) &&
    r.coefficients_y.rows()==r.coefficients_x.rows() && r.coefficients_x.cols()==6 &&
    r.coefficients_y.cols()==6 && r.coefficients_x.allFinite() && r.coefficients_y.allFinite() &&
    std::all_of(r.segment_times.begin(),r.segment_times.end(),[](double t){return std::isfinite(t)&&t>0;});
}
}

bool TdtMinimumJerkBackend::generate(const TdtRequest & r,TimedTrajectory & output,TdtReport & report) const {
  const auto begin=std::chrono::steady_clock::now();
  report={};
  const auto finish=[&](const char * reason) {
    report.reason=reason;
    report.wall_time_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    return false;
  };
  const auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
  const auto & limits=options_.limits;
  for (double value : {limits.max_velocity,limits.max_acceleration,limits.max_jerk,limits.sample_period,
      limits.max_yaw_rate,limits.max_yaw_acceleration,limits.max_yaw_jerk,options_.solve_budget_s,options_.corridor_range_m})
    if (!std::isfinite(value)||value<=0) return finish("invalid limits");
  if (options_.collision_iterations<1||options_.collision_iterations>8||options_.retime_attempts<1||
      options_.retime_attempts>8||options_.solve_budget_s>5||r.path.size()<2||r.path.size()>128||
      r.map_version.empty()||r.environment_version.empty()||r.trajectory_id==0||r.execution_epoch_ns<0||
      !r.initial_velocity.allFinite()||r.initial_velocity.head<2>().stableNorm()>limits.max_velocity||
      std::abs(r.initial_velocity.z())>limits.max_yaw_rate) return finish("invalid request");
  const auto & grid=r.grid;
  if (!grid.origin.allFinite()||!std::isfinite(grid.resolution)||grid.resolution<=0||
      grid.width<2||grid.height<2||grid.width>4000000/grid.height||
      grid.cells.size()!=grid.width*grid.height) return finish("invalid grid");
  for (auto value : grid.cells) if (value < -1 || value > 100) return finish("invalid occupancy value");
  const double cg=std::cos(grid.origin.z()),sg=std::sin(grid.origin.z());
  const auto to_local=[&](const Eigen::Vector2d & world) {
    const Eigen::Vector2d d=world-grid.origin.head<2>();
    return Eigen::Vector2d(cg*d.x()+sg*d.y(),-sg*d.x()+cg*d.y());
  };
  const auto to_world=[&](const Eigen::Vector2d & local) {
    return Eigen::Vector2d(cg*local.x()-sg*local.y(),sg*local.x()+cg*local.y());
  };
  std::vector<Eigen::Vector2f> path;
  for (const auto & p : r.path) {
    if (!p.allFinite()) return finish("nonfinite path");
    const auto local=to_local(p.head<2>());
    if (!local.allFinite()||local.x()<0||local.y()<0||local.x()>=grid.width*grid.resolution||
        local.y()>=grid.height*grid.resolution) return finish("path outside grid");
    const auto x=static_cast<std::size_t>(local.x()/grid.resolution),y=static_cast<std::size_t>(local.y()/grid.resolution);
    if (grid.cells[y*grid.width+x]<0||grid.cells[y*grid.width+x]>=65) return finish("waypoint blocked");
    if (path.empty()||(local-path.back().cast<double>()).stableNorm()>=0.01) path.push_back(local.cast<float>());
  }
  if (path.size()==1) path.push_back(path.front());
  // Preserve the requested endpoint after removing very close interior XY points.
  path.back()=to_local(r.path.back().head<2>()).cast<float>();
  MinimumSnap::Map map(grid.height,grid.width);
  for (std::size_t y=0;y<grid.height;++y) for (std::size_t x=0;x<grid.width;++x) {
    const auto value=grid.cells[y*grid.width+x]; map(y,x)=value>=0&&value<65?255:0;
  }
  MinimumSnap xy,yaw;
  for (auto * solver : {&xy,&yaw}) {
    solver->setOrder(6); // six coefficients == degree five
    solver->setMaxDx(3); solver->setStrictCollision(true); solver->setTL(options_.solve_budget_s);
  }
  xy.setMap(map,grid.resolution,0,0);
  MinimumSnap::SolveInput input;
  input.setPath(path);
  const Eigen::Vector2d initial_xy=r.initial_velocity.head<2>();
  input.setInitVel(Eigen::Vector2f(cg*initial_xy.x()+sg*initial_xy.y(),-sg*initial_xy.x()+cg*initial_xy.y()));
  input.setBackend(MinimumSnap::Backend::OSQPCorridor); input.setNormTime(true);
  input.setCollisionCheckIter(options_.collision_iterations); input.setMaxCorridorRange(options_.corridor_range_m);
  auto times=MinimumSnap::trapezoidalTimeAllocation(path,limits.max_velocity,limits.max_acceleration);
  if (times.size()+1!=path.size()) return finish("time allocation failed");
  for (auto & time : times) time=std::max(0.2,time);
  const double start_yaw=wrap(r.path.front().z());
  const double yaw_delta=wrap(wrap(r.path.back().z())-start_yaw);
  MinimumSnap::SolveInput yaw_input;
  yaw_input.setPath({Eigen::Vector2f(start_yaw,0),Eigen::Vector2f(start_yaw+yaw_delta,0)});
  yaw_input.setInitVel(Eigen::Vector2f(r.initial_velocity.z(),0));
  yaw_input.setBackend(MinimumSnap::Backend::OSQPPath); yaw_input.setNormTime(true); yaw_input.setCollisionCheckIter(1);
  for (int attempt=0;attempt<options_.retime_attempts;++attempt) {
    if (elapsed()>options_.solve_budget_s) return finish("solve budget exhausted");
    input.setTimeAllocated(times);
    auto position=xy.solve(input); report.solves++; report.collision_iterations+=position.iter;
    if (!valid_result(position)) return finish("TDT XY QP/collision failed");
    const double duration=std::accumulate(position.segment_times.begin(),position.segment_times.end(),0.0);
    if (!std::isfinite(duration)||duration>120) return finish("duration limit");
    std::vector<double> yaw_times{duration}; yaw_input.setTimeAllocated(yaw_times);
    auto heading=yaw.solve(yaw_input); report.solves++;
    if (!valid_result(heading)) return finish("TDT yaw QP failed");
    report.velocity_bound=derivative_bound(position.coefficients_x,position.coefficients_y,position.segment_times,1);
    report.acceleration_bound=derivative_bound(position.coefficients_x,position.coefficients_y,position.segment_times,2);
    report.jerk_bound=derivative_bound(position.coefficients_x,position.coefficients_y,position.segment_times,3);
    const double v_yaw=derivative_bound(heading.coefficients_x,heading.coefficients_y,heading.segment_times,1);
    const double a_yaw=derivative_bound(heading.coefficients_x,heading.coefficients_y,heading.segment_times,2);
    const double j_yaw=derivative_bound(heading.coefficients_x,heading.coefficients_y,heading.segment_times,3);
    const double scale=std::max({report.velocity_bound/limits.max_velocity,
      std::sqrt(report.acceleration_bound/limits.max_acceleration),std::cbrt(report.jerk_bound/limits.max_jerk),
      v_yaw/limits.max_yaw_rate,std::sqrt(a_yaw/limits.max_yaw_acceleration),std::cbrt(j_yaw/limits.max_yaw_jerk)});
    if (!std::isfinite(scale)) return finish("nonfinite derivative bounds");
    if (scale>1.000001) {
      for (auto & t : times) t*=std::max(1.1,scale*1.02);
      continue; // re-solve, preserving the actual initial velocity constraint
    }
    if (elapsed()>options_.solve_budget_s) return finish("solve budget exhausted");
    const double intervals=std::ceil(duration/limits.sample_period);
    if (!std::isfinite(intervals)||intervals>199999) return finish("sampling budget");
    const auto count=static_cast<std::size_t>(std::max(1.0,intervals))+1;
    const long double lifetime=std::ceil((static_cast<long double>(duration)+0.5L)*1e9L);
    if (lifetime>static_cast<long double>(std::numeric_limits<std::int64_t>::max())-r.execution_epoch_ns)
      return finish("timestamp overflow");
    TimedTrajectory candidate;
    candidate.trajectory_id=r.trajectory_id; candidate.map_bundle_version=r.map_version;
    candidate.environment_version=r.environment_version; candidate.generated_at_ns=r.execution_epoch_ns;
    candidate.valid_until_ns=r.execution_epoch_ns+static_cast<std::int64_t>(lifetime);
    std::size_t segment=0; double segment_start=0;
    for (std::size_t i=0;i<count;++i) {
      const double t=duration*i/(count-1);
      while (segment+1<position.segment_times.size()&&t>segment_start+position.segment_times[segment])
        segment_start+=position.segment_times[segment++];
      const double u=std::clamp((t-segment_start)/position.segment_times[segment],0.0,1.0);
      const Eigen::Vector2d p=to_world(evaluate(position,segment,u,0))+grid.origin.head<2>();
      const Eigen::Vector2d v=to_world(evaluate(position,segment,u,1));
      candidate.points.push_back({t,{p.x(),p.y(),wrap(evaluate(heading,0,t/duration,0).x())},
        {v.x(),v.y(),evaluate(heading,0,t/duration,1).x()},0});
    }
    if (!candidate.valid_at(r.execution_epoch_ns,r.map_version,r.environment_version)) return finish("invalid output");
    if ((candidate.points.front().pose.head<2>()-r.path.front().head<2>()).norm()>1e-3||
        (candidate.points.back().pose.head<2>()-r.path.back().head<2>()).norm()>1e-3||
        (candidate.points.front().velocity-r.initial_velocity).norm()>1e-3||
        candidate.points.back().velocity.norm()>1e-3) return finish("endpoint constraint residual");
    if (!PolygonFootprintValidator::validate(candidate,r.footprint,grid)) return finish("polygon collision");
    if (elapsed()>options_.solve_budget_s) return finish("validation budget exhausted");
    output=std::move(candidate); report.reason="accepted"; report.wall_time_s=elapsed(); return true;
  }
  return finish("derivative constraints exceed retiming budget");
}
}  // namespace rm_nav_planning
