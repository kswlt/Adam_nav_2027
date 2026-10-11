#include "rm_nav_planning/minimum_jerk.hpp"

namespace rm_nav_planning {
static double wrap_yaw(double angle)
{
  constexpr double pi=3.14159265358979323846;
  while(angle>pi) angle-=2*pi;
  while(angle<-pi) angle+=2*pi;
  return angle;
}
bool MinimumJerkGenerator::generate(const Eigen::Vector3d & start, const Eigen::Vector3d & goal, double duration_s,
                                    std::uint64_t id, const std::string & map_version, const std::string & environment,
                                    std::int64_t now_ns, TimedTrajectory & output) const {
  if (!start.allFinite() || !goal.allFinite() || map_version.empty() || environment.empty() ||
      !std::isfinite(limits_.max_velocity) || limits_.max_velocity<=0 || !std::isfinite(limits_.max_acceleration) || limits_.max_acceleration<=0 ||
      !std::isfinite(limits_.max_jerk) || limits_.max_jerk<=0 || !std::isfinite(limits_.sample_period) || limits_.sample_period<=0) return false;
  Eigen::Vector3d delta=goal-start; delta[2]=wrap_yaw(delta[2]);
  if (!std::isfinite(duration_s) || duration_s<=0) duration_s=std::max(0.5, delta.head<2>().norm()/limits_.max_velocity*1.5);
  if (duration_s<=0 || duration_s>3600) return false;
  const std::size_t count=static_cast<std::size_t>(std::ceil(duration_s/limits_.sample_period))+1;
  output={}; output.trajectory_id=id; output.map_bundle_version=map_version; output.environment_version=environment;
  output.generated_at_ns=now_ns; output.valid_until_ns=now_ns+static_cast<std::int64_t>((duration_s+0.5)*1e9); output.points.reserve(count);
  for(std::size_t i=0;i<count;++i){
    const double t=std::min(duration_s, i*limits_.sample_period), s=t/duration_s, s2=s*s, s3=s2*s, s4=s3*s2, s5=s4*s;
    const double h=10*s3-15*s4+6*s5, dh=(30*s2-60*s3+30*s4)/duration_s;
    const double ddh=(60*s-180*s2+120*s3)/(duration_s*duration_s);
    const double dddh=(60-360*s+360*s2)/(duration_s*duration_s*duration_s);
    TrajectoryPoint point; point.t_from_start_s=t; point.pose=start+delta*h; point.pose[2]=wrap_yaw(start[2]+delta[2]*h);
    point.velocity=delta*dh; point.clearance_m=0.0;
    if (point.velocity.head<2>().norm()>limits_.max_velocity+1e-6 || std::abs(point.velocity[2])>limits_.max_velocity+1e-6 ||
        (delta*ddh).norm()>limits_.max_acceleration+1e-6 || (delta*dddh).norm()>limits_.max_jerk+1e-6) return false;
    output.points.push_back(point);
  }
  return output.valid_at(now_ns,map_version,environment);
}
}  // namespace rm_nav_planning
