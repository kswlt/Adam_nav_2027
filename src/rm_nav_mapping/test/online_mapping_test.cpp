#include "rm_nav_mapping/online_mapping.hpp"
#include <random>
#include <iostream>
#include <stdexcept>
using namespace rm_nav_mapping;
void require(bool ok,const char * why){if(!ok)throw std::runtime_error(why);}
int main(){
  Cloud world;std::mt19937 rng(20271009);std::uniform_real_distribution<double> u(-3,3);
  for(int i=0;i<12000;++i)world.emplace_back(u(rng),u(rng),u(rng));
  OnlineMappingConfig cfg;cfg.loop_budget_s=30; // Deterministic solver test, not a real-time deadline assertion.
  OnlineMapping engine(cfg);
  const std::vector<Eigen::Vector2d> route{{0,0},{.7,0},{1.4,0},{2.1,0},{2.8,.2},{2.8,.9},
    {2.1,1.2},{1.4,1.1},{.7,.7},{.2,.3},{0,0},{.15,.05}};
  Eigen::Isometry3d actual=Eigen::Isometry3d::Identity(),raw=actual;
  for(std::size_t i=0;i<route.size();++i){
    actual.translation()=Eigen::Vector3d(route[i].x(),route[i].y(),0);
    actual.linear()=Eigen::AngleAxisd(.02*i,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    raw=actual;raw.translation()+=Eigen::Vector3d(.025*i,.012*i,0);
    raw.linear()=Eigen::AngleAxisd(.025*i,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Cloud local;for(const auto & p:world)local.push_back(actual.inverse()*p);
    engine.append({i,static_cast<std::int64_t>(100+i)*1000000000,raw},local);
    require(engine.frames().size()==i+1 && engine.revision()==i+1,"Nonatomic online frame/revision update");
  }
  std::cout<<"loops="<<engine.loops().size()<<" candidates="<<engine.candidates_tested()<<" rejected="<<engine.loops_rejected()
    <<" reason="<<engine.reason()<<" work_ms="<<engine.last_work_ms()<<'\n';
  require(!engine.loops().empty(),"Real bidirectional KISS/GICP loop not accepted online");
  require((engine.poses().back().translation()-actual.translation()).norm()<
    (raw.translation()-actual.translation()).norm(),"Online GTSAM failed to reduce controlled drift");
  require((engine.poses()[5].translation()-engine.frames()[5].odom_T_chassis.translation()).norm()>.001,
    "Historical poses were not corrected");
  require(!engine.map_points().empty(),"Optimized map absent");
  const auto rev=engine.revision();const auto size=engine.map_points().size();
  bool refused=false;
  try{engine.append({99,1000000000000,raw},world);}catch(const std::invalid_argument &){refused=true;}
  require(refused && engine.revision()==rev && engine.map_points().size()==size,"Invalid input changed accepted snapshot");
  cfg.max_keyframes=1;OnlineMapping limited(cfg);limited.append({0,1,Eigen::Isometry3d::Identity()},world);
  refused=false;try{limited.append({1,2,Eigen::Isometry3d::Identity()},world);}catch(const std::runtime_error &){refused=true;}
  require(refused && limited.frames().size()==1,"Session capacity silently truncated/overflowed");
  cfg.max_map_points=100;OnlineMapping tiny(cfg);refused=false;
  try{tiny.append({0,1,Eigen::Isometry3d::Identity()},world);}catch(const std::runtime_error &){refused=true;}
  require(refused && tiny.frames().empty() && tiny.revision()==0,"Map resource failure committed partial snapshot");
  cfg=OnlineMappingConfig{};OnlineMapping planar(cfg);Cloud plane;
  for(int x=0;x<40;++x)for(int y=0;y<40;++y)plane.emplace_back(.1*x,.1*y,0);
  for(int i=0;i<=10;++i)planar.append({static_cast<std::uint64_t>(i),static_cast<std::int64_t>(100+i)*1000000000,
    Eigen::Isometry3d::Identity()},plane);
  require(planar.loops().empty() && planar.loops_rejected()>0 && planar.reason()=="degenerate scan geometry",
    "Single-plane revisit was accepted as a loop");
  std::cout<<"PASS online real KISS/GICP + audited GTSAM, historical correction, rebuilt map and capacity rollback\n";
}
