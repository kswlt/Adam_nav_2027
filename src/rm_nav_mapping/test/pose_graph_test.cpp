#include "rm_nav_mapping/pose_graph.hpp"
#include <stdexcept>
#include <iostream>

void require(bool value,const char * message){if(!value)throw std::runtime_error(message);}
int main()
{
  using namespace rm_nav_mapping;
  std::vector<Eigen::Isometry3d> initial;
  std::vector<GraphEdge> edges;
  for(int i=0;i<5;++i) {
    auto pose=Eigen::Isometry3d::Identity();pose.translation()=Eigen::Vector3d(i*1.1,i*.1,.1*i);
    pose.linear()=Eigen::AngleAxisd(.05*i,Eigen::Vector3d::UnitZ()).toRotationMatrix();initial.push_back(pose);
    if(i) {GraphEdge edge;edge.from=i-1;edge.to=i;edge.from_T_to.translation().x()=1;edges.push_back(edge);}
  }
  const auto result=optimize_pose_graph(initial,edges);
  require(result.final_error<result.initial_error*.001,"Real GTSAM did not reduce inconsistent graph error");
  for(int i=0;i<5;++i)require((result.poses[i].translation()-Eigen::Vector3d(i,0,0)).norm()<1e-5,"Wrong optimized pose/direction");
  auto bad=edges;bad[1].to=4;
  try{optimize_pose_graph(initial,bad);require(false,"Disconnected graph accepted");}catch(const std::invalid_argument &){}
  bad=edges;bad[0].sigmas[0]=0;
  try{optimize_pose_graph(initial,bad);require(false,"Zero noise accepted");}catch(const std::invalid_argument &){}
  bad=edges;bad[0].from_T_to.linear()(0,0)=2;
  try{optimize_pose_graph(initial,bad);require(false,"Nonrigid factor accepted");}catch(const std::invalid_argument &){}
  const auto single=optimize_pose_graph({initial[0]},{});
  require(single.poses.size()==1 && single.final_error==0,"Single-frame anchor failed");
  GraphEdge loop;loop.from=0;loop.to=4;loop.loop=true;loop.validated=true;loop.huber_k=1.0;
  loop.from_T_to.translation().x()=4;
  const auto with_loop=optimize_pose_graph(initial,edges,{loop});
  require(with_loop.loop_count==1 && with_loop.final_error<=with_loop.initial_error,
          "Validated loop graph did not optimize");
  auto unvalidated=loop;unvalidated.validated=false;
  try{optimize_pose_graph(initial,edges,{unvalidated});require(false,"Unvalidated loop accepted");}
  catch(const std::invalid_argument &){}
  auto adjacent_marked=edges;adjacent_marked[0].loop=true;
  try{optimize_pose_graph(initial,adjacent_marked);require(false,"Marked adjacent edge accepted");}
  catch(const std::invalid_argument &){}
  std::cout<<"PASS actual GTSAM Pose3 adjacent graph, error reduction, direction and invalid factor refusal\n";
}
