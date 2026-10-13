#include "rm_nav_mapping/overlap.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>
double reference(const std::vector<Eigen::Vector3d> & target,const std::vector<Eigen::Vector3d> & source,
                 const Eigen::Isometry3d & transform) {
  if(target.empty()||source.empty())return 0;
  const auto ts=std::max<std::size_t>(1,target.size()/5000),ss=std::max<std::size_t>(1,source.size()/2000);
  std::size_t accepted=0,total=0;
  for(std::size_t i=0;i<source.size();i+=ss) {
    const Eigen::Vector3d p=transform*source[i];bool found=false;
    for(std::size_t j=0;j<target.size();j+=ts)if((p-target[j]).squaredNorm()<=.04){found=true;break;}
    accepted+=found;++total;
  }
  return static_cast<double>(accepted)/total;
}
int main() {
  std::mt19937 random(42);std::uniform_real_distribution<double> d(-10,10);
  std::vector<Eigen::Vector3d> a,b;
  for(int i=0;i<10000;++i){a.emplace_back(d(random),d(random),d(random));b.emplace_back(d(random),d(random),d(random));}
  auto transform=Eigen::Isometry3d::Identity();transform.translation()<<.1,-.1,.05;
  const auto start=std::chrono::steady_clock::now();const double expected=reference(a,b,transform);
  const auto middle=std::chrono::steady_clock::now();const double actual=rm_nav_mapping::overlap_fraction(a,b,transform);
  const auto end=std::chrono::steady_clock::now();
  if(actual!=expected || rm_nav_mapping::overlap_fraction({},b,transform)!=0)throw std::runtime_error("Overlap metric changed");
  // Exact boundary, duplicate target and deterministic repetition.
  a={Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};b={Eigen::Vector3d(.199999,0,0),Eigen::Vector3d(.200001,0,0)};
  if(rm_nav_mapping::overlap_fraction(a,b,Eigen::Isometry3d::Identity())!=.5)throw std::runtime_error("Radius boundary changed");
  std::cout<<"PASS exact overlap="<<actual<<" brute_ms="<<std::chrono::duration<double,std::milli>(middle-start).count()
    <<" kdtree_ms="<<std::chrono::duration<double,std::milli>(end-middle).count()<<'\n';
}
