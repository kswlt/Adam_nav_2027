#include "rm_nav_mapping/pose_graph.hpp"
#include "rm_nav_mapping/archive_io.hpp"
#include "rm_nav_mapping/loop_candidate_manager.hpp"
#include "rm_nav_mapping/loop_validator.hpp"
#include "rm_nav_registration/small_gicp_backend.hpp"
#include "rm_nav_registration/kiss_gicp_backend.hpp"
#include <small_gicp/registration/registration_helper.hpp>
#include "rm_nav_sensors/point_cloud.hpp"
#include <rm_nav_interfaces/msg/observation_frame.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <regex>
#include <map>
#include <set>
#include <tuple>
#include <sys/syscall.h>
#include <linux/fs.h>
#include <openssl/sha.h>
#include <array>
#include <sys/file.h>

namespace fs=std::filesystem;
using rm_nav_interfaces::msg::ObservationFrame;
struct ArchivedFrame {
  fs::path path;
  std::uint64_t id,sequence;
  std::int64_t stamp;
  std::string calibration,source,sensor,body;
  std::array<unsigned char,SHA256_DIGEST_LENGTH> digest{};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};
void require(bool value,const std::string & message){if(!value)throw std::runtime_error(message);}
struct SessionReadLock {
  int fd{-1};
  explicit SessionReadLock(const fs::path & session) {
    const auto path=session/"recording.lock";
    if(!fs::exists(path))return;  // Legacy schema-1 archives require the operator to stop recording.
    require(!fs::is_symlink(path),"Recording lock symlink rejected");
    fd=::open(path.c_str(),O_RDONLY|O_CLOEXEC);
    require(fd>=0,"Cannot open recording lock");
    if(::flock(fd,LOCK_SH|LOCK_NB)!=0){::close(fd);fd=-1;throw std::runtime_error("Recording session is active; stop recorder before optimization");}
  }
  ~SessionReadLock(){if(fd>=0)::close(fd);}
};
std::vector<Eigen::Vector3d> load(const fs::path & directory,ArchivedFrame & frame,bool local)
{
  require(!fs::is_symlink(directory) && !fs::is_symlink(directory/"metadata.json") && !fs::is_symlink(directory/"observation.cdr"),
          "Archive symlinks are not accepted");
  require(fs::file_size(directory/"metadata.json")<=65536,"Oversized metadata");
  boost::property_tree::ptree metadata;boost::property_tree::read_json((directory/"metadata.json").string(),metadata);
  require(metadata.get<int>("schema")==1 && metadata.get<std::string>("cloud_frame")=="odom" &&
          metadata.get<std::string>("observation_file")=="observation.cdr","Unsupported archive schema/frame/file");
  frame.path=directory;frame.id=metadata.get<std::uint64_t>("id");frame.stamp=metadata.get<std::int64_t>("stamp_ns");
  frame.calibration=metadata.get<std::string>("calibration_id");frame.source=metadata.get<std::string>("source_id");
  frame.sensor=metadata.get<std::string>("sensor_frame");frame.body=metadata.get<std::string>("body_frame");
  require(frame.stamp>0 && !frame.calibration.empty() && !frame.source.empty() && !frame.sensor.empty() &&
          !frame.body.empty() && frame.body!="odom" && directory.filename()=="keyframe_"+std::to_string(frame.id),
          "Archive identity or source time invalid");
  int element=0;
  for(const auto & item:metadata.get_child("recorded_odom_T_body")) {
    require(element<16,"Too many matrix entries");frame.pose.matrix()(element/4,element%4)=item.second.get_value<double>();++element;
  }
  require(element==16 && rm_nav_registration::valid_rigid_transform(frame.pose) && frame.pose.translation().cwiseAbs().maxCoeff()<=10000,
          "Invalid archived chassis reference pose");
  const auto size=fs::file_size(directory/"observation.cdr");
  require(size>0 && size<=32*1024*1024 && size==metadata.get<std::uint64_t>("observation_bytes"),"Archive size/truncation mismatch");
  rclcpp::SerializedMessage serialized(size);auto & buffer=serialized.get_rcl_serialized_message();
  std::ifstream input(directory/"observation.cdr",std::ios::binary);
  input.read(reinterpret_cast<char *>(buffer.buffer),size);require(input.good(),"Archive read failed");buffer.buffer_length=size;
  SHA256(buffer.buffer,buffer.buffer_length,frame.digest.data());
  ObservationFrame observation;rclcpp::Serialization<ObservationFrame> serializer;serializer.deserialize_message(&serialized,&observation);
  const auto stamp=static_cast<std::int64_t>(observation.header.stamp.sec)*1000000000+observation.header.stamp.nanosec;
  require(stamp==frame.stamp && observation.header.frame_id=="odom" && observation.point_cloud.header==observation.header &&
          observation.calibration_id==frame.calibration && observation.source_id==frame.source && observation.sensor_frame==frame.sensor &&
          observation.deskewed && observation.healthy && observation.roles==17 && observation.sequence>0,"Archived observation provenance mismatch");
  frame.sequence=observation.sequence;
  const auto & p=observation.sensor_pose.position;const auto & q=observation.sensor_pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) && std::abs(norm-1)<1e-5,
          "Invalid archived source origin");
  auto points=rm_nav_sensors::read_xyz(observation.point_cloud,200000);
  require(points.size()>=50 && (observation.per_point_time.empty() || observation.per_point_time.size()==points.size()),"Insufficient geometry/time count");
  for(double time:observation.per_point_time)require(std::isfinite(time),"Nonfinite acquisition time");
  if(local)for(auto & point:points)point=frame.pose.inverse()*point;
  return points;
}
void matrix_json(std::ostream & out,const Eigen::Isometry3d & pose)
{
  out<<'[';for(int row=0;row<4;++row)for(int col=0;col<4;++col)out<<(row||col?",":"")<<pose(row,col);out<<']';
}
std::vector<Eigen::Vector3d> bounded_cloud(const std::vector<Eigen::Vector3d> & points)
{
  if(points.size()<=50000)return points;
  std::vector<Eigen::Vector3d> result;result.reserve(50000);
  const std::size_t stride=(points.size()+49999)/50000;
  for(std::size_t i=0;i<points.size() && result.size()<50000;i+=stride)result.push_back(points[i]);
  return result;
}
double overlap_fraction(const std::vector<Eigen::Vector3d> & target,
                        const std::vector<Eigen::Vector3d> & source,
                        const Eigen::Isometry3d & target_T_source)
{
  if(target.empty()||source.empty())return 0;
  const std::size_t source_stride=std::max<std::size_t>(1,source.size()/2000);
  const std::size_t target_stride=std::max<std::size_t>(1,target.size()/5000);
  std::vector<Eigen::Vector3d> target_sampled,source_sampled;
  target_sampled.reserve((target.size()+target_stride-1)/target_stride);
  source_sampled.reserve((source.size()+source_stride-1)/source_stride);
  for(std::size_t j=0;j<target.size();j+=target_stride)target_sampled.push_back(target[j]);
  for(std::size_t i=0;i<source.size();i+=source_stride)source_sampled.push_back(source[i]);
  // Reuse small_gicp's deterministic voxel preprocessing and KdTree. The
  // threshold remains the same 0.2 m Euclidean radius as the old metric.
  auto [target_cloud,target_tree]=small_gicp::preprocess_points(target_sampled,.01,10,1);
  if(!target_cloud || !target_tree || target_cloud->size()==0)return 0;
  std::size_t accepted=0,total=0;
  for(const auto & source_point:source_sampled) {
    Eigen::Vector4d point; point.head<3>()=target_T_source*source_point; point.w()=1.0;
    std::size_t index=0;double distance=0;
    if(target_tree->knn_search(point,1,&index,&distance) && distance<=.04)++accepted;
    ++total;
  }
  return total?static_cast<double>(accepted)/total:0;
}
double geometry_ratio(const std::vector<Eigen::Vector3d> & points)
{
  if(points.size()<4)return 0;Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  for(const auto & p:points)mean+=p;mean/=static_cast<double>(points.size());
  Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();for(const auto & p:points){const auto d=p-mean;covariance+=d*d.transpose();}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance/static_cast<double>(points.size()-1));
  if(solver.info()!=Eigen::Success||!solver.eigenvalues().allFinite()||solver.eigenvalues().maxCoeff()<=0)return 0;
  return solver.eigenvalues().minCoeff()/solver.eigenvalues().maxCoeff();
}
int main(int argc,char ** argv)
{
  try {
    require(argc==3 || (argc==4 && std::string(argv[3])=="--enable-loops"),"Usage: offline_graph_optimizer <completed-session-dir> <new-output-dir> [--enable-loops]");
    const bool enable_loops=argc==4;
    const fs::path session=fs::canonical(argv[1]);const fs::path output=fs::weakly_canonical(fs::absolute(argv[2]));
    require(fs::is_directory(session) && !fs::exists(output),"Input must be a session; output must not exist");
    const auto relative=output.lexically_relative(session);
    require(relative.empty() || *relative.begin()=="..","Output must be outside the immutable input session");
    const SessionReadLock recording_lock(session);
    std::map<std::uint64_t,fs::path> directories;
    const std::regex pattern("keyframe_([0-9]+)");
    for(const auto & item:fs::directory_iterator(session)) {
      const auto name=item.path().filename().string();std::smatch match;
      require(name.find(".partial")==std::string::npos,"Session contains an incomplete archive; resolve it before optimization");
      if(std::regex_match(name,match,pattern))require(directories.emplace(std::stoull(match[1]),item.path()).second,"Duplicate frame ID");
    }
    require(directories.size()>=2 && directories.size()<=500,"Offline graph requires 2..500 contiguous keyframes");
    std::vector<ArchivedFrame> frames;std::vector<Eigen::Isometry3d> initial;
    std::vector<rm_nav_mapping::GraphEdge> edges;std::vector<rm_nav_registration::RegistrationResult> qualities;
    std::vector<Eigen::Vector3d> target;std::vector<std::vector<Eigen::Vector3d>> clouds;
    std::vector<rm_nav_mapping::Keyframe> keyframes;std::vector<rm_nav_mapping::GraphEdge> loop_edges;
    std::vector<rm_nav_registration::RegistrationResult> loop_qualities;
    rm_nav_registration::SmallGicpConfig config;config.use_voxelized_target=true;config.num_threads=1;
    config.max_input_points=200000;rm_nav_registration::SmallGicpBackend backend(config);
    for(const auto & item:directories) {
      ArchivedFrame frame;auto points=load(item.second,frame,true);
      require(frame.id==frames.size(),"Missing/noncontiguous keyframe ID");
      if(!frames.empty()) {
        const auto & previous=frames.back();
        require(frame.calibration==previous.calibration && frame.source==previous.source && frame.sensor==previous.sensor &&
                frame.body==previous.body && frame.stamp>previous.stamp && frame.sequence>previous.sequence,
                "Mixed calibration/source/reference point or nonmonotonic recording");
        rm_nav_registration::RegistrationRequest request;request.target_points=target;request.source_points=points;
        request.initial_target_T_source=previous.pose.inverse()*frame.pose;
        require(request.initial_target_T_source.translation().norm()<=2. &&
                Eigen::AngleAxisd(request.initial_target_T_source.rotation()).angle()<=1.,"Adjacent seed jump too large");
        const auto result=backend.register_clouds(request);
        std::cout<<"edge="<<previous.id<<"->"<<frame.id<<" converged="<<result.converged<<" rmse="<<result.residual
          <<" inliers="<<result.inlier_ratio<<" condition="<<result.condition_score<<'\n';
        require(rm_nav_registration::minimally_valid(result) && result.inlier_count>=50 && result.inlier_ratio>=.6 &&
                result.residual<=.15 && result.condition_score>=1e-6 && result.translation_delta<=.5 && result.rotation_delta<=.3,
                "VGICP adjacent edge rejected; no odometry-only bridge inserted");
        rm_nav_mapping::GraphEdge edge;edge.from=previous.id;edge.to=frame.id;edge.from_T_to=result.target_T_source;
        edges.push_back(edge);qualities.push_back(result);
      }
      target=std::move(points);initial.push_back(frame.pose);frames.push_back(frame);
      clouds.push_back(target);
      keyframes.push_back({frame.id,frame.stamp,frame.pose});
    }
    if(enable_loops) {
      rm_nav_mapping::LoopCandidateManager candidates;
      rm_nav_registration::KissGicpConfig kiss_config;kiss_config.num_threads=1;kiss_config.max_input_points=50000;
      kiss_config.refinement.num_threads=1;kiss_config.refinement.max_input_points=50000;
      rm_nav_registration::KissGicpBackend kiss(kiss_config);
      std::set<std::size_t> loop_current_frames;
      for(std::size_t j=0;j<keyframes.size();++j) {
        std::vector<rm_nav_mapping::Keyframe> history(keyframes.begin(),keyframes.begin()+j);
        for(const auto id:candidates.candidates(keyframes[j],history)) {
          if (loop_current_frames.count(j)) break;
          const std::size_t i=static_cast<std::size_t>(id);
          rm_nav_registration::RegistrationRequest forward_request;
          forward_request.target_points=bounded_cloud(clouds[i]);forward_request.source_points=bounded_cloud(clouds[j]);
          forward_request.initial_target_T_source=frames[i].pose.inverse()*frames[j].pose;
          const auto forward=kiss.register_clouds(forward_request);
          rm_nav_registration::RegistrationRequest reverse_request;
          reverse_request.target_points=bounded_cloud(clouds[j]);reverse_request.source_points=bounded_cloud(clouds[i]);
          reverse_request.initial_target_T_source=forward_request.initial_target_T_source.inverse();
          const auto reverse=kiss.register_clouds(reverse_request);
          rm_nav_mapping::LoopValidationInput validation{forward,reverse,
            overlap_fraction(clouds[i],clouds[j],forward.target_T_source),
            overlap_fraction(clouds[j],clouds[i],reverse.target_T_source),
            std::max(forward.translation_delta,reverse.translation_delta),
            std::max(forward.rotation_delta,reverse.rotation_delta),
            std::min(geometry_ratio(clouds[i]),geometry_ratio(clouds[j]))};
          const auto verdict=rm_nav_mapping::validate_loop(validation);
          std::cout<<"loop="<<i<<"->"<<j<<" accepted="<<verdict.accepted<<" reason="<<verdict.reason<<'\n';
          if(!verdict.accepted)continue;
          rm_nav_mapping::GraphEdge edge;edge.from=i;edge.to=j;edge.from_T_to=forward.target_T_source;
          edge.loop=true;edge.validated=true;edge.huber_k=1.;edge.sigmas<<.05,.05,.05,.10,.10,.10;
          loop_edges.push_back(edge);loop_qualities.push_back(forward);
          loop_current_frames.insert(j);
        }
      }
    }
    const auto solution=rm_nav_mapping::optimize_pose_graph(initial,edges,loop_edges);
    // Re-read original frames, never rebuild from a downsampled registration cloud or online merged map.
    std::map<std::tuple<long,long,long>,Eigen::Vector3d> voxels;
    for(std::size_t i=0;i<frames.size();++i) {
      ArchivedFrame reread;const auto points=load(frames[i].path,reread,false);
      require(reread.id==frames[i].id && reread.stamp==frames[i].stamp && reread.calibration==frames[i].calibration &&
              reread.source==frames[i].source && reread.sensor==frames[i].sensor && reread.body==frames[i].body &&
              reread.sequence==frames[i].sequence && reread.digest==frames[i].digest &&
              (reread.pose.matrix()-frames[i].pose.matrix()).norm()<1e-10,"Archive changed during optimization");
      const auto correction=solution.poses[i]*frames[i].pose.inverse();
      for(const auto & point:points) {
        const Eigen::Vector3d rebuilt=correction*point;
        require(rebuilt.allFinite() && rebuilt.cwiseAbs().maxCoeff()<=20000,"Invalid rebuilt map coordinate");
        voxels.try_emplace(std::make_tuple(static_cast<long>(std::floor(rebuilt.x()/.05)),
          static_cast<long>(std::floor(rebuilt.y()/.05)),static_cast<long>(std::floor(rebuilt.z()/.05))),rebuilt);
        require(voxels.size()<=1000000,"Rebuilt map voxel cap exceeded");
      }
    }
    // Offline artifact transaction: a unique temporary sibling is renamed only after all files close.
    fs::create_directories(output.parent_path());auto staging=output;
    staging+=".partial";
    require(fs::create_directory(staging),"Output staging exists; refusing overwrite");
    std::ofstream poses(staging/"optimized_poses.json");poses.exceptions(std::ios::badbit|std::ios::failbit);poses<<std::setprecision(17);
    poses<<"{\"schema\":1,\"coordinate_frame\":\"mapping_odom\",\"calibration_id\":"<<rm_nav_mapping::json_string(frames[0].calibration)
      <<",\"source_id\":"<<rm_nav_mapping::json_string(frames[0].source)<<",\"body_frame\":"<<rm_nav_mapping::json_string(frames[0].body)
      <<",\"official_alignment_applied\":false,\"initial_error\":"
      <<solution.initial_error<<",\"final_error\":"<<solution.final_error<<",\"iterations\":"<<solution.iterations
      <<",\"loop_count\":"<<solution.loop_count<<",\"poses\":[";
    for(std::size_t i=0;i<frames.size();++i){poses<<(i?",":"")<<"{\"id\":"<<frames[i].id<<",\"matrix\":";matrix_json(poses,solution.poses[i]);poses<<'}';}
    poses<<"]}\n";poses.close();
    std::ofstream hashes(staging/"input_hashes.json");hashes.exceptions(std::ios::badbit|std::ios::failbit);hashes<<'[';
    for(std::size_t i=0;i<frames.size();++i) {
      hashes<<(i?",":"")<<"{\"id\":"<<frames[i].id<<",\"stamp_ns\":"<<frames[i].stamp<<",\"observation_sha256\":\"";
      for(unsigned char value:frames[i].digest)hashes<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(value);
      hashes<<std::dec<<"\"}";
    }hashes<<"]\n";hashes.close();
    std::ofstream audit(staging/"adjacent_edges.json");audit.exceptions(std::ios::badbit|std::ios::failbit);audit<<std::setprecision(17)<<'[';
    for(std::size_t i=0;i<edges.size();++i) {
      const auto & quality=qualities[i];audit<<(i?",":"")<<"{\"from\":"<<edges[i].from<<",\"to\":"<<edges[i].to
        <<",\"algorithm\":\"small_gicp_VGICP\",\"matrix\":";matrix_json(audit,edges[i].from_T_to);
      audit<<",\"rmse_m\":"<<quality.residual<<",\"inlier_ratio\":"<<quality.inlier_ratio<<",\"condition\":"<<quality.condition_score
        <<",\"sigmas_rotation_translation\":[0.03,0.03,0.03,0.05,0.05,0.05]}";
    }audit<<"]\n";audit.close();
    std::ofstream loops(staging/"loop_edges.json");loops.exceptions(std::ios::badbit|std::ios::failbit);loops<<std::setprecision(17)<<'[';
    for(std::size_t i=0;i<loop_edges.size();++i) {
      const auto & q=loop_qualities[i];loops<<(i?",":"")<<"{\"from\":"<<loop_edges[i].from<<",\"to\":"<<loop_edges[i].to<<",\"algorithm\":\"KISS_GICP_bidirectional_validated\",\"matrix\":";
      matrix_json(loops,loop_edges[i].from_T_to);loops<<",\"rmse_m\":"<<q.residual<<",\"inlier_ratio\":"<<q.inlier_ratio<<",\"condition\":"<<q.condition_score<<"}";
    }loops<<"]\n";loops.close();
    std::ofstream pcd(staging/"rebuilt_map.pcd",std::ios::binary);pcd.exceptions(std::ios::badbit|std::ios::failbit);
    pcd<<"# mapping_odom; official alignment pending\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
      <<voxels.size()<<"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "<<voxels.size()<<"\nDATA binary\n";
    for(const auto & item:voxels){const auto point=item.second.cast<float>().eval();pcd.write(reinterpret_cast<const char *>(point.data()),12);}pcd.close();
    for(const auto & name:{"optimized_poses.json","adjacent_edges.json","loop_edges.json","input_hashes.json","rebuilt_map.pcd"}) {
      const int fd=::open((staging/name).c_str(),O_RDONLY|O_CLOEXEC);
      require(fd>=0,"Cannot fsync output file");const int result=::fsync(fd);::close(fd);require(result==0,"Output file fsync failed");
    }
    rm_nav_mapping::durable_directory(staging);
    require(::syscall(SYS_renameat2,AT_FDCWD,staging.c_str(),AT_FDCWD,output.c_str(),RENAME_NOREPLACE)==0,
            "Atomic no-replace output commit failed");rm_nav_mapping::durable_directory(output.parent_path());
    std::cout<<"PASS actual VGICP + GTSAM adjacent graph; error="<<solution.initial_error<<"->"<<solution.final_error
      <<" rebuilt_voxels="<<voxels.size()<<" output="<<output<<'\n';return 0;
  }catch(const std::exception & error){std::cerr<<"Mapping optimization refused: "<<error.what()<<'\n';return 1;}
}
