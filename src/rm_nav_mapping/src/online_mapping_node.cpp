#include "rm_nav_mapping/archive_reader.hpp"
#include "rm_nav_mapping/online_mapping.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <regex>
#include <sstream>

namespace rm_nav_mapping {
namespace fs=std::filesystem;
class OnlineMappingNode : public rclcpp::Node {
public:
  OnlineMappingNode():Node("online_mapping") {
    const auto session=declare_parameter<std::string>("session","");
    if(session.empty() || !fs::path(session).is_absolute() || fs::is_symlink(session))
      throw std::invalid_argument("An explicit absolute recorder session directory is required");
    session_=fs::canonical(session);
    if(!fs::is_directory(session_))throw std::invalid_argument("Session is not a directory");
    calibration_=declare_parameter<std::string>("calibration_id","");
    if(calibration_.empty())throw std::invalid_argument("Expected calibration_id is required");
    body_=declare_parameter<std::string>("body_frame","base_footprint");
    if(body_!="base_footprint" && body_!="nav_reference")
      throw std::invalid_argument("Unsupported explicit mapping reference");
    OnlineMappingConfig config;
    config.max_keyframes=declare_parameter<int>("max_keyframes",500);
    config.max_cloud_points=declare_parameter<int>("max_cloud_points",20000);
    config.max_map_points=declare_parameter<int>("max_map_points",500000);
    config.map_voxel_m=declare_parameter<double>("map_voxel_m",.15);
    config.loop_budget_s=declare_parameter<double>("loop_budget_s",3.);
    config.candidates.max_candidates=declare_parameter<int>("max_candidates",2);
    config.candidates.min_time_separation_ns=static_cast<std::int64_t>(declare_parameter<double>("min_loop_time_s",10.)*1e9);
    config.candidates.min_keyframe_separation=declare_parameter<int>("min_loop_frames",10);
    engine_=std::make_unique<OnlineMapping>(config);
    auto qos=rclcpp::QoS(1).reliable().transient_local();
    cloud_=create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/optimized_map",qos);
    preview_=create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/optimized_map_preview",qos);
    path_=create_publisher<nav_msgs::msg::Path>("/mapping/optimized_path",qos);
    graph_=create_publisher<visualization_msgs::msg::MarkerArray>("/mapping/loop_graph",qos);
    correction_=create_publisher<geometry_msgs::msg::PoseStamped>("/mapping/mapping_odom_T_odom",qos);
    status_=create_publisher<std_msgs::msg::String>("/mapping/online_status",qos);
    health_=create_subscription<std_msgs::msg::Bool>("/state/chassis_healthy",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){
      std::lock_guard<std::mutex> lock(mutex_);upstream_=m->data;upstream_at_=std::chrono::steady_clock::now();});
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{publish_status();});
    worker_=std::thread([this]{work();});
  }
  ~OnlineMappingNode() override {
    stop_=true;cv_.notify_all();if(worker_.joinable())worker_.join();
  }
private:
  geometry_msgs::msg::Pose pose(const Eigen::Isometry3d & t) {
    geometry_msgs::msg::Pose p;p.position.x=t.translation().x();p.position.y=t.translation().y();p.position.z=t.translation().z();
    Eigen::Quaterniond q(t.linear());p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;
  }
  void publish_snapshot() {
    // Identical source stamp is the snapshot identity across all three topics.
    std_msgs::msg::Header header;header.frame_id="mapping_odom";
    header.stamp=rclcpp::Time(engine_->frames().back().stamp_ns);
    const auto points=engine_->map_points();
    sensor_msgs::msg::PointCloud2 cloud;cloud.header=header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    for(const auto & p:points){*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}
    // The full bounded map remains available, but Wi-Fi defaults to <=50k
    // points spanning the whole map, not just its most recent region.
    const std::size_t stride=std::max<std::size_t>(1,(points.size()+49999)/50000);
    sensor_msgs::msg::PointCloud2 preview;preview.header=header;
    sensor_msgs::PointCloud2Modifier pm(preview);pm.setPointCloud2FieldsByString(1,"xyz");
    pm.resize((points.size()+stride-1)/stride);
    sensor_msgs::PointCloud2Iterator<float> px(preview,"x"),py(preview,"y"),pz(preview,"z");
    for(std::size_t i=0;i<points.size();i+=stride){*px=points[i].x();*py=points[i].y();*pz=points[i].z();++px;++py;++pz;}
    nav_msgs::msg::Path path;path.header=header;
    for(const auto & t:engine_->poses()){geometry_msgs::msg::PoseStamped p;p.header=header;p.pose=pose(t);path.poses.push_back(p);}
    visualization_msgs::msg::MarkerArray graph;
    visualization_msgs::msg::Marker nodes;nodes.header=header;nodes.ns="keyframes";nodes.id=0;
    nodes.type=visualization_msgs::msg::Marker::SPHERE_LIST;nodes.action=0;
    nodes.scale.x=nodes.scale.y=nodes.scale.z=.12;nodes.color.r=1;nodes.color.g=.8;nodes.color.a=1;
    for(const auto & p:path.poses)nodes.points.push_back(p.pose.position);
    graph.markers.push_back(nodes);
    visualization_msgs::msg::Marker edges;edges.header=header;edges.ns="accepted_loops";edges.id=1;
    edges.type=visualization_msgs::msg::Marker::LINE_LIST;edges.action=0;edges.scale.x=.04;edges.color.r=1;edges.color.a=1;
    for(const auto & e:engine_->loops()){edges.points.push_back(path.poses[e.from].pose.position);edges.points.push_back(path.poses[e.to].pose.position);}
    graph.markers.push_back(edges);
    geometry_msgs::msg::PoseStamped correction;correction.header=header;
    correction.pose=pose(engine_->poses().back()*engine_->frames().back().odom_T_chassis.inverse());
    cloud_->publish(cloud);preview_->publish(preview);path_->publish(path);graph_->publish(graph);correction_->publish(correction);
    std::lock_guard<std::mutex> lock(mutex_);
    revision_=engine_->revision();frames_=engine_->frames().size();loops_=engine_->loops().size();points_=points.size();
    candidates_=engine_->candidates_tested();rejected_=engine_->loops_rejected();
    work_ms_=engine_->last_work_ms();graph_ms_=engine_->last_graph_ms();reason_=engine_->reason();
    if(backlog_>0)--backlog_;
  }
  void work() {
    std::size_t published_revision=0;
    auto published_at=std::chrono::steady_clock::time_point{};
    while(!stop_) {
      bool processed=false;
      try {
        std::size_t committed=0;
        // Recorder exposes a keyframe only after its directory is atomically
        // renamed. Ignore .partial; missing IDs are never skipped.
        for(const auto & item:fs::directory_iterator(session_))
          if(std::regex_match(item.path().filename().string(),std::regex("keyframe_[0-9]+")))++committed;
        {std::lock_guard<std::mutex> lock(mutex_);backlog_=committed>engine_->frames().size()?committed-engine_->frames().size():0;}
        if(committed>engine_->frames().size()) {
          const auto index=engine_->frames().size();const auto dir=session_/("keyframe_"+std::to_string(index));
          archive::ArchivedFrame frame;auto points=archive::load(dir,frame,true,false);
          if(frame.calibration!=calibration_ || frame.body!=body_ || frame.source!="mid360_main" || frame.sensor!="front_mid360" ||
             (!source_.empty() && (source_!=frame.source || frame.sequence<=last_sequence_)))
            throw std::runtime_error("Mixed calibration/source/body or nonmonotonic sequence");
          engine_->append({frame.id,frame.stamp,frame.pose},points);
          source_=frame.source;last_sequence_=frame.sequence;
          processed=true;
        }
        if(engine_->revision()!=published_revision &&
           std::chrono::duration<double>(std::chrono::steady_clock::now()-published_at).count()>=1.){
          publish_snapshot();published_revision=engine_->revision();published_at=std::chrono::steady_clock::now();
        }
      }catch(const std::exception & e){
        {std::lock_guard<std::mutex> lock(mutex_);fault_=true;reason_=e.what();}
        RCLCPP_ERROR(get_logger(),"Online mapping latched fault, snapshot retained: %s",e.what());
        return;
      }
      // Ingest every committed frame without an artificial one-frame/second
      // bottleneck; publish only the latest complete snapshot at <=1 Hz.
      std::unique_lock<std::mutex> lock(wait_mutex_);
      cv_.wait_for(lock,std::chrono::milliseconds(processed?25:100),[this]{return stop_.load();});
    }
  }
  void publish_status() {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool fresh=upstream_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-upstream_at_).count()<.5;
    boost::property_tree::ptree data;
    data.put("mode","ONLINE_MAPPING_ONLY_NOT_PUBLISHED_MAP");data.put("frame","mapping_odom");
    data.put("session",session_.string());data.put("revision",revision_);data.put("keyframes",frames_);data.put("map_points",points_);
    data.put("accepted_loops",loops_);data.put("candidates_tested",candidates_);data.put("loops_rejected",rejected_);
    data.put("pending_keyframes",backlog_);data.put("last_work_ms",work_ms_);data.put("last_graph_ms",graph_ms_);
    data.put("upstream_healthy",fresh);data.put("fault",fault_);data.put("reason",reason_);
    data.put("budget_semantics","soft deadline, non-preemptible registration; overshoot rejected");
    std::ostringstream stream;boost::property_tree::write_json(stream,data,false);
    std_msgs::msg::String msg;msg.data=stream.str();status_->publish(msg);
  }
  fs::path session_;std::string calibration_,source_,body_;std::uint64_t last_sequence_{0};
  std::unique_ptr<OnlineMapping> engine_;std::thread worker_;std::atomic<bool> stop_{false};
  std::mutex mutex_,wait_mutex_;std::condition_variable cv_;
  bool upstream_{false},fault_{false};std::chrono::steady_clock::time_point upstream_at_{};
  std::size_t revision_{0},frames_{0},loops_{0},points_{0},backlog_{0},candidates_{0},rejected_{0};
  double work_ms_{0},graph_ms_{0};std::string reason_{"waiting for committed keyframes"};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_,preview_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr correction_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<rm_nav_mapping::OnlineMappingNode>());}
  catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
