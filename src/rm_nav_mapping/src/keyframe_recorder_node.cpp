#include "rm_nav_mapping/archive_io.hpp"
#include "rm_nav_mapping/keyframe_manager.hpp"
#include "rm_nav_sensors/point_cloud.hpp"
#include <rm_nav_interfaces/msg/observation_batch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <random>
#include <fcntl.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/fs.h>
#include <cerrno>
#include <sys/file.h>

namespace rm_nav_mapping {
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;

class KeyframeRecorder : public rclcpp::Node {
public:
  KeyframeRecorder():Node("keyframe_recorder"),buffer_(get_clock()),listener_(buffer_)
  {
    calibration_=declare_parameter<std::string>("calibration_id","");
    body_=declare_parameter("body_frame","base_footprint");
    source_=declare_parameter("source_id","mid360_main");
    sensor_=declare_parameter("sensor_frame","front_mid360");
    const auto root=declare_parameter<std::string>("archive_root","");
    max_age_=declare_parameter("max_age",.3);
    // Offline PoseGraph currently has a hard bounded graph of 500 poses. Keep the
    // recorder contract identical so frame 501 is refused explicitly rather than
    // producing an archive that the optimizer cannot consume.
    max_frames_=declare_parameter<std::int64_t>("max_keyframes",500);
    max_bytes_=declare_parameter<std::int64_t>("max_archive_bytes",2147483648LL);
    KeyframeTriggerConfig config;
    config.translation_threshold_m=declare_parameter("translation_threshold_m",.15);
    config.yaw_threshold_rad=declare_parameter("yaw_threshold_rad",.17453292519943295);
    manager_=KeyframeManager(config);
    if(calibration_.empty() || source_.empty() || sensor_.empty() || body_.empty() || body_=="odom" ||
       root.empty() || !fs::path(root).is_absolute() || !std::isfinite(max_age_) || max_age_<=0 || max_age_>1 ||
       max_frames_<1 || max_frames_>500 || max_bytes_<4096 || max_bytes_>107374182400LL)
      throw std::invalid_argument("Archive max_keyframes must be 1..500; use a new session for the next segment");
    fs::create_directories(root);
    root_=fs::canonical(root);
    std::random_device random;
    for(int attempt=0;attempt<10;++attempt) {
      session_=root_/ ("session_"+std::to_string(now().nanoseconds())+"_"+std::to_string(random()));
      if(fs::create_directory(session_))break;
      session_.clear();
    }
    if(session_.empty())throw std::runtime_error("Cannot create unique recording session");
    lock_fd_=::open((session_/"recording.lock").c_str(),O_RDWR|O_CREAT|O_EXCL|O_CLOEXEC,0600);
    if(lock_fd_<0 || ::flock(lock_fd_,LOCK_EX|LOCK_NB)!=0)throw std::runtime_error("Cannot exclusively lock recording session");
    durable_directory(root_);
    status_pub_=create_publisher<std_msgs::msg::String>("/mapping/recorder_status",10);
    accepted_pub_=create_publisher<std_msgs::msg::String>("/mapping/keyframe_archive",10);
    health_pub_=create_publisher<std_msgs::msg::Bool>("/mapping/recorder_healthy",10);
    state_sub_=create_subscription<std_msgs::msg::Bool>("/state/chassis_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){state_=msg->data;state_at_=Clock::now();if(!state_)pending_.reset();});
    source_sub_=create_subscription<std_msgs::msg::Bool>("/sensors/localization_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){source_valid_=msg->data;source_at_=Clock::now();if(!source_valid_)pending_.reset();});
    input_=create_subscription<rm_nav_interfaces::msg::ObservationBatch>("/sensors/localization_observations",rclcpp::QoS(1),
      [this](rm_nav_interfaces::msg::ObservationBatch::ConstSharedPtr msg){
        // Latest-only pending work; disk I/O happens on the recorder timer, never in the LIO process.
        if(!fault_)pending_=msg;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){tick();});
    RCLCPP_INFO(get_logger(),"New mapping archive: %s",session_.c_str());
  }
  ~KeyframeRecorder() override {if(lock_fd_>=0)::close(lock_fd_);}
private:
  double elapsed(Clock::time_point time)const{return std::chrono::duration<double>(Clock::now()-time).count();}
  bool upstream()const{return state_ && source_valid_ && elapsed(state_at_)<=.25 && elapsed(source_at_)<=.3;}
  void tick()
  {
    if(!upstream()){
      pending_.reset();valid_=false;
      if(!fault_)reason_="chassis/source health absent or stale";
    }
    if(pending_ && !fault_){const auto batch=pending_;pending_.reset();receive(*batch);}
    const double source_age=(now().nanoseconds()-accepted_stamp_)/1e9;
    if(elapsed(accepted_at_)>max_age_ || source_age<-.05 || source_age>max_age_)valid_=false;
    std_msgs::msg::Bool health;health.data=valid_ && !fault_ && upstream();health_pub_->publish(health);
    std_msgs::msg::String status;
    status.data="session="+session_.string()+" frames="+std::to_string(manager_.keyframes().size())+
      " bytes="+std::to_string(written_bytes_)+" fault="+(fault_?"true":"false")+" reason="+reason_;
    status_pub_->publish(status);
  }
  void receive(const rm_nav_interfaces::msg::ObservationBatch & batch)
  {
    try {
      const auto stamp=rclcpp::Time(batch.header.stamp,get_clock()->get_clock_type());
      const double age=(now()-stamp).seconds();
      if(!upstream() || batch.header.frame_id!="odom" || stamp.nanoseconds()<=last_stamp_ || age<-.05 || age>max_age_ ||
         batch.frames.empty() || batch.frames.size()>16)
        throw std::invalid_argument("Batch health/frame/order/age invalid");
      const rm_nav_interfaces::msg::ObservationFrame * observation=nullptr;
      for(const auto & frame:batch.frames)if(frame.roles & frame.LOCALIZATION) {
        if(observation)throw std::invalid_argument("Ambiguous primary mapping observation");
        observation=&frame;
      }
      if(!observation || observation->source_id!=source_ || observation->sensor_frame!=sensor_ ||
         observation->calibration_id!=calibration_ || observation->header!=batch.header ||
         observation->point_cloud.header!=batch.header || !observation->healthy || !observation->deskewed ||
         observation->roles!=(observation->LOCALIZATION|observation->RELOCALIZATION) ||
         observation->sequence<=last_sequence_ || observation->point_cloud.data.size()>16*1024*1024)
        throw std::invalid_argument("Observation provenance/sequence/size invalid");
      const auto points=rm_nav_sensors::read_xyz(observation->point_cloud,200000);
      if(points.size()<20)throw std::invalid_argument("Insufficient original keyframe geometry");
      const auto & origin=observation->sensor_pose;
      const Eigen::Quaterniond origin_q(origin.orientation.w,origin.orientation.x,origin.orientation.y,origin.orientation.z);
      if(!origin_q.coeffs().allFinite() || std::abs(origin_q.norm()-1)>1e-5 ||
         !std::isfinite(origin.position.x) || !std::isfinite(origin.position.y) || !std::isfinite(origin.position.z))
        throw std::invalid_argument("Invalid source origin metadata");
      if(!observation->per_point_time.empty() && observation->per_point_time.size()!=points.size())
        throw std::invalid_argument("Acquisition time count invalid");
      for(double time:observation->per_point_time)if(!std::isfinite(time))throw std::invalid_argument("Acquisition time invalid");
      const auto transform=buffer_.lookupTransform("odom",body_,stamp,rclcpp::Duration::from_seconds(.05)).transform;
      const double after_wait=(now()-stamp).seconds();
      if(!upstream() || after_wait<-.05 || after_wait>max_age_)throw std::invalid_argument("State/observation stale after exact-time TF wait");
      Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
      const Eigen::Quaterniond q(transform.rotation.w,transform.rotation.x,transform.rotation.y,transform.rotation.z);
      if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-5)throw std::invalid_argument("Invalid chassis orientation");
      pose.linear()=q.normalized().toRotationMatrix();
      pose.translation()=Eigen::Vector3d(transform.translation.x,transform.translation.y,transform.translation.z);
      auto candidate=manager_;
      const bool triggered=candidate.consider(stamp.nanoseconds(),pose);
      last_stamp_=stamp.nanoseconds();last_sequence_=observation->sequence;
      if(triggered) {
        // The manager advances only after a durable frame. Any storage error latches a fault.
        try {archive(candidate.keyframes().back(),*observation);}
        catch(const std::exception & error){fault_=true;valid_=false;reason_=error.what();return;}
      }
      manager_=std::move(candidate);accepted_at_=Clock::now();accepted_stamp_=stamp.nanoseconds();valid_=true;
      reason_=triggered?"original keyframe committed":"valid observation; chassis trigger not reached";
    }catch(const std::exception & error){valid_=false;reason_=error.what();}
  }
  void archive(const Keyframe & keyframe,const rm_nav_interfaces::msg::ObservationFrame & observation)
  {
    if(manager_.keyframes().size()>=static_cast<std::size_t>(max_frames_))
      throw std::runtime_error("Keyframe quota reached at graph limit; recording stopped, start a new segment");
    rclcpp::Serialization<rm_nav_interfaces::msg::ObservationFrame> serializer;
    rclcpp::SerializedMessage serialized;serializer.serialize_message(&observation,&serialized);
    const auto & buffer=serialized.get_rcl_serialized_message();
    std::ostringstream metadata;metadata<<std::setprecision(17);
    metadata<<"{\n\"schema\":1,\"id\":"<<keyframe.id<<",\"stamp_ns\":"<<keyframe.stamp_ns
      <<",\"calibration_id\":"<<json_string(calibration_)<<",\"source_id\":"<<json_string(source_)
      <<",\"sensor_frame\":"<<json_string(sensor_)<<",\"body_frame\":"<<json_string(body_)
      <<",\"cloud_frame\":\"odom\",\"observation_file\":\"observation.cdr\",\"observation_bytes\":"<<buffer.buffer_length
      <<",\"reconstruction\":\"optimized_T_body * inverse(recorded_odom_T_body) * original_odom_point\""
      <<",\"recorded_odom_T_body\":[";
    for(int row=0;row<4;++row)for(int col=0;col<4;++col)metadata<<(row||col?",":"")<<keyframe.odom_T_chassis(row,col);
    metadata<<"]}\n";
    const auto text=metadata.str();const std::uint64_t size=buffer.buffer_length+text.size();
    if(size>static_cast<std::uint64_t>(max_bytes_)-written_bytes_ || fs::space(session_).available<size+64*1024*1024)
      throw std::runtime_error("Archive bytes/free-space quota reached; recording stopped");
    const auto name="keyframe_"+std::to_string(keyframe.id);
    const auto staging=session_/(name+".partial"),final=session_/name;
    if(!fs::create_directory(staging))throw std::runtime_error("Archive staging collision");
    // On failure retain .partial for diagnosis. It is never announced as a committed keyframe.
    durable_file(staging/"observation.cdr",buffer.buffer,buffer.buffer_length);
    durable_file(staging/"metadata.json",text.data(),text.size());
    durable_directory(staging);
    if(::syscall(SYS_renameat2,AT_FDCWD,staging.c_str(),AT_FDCWD,final.c_str(),RENAME_NOREPLACE)!=0)
      throw std::runtime_error("Atomic no-replace archive commit failed");
    durable_directory(session_);written_bytes_+=size;
    std_msgs::msg::String path;path.data=final.string();accepted_pub_->publish(path);
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  KeyframeManager manager_;
  int lock_fd_{-1};
  fs::path root_,session_;
  std::string calibration_,body_,source_,sensor_,reason_{"no valid mapping observation"};
  double max_age_;
  std::int64_t max_frames_,max_bytes_,last_stamp_{0},accepted_stamp_{0};
  std::uint64_t written_bytes_{0},last_sequence_{0};
  bool state_{false},source_valid_{false},valid_{false},fault_{false};
  Clock::time_point state_at_{},source_at_{},accepted_at_{};
  rm_nav_interfaces::msg::ObservationBatch::ConstSharedPtr pending_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,accepted_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_,source_sub_;
  rclcpp::Subscription<rm_nav_interfaces::msg::ObservationBatch>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<rm_nav_mapping::KeyframeRecorder>());rclcpp::shutdown();}
