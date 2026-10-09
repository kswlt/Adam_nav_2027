#include "rm_nav_localization/local_submap_builder.hpp"
#include "rm_nav_sensors/point_cloud.hpp"
#include <rm_nav_interfaces/msg/observation_batch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <tuple>

namespace rm_nav_localization {
class ObservationSubmap : public rclcpp::Node {
public:
  ObservationSubmap() : Node("observation_submap"),builder_(1000000000)
  {
    source_=declare_parameter("source_id","mid360_main");
    sensor_=declare_parameter("sensor_frame","front_mid360");
    calibration_=declare_parameter<std::string>("calibration_id","");
    voxel_=declare_parameter("voxel_resolution",0.05);
    max_age_=declare_parameter("max_age",0.4);
    const auto window=declare_parameter("window_seconds",1.0);
    output_limit_=declare_parameter("max_output_points",50000);
    if (source_.empty() || sensor_.empty() || sensor_=="odom" || calibration_.empty() || !std::isfinite(voxel_) || voxel_<.001 ||
        !std::isfinite(max_age_) || max_age_<=0 || !std::isfinite(window) || window<=0 || window>5 ||
        output_limit_<20 || output_limit_>50000) throw std::invalid_argument("Invalid submap source/window/limits");
    builder_=LocalSubmapBuilder(static_cast<std::int64_t>(window*1e9));
    output_=create_publisher<sensor_msgs::msg::PointCloud2>("/localization/odom_submap",rclcpp::SensorDataQoS().keep_last(1));
    health_=create_publisher<std_msgs::msg::Bool>("/localization/submap_healthy",10);
    diagnostics_=create_publisher<std_msgs::msg::String>("/localization/submap_reason",10);
    state_sub_=create_subscription<std_msgs::msg::Bool>("/state/chassis_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){state_valid_=msg->data;state_at_=Clock::now();if(!state_valid_)reset("chassis state lost");});
    input_=create_subscription<rm_nav_interfaces::msg::ObservationBatch>("/sensors/localization_observations",10,
      [this](rm_nav_interfaces::msg::ObservationBatch::ConstSharedPtr msg){receive(*msg);});
    source_health_sub_=create_subscription<std_msgs::msg::Bool>("/sensors/localization_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){if(!msg->data)reset("observation source invalidated");});
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this](){publish();});
  }
private:
  using Clock=std::chrono::steady_clock;
  bool state_ready() const {return state_valid_ && std::chrono::duration<double>(Clock::now()-state_at_).count()<=.25;}
  void reset(const std::string & reason){builder_.clear();pending_=valid_=false;reason_=reason;}
  void receive(const rm_nav_interfaces::msg::ObservationBatch & batch)
  {
    try {
      const auto stamp=rclcpp::Time(batch.header.stamp,get_clock()->get_clock_type());
      const double age=(now()-stamp).seconds();
      if (!state_ready() || batch.header.frame_id!="odom" || stamp.nanoseconds()<=last_stamp_ || age<-.05 || age>max_age_)
        throw std::invalid_argument("Observation batch state/frame/order/age invalid");
      const rm_nav_interfaces::msg::ObservationFrame * primary=nullptr;
      for (const auto & frame:batch.frames) if (frame.roles & frame.LOCALIZATION) {
        if(primary) throw std::invalid_argument("Multiple primary localization frames are not configured");
        primary=&frame;
      }
      if (!primary || primary->source_id!=source_ || primary->calibration_id!=calibration_ ||
          primary->sensor_frame!=sensor_ || primary->roles!=(primary->LOCALIZATION|primary->RELOCALIZATION) ||
          !primary->healthy || !primary->deskewed || primary->sequence<=last_sequence_ ||
          primary->header!=batch.header || primary->point_cloud.header!=batch.header)
        throw std::invalid_argument("Primary observation contract/calibration/sequence invalid");
      auto points=rm_nav_sensors::read_xyz(primary->point_cloud,200000);
      const auto & p=primary->sensor_pose.position;
      const auto & q=primary->sensor_pose.orientation;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm-1)>1e-5)
        throw std::invalid_argument("Observation origin metadata invalid");
      if(!primary->per_point_time.empty() && primary->per_point_time.size()!=points.size())
        throw std::invalid_argument("Per-point time count invalid");
      for(double time:primary->per_point_time) if(!std::isfinite(time))throw std::invalid_argument("Nonfinite per-point time");
      if(points.size()<20)throw std::invalid_argument("Insufficient submap observation points");
      last_stamp_=stamp.nanoseconds();last_sequence_=primary->sequence;
      if (builder_.stored_point_count()+points.size()>200000 || builder_.frame_count()>=100) builder_.clear();
      LocalSubmapFrame frame;frame.stamp_ns=stamp.nanoseconds();frame.points=std::move(points);
      // Already-odom XYZ must not receive the sensor origin as a second transform.
      builder_.add_frame(std::move(frame));header_=batch.header;accepted_at_=Clock::now();pending_=valid_=true;
      reason_="fresh primary observation added; bounded rolling window";
    }catch(const std::exception & error){reset(error.what());}
  }
  void publish()
  {
    const double age=(now().nanoseconds()-last_stamp_)/1e9;
    if(!state_ready() || age<-.05 || age>max_age_ ||
       std::chrono::duration<double>(Clock::now()-accepted_at_).count()>max_age_)reset("observation/state timed out; window cleared");
    if(valid_ && pending_) {
      const auto submap=builder_.build();
      std::map<std::tuple<long,long,long>,Eigen::Vector3d> voxels;
      for(const auto & point:submap.points) {
        voxels.try_emplace(std::make_tuple(static_cast<long>(std::floor(point.x()/voxel_)),
          static_cast<long>(std::floor(point.y()/voxel_)),static_cast<long>(std::floor(point.z()/voxel_))),point);
      }
      std::vector<Eigen::Vector3d> selected;
      const std::size_t count=std::min<std::size_t>(voxels.size(),output_limit_);
      selected.reserve(count);
      std::size_t index=0,next=0;
      for(const auto & entry:voxels) {
        if(selected.size()<count && index>=next) {
          selected.push_back(entry.second);next=selected.size()*voxels.size()/count;
        }
        ++index;
      }
      if(selected.size()<20)reset("Voxel geometry insufficient; no submap emitted");
      else {output_->publish(rm_nav_sensors::xyz_cloud(header_,selected));pending_=false;}
    }
    std_msgs::msg::Bool health;health.data=valid_;health_->publish(health);
    std_msgs::msg::String reason;reason.data=reason_;diagnostics_->publish(reason);
  }
  LocalSubmapBuilder builder_;
  std::string source_,sensor_,calibration_,reason_{"no verified observation"};
  double voxel_,max_age_;
  int output_limit_;
  bool state_valid_{false},pending_{false},valid_{false};
  std::int64_t last_stamp_{0};
  std::uint64_t last_sequence_{0};
  std_msgs::msg::Header header_;
  Clock::time_point state_at_{},accepted_at_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_,source_health_sub_;
  rclcpp::Subscription<rm_nav_interfaces::msg::ObservationBatch>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<rm_nav_localization::ObservationSubmap>());rclcpp::shutdown();}
