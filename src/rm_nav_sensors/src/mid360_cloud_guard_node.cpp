#include "rm_nav_sensors/mid360_quality.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace rm_nav_sensors {
class Mid360CloudGuard : public rclcpp::Node {
public:
  Mid360CloudGuard() : Node("mid360_cloud_guard") {
    frame_ = declare_parameter("sensor_frame","front_mid360");
    max_age_ = declare_parameter("max_age",.25);
    min_points_ = declare_parameter("min_valid_points",500);
    fraction_ = declare_parameter("min_valid_fraction",.1);
    variance_ = declare_parameter("min_geometry_variance",.02);
    ratio_ = declare_parameter("min_geometry_eigen_ratio",.001);
    if (frame_.empty() || !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 1 ||
        min_points_ < 50 || min_points_ > 200000 || !std::isfinite(fraction_) || fraction_ <= 0 || fraction_ > 1 ||
        !std::isfinite(variance_) || variance_ <= 0 || !std::isfinite(ratio_) || ratio_ <= 0 || ratio_ > 1)
      throw std::invalid_argument("Invalid explicit raw input gate configuration");
    output_ = create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/front_mid360/guarded_points",rclcpp::SensorDataQoS());
    health_ = create_publisher<std_msgs::msg::Bool>("/sensors/front_mid360/cloud_healthy",10);
    reason_ = create_publisher<std_msgs::msg::String>("/sensors/front_mid360/cloud_reason",10);
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        try {
          const auto stamp = rclcpp::Time(msg->header.stamp,get_clock()->get_clock_type());
          const double age = (now()-stamp).seconds();
          if (msg->header.frame_id != frame_ || stamp.nanoseconds() <= last_seen_ || age < -.05 || age > max_age_)
            throw std::invalid_argument("Raw scan frame/order/source age invalid");
          last_seen_ = stamp.nanoseconds();
          const auto quality = inspect_mid360(*msg,min_points_,fraction_,variance_,ratio_);
          if ((now()-stamp).seconds() > max_age_)
            throw std::invalid_argument("Raw scan became stale during validation");
          output_->publish(*msg);  // All bytes/fields/frame/source time preserved; no filtering/restamping.
          valid_ = true; accepted_stamp_ = stamp.nanoseconds();
          accepted_at_ = std::chrono::steady_clock::now();
          detail_ = "accepted native scan; valid_points="+std::to_string(quality.valid_points)+
                    "; eigen_ratio="+std::to_string(quality.eigen_ratio);
        } catch (const std::exception & error) {
          valid_ = false; detail_ = error.what();
        }
        publish_status();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50),[this]() {
      if (valid_ && ((now().nanoseconds()-accepted_stamp_)/1e9 > max_age_ ||
          (now().nanoseconds()-accepted_stamp_)/1e9 < -.05 ||
          std::chrono::duration<double>(std::chrono::steady_clock::now()-accepted_at_).count() > max_age_)) {
        valid_ = false; detail_ = "raw scan timed out";
      }
      publish_status();
    });
  }
private:
  void publish_status() {
    std_msgs::msg::Bool h; h.data=valid_; health_->publish(h);
    std_msgs::msg::String r; r.data=detail_; reason_->publish(r);
  }
  std::string frame_, detail_{"no accepted native scan"};
  double max_age_,fraction_,variance_,ratio_;
  int min_points_;
  bool valid_{false};
  std::int64_t last_seen_{0},accepted_stamp_{0};
  std::chrono::steady_clock::time_point accepted_at_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<rm_nav_sensors::Mid360CloudGuard>());
  rclcpp::shutdown();
}
