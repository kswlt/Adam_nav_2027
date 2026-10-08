#pragma once
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <chrono>
#include <vector>

namespace rm_nav_localization {
// Async Nav2 boundary. Cancellation acceptance alone is not terminal cancellation.
class NavReset {
public:
  enum class State { IDLE, CANCELLING, QUIESCENT, CLEARING, DONE, FAILED };
  NavReset(rclcpp::Node & node, const std::vector<std::string> & actions)
  {
    if (actions.empty()) throw std::invalid_argument("Navigation cancellation list must not be empty");
    for (const auto & action : actions) {
      auto endpoint = std::make_shared<Endpoint>();
      endpoint->client = node.create_client<action_msgs::srv::CancelGoal>(action + "/_action/cancel_goal");
      endpoint->status = node.create_subscription<action_msgs::msg::GoalStatusArray>(
        action + "/_action/status", rclcpp::QoS(1).transient_local(),
        [endpoint](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
          endpoint->observed = std::chrono::steady_clock::now();
          endpoint->active = false;
          for (const auto & goal : msg->status_list) {
            if (goal.status == 1 || goal.status == 2 || goal.status == 3) endpoint->active = true;
          }
        });
      endpoints_.push_back(endpoint);
    }
    for (const auto & name : {"/local_costmap/clear_entirely_local_costmap",
                             "/global_costmap/clear_entirely_global_costmap"}) {
      clear_clients_.push_back(node.create_client<nav2_msgs::srv::ClearEntireCostmap>(name));
    }
  }
  bool ready() const
  {
    for (const auto & endpoint : endpoints_) if (!endpoint->client->service_is_ready()) return false;
    for (const auto & client : clear_clients_) if (!client->service_is_ready()) return false;
    return true;
  }
  void start()
  {
    started_ = std::chrono::steady_clock::now(); state_ = State::CANCELLING;
    for (auto & endpoint : endpoints_) {
      endpoint->answered = false; endpoint->had_goals = false;
      // Zero UUID and timestamp request cancellation of every current goal.
      endpoint->future = endpoint->client->async_send_request(std::make_shared<action_msgs::srv::CancelGoal::Request>()).share();
    }
  }
  void clear_after_commit()
  {
    if (state_ != State::QUIESCENT) throw std::logic_error("Clear requires terminal cancellation");
    clear_futures_.clear(); state_ = State::CLEARING;
    for (const auto & client : clear_clients_) {
      clear_futures_.push_back(client->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>()).share());
    }
  }
  State poll()
  {
    if (state_ != State::CANCELLING && state_ != State::CLEARING) return state_;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count() > 5.0) {
      state_ = State::FAILED; return state_;
    }
    try {
      for (auto & endpoint : endpoints_) {
        if (!endpoint->answered) {
          if (endpoint->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return state_;
          const auto response = endpoint->future.get();
          if (response->return_code != response->ERROR_NONE) { state_ = State::FAILED; return state_; }
          endpoint->had_goals = !response->goals_canceling.empty(); endpoint->answered = true;
        }
        if (endpoint->active || (endpoint->had_goals && endpoint->observed < started_)) return state_;
      }
      if (state_ == State::CANCELLING) { state_ = State::QUIESCENT; return state_; }
      for (const auto & future : clear_futures_) {
        if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return state_;
        future.get();
      }
      state_ = State::DONE;
    } catch (const std::exception &) { state_ = State::FAILED; }
    return state_;
  }
private:
  struct Endpoint {
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr client;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future;
    std::chrono::steady_clock::time_point observed{};
    bool active{false}, answered{false}, had_goals{false};
  };
  std::vector<std::shared_ptr<Endpoint>> endpoints_;
  std::vector<rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clear_clients_;
  std::vector<rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture> clear_futures_;
  std::chrono::steady_clock::time_point started_{};
  State state_{State::IDLE};
};
}  // namespace rm_nav_localization
