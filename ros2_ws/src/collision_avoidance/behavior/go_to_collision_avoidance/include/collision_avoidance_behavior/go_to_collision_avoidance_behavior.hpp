#ifndef VERIFIER__GO_TO_COLLISION_AVOIDANCE_BEHAVIOR_HPP_
#define VERIFIER__GO_TO_COLLISION_AVOIDANCE_BEHAVIOR_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "go_to_behavior/go_to_base.hpp"

// GoTo behavior that fails and holds in place (hover) when collision_checker reports a
// collision while the goal is being executed. The motion itself is done by the stock GoTo
// plugins of as2_behaviors_motion.
class GoToCollisionAvoidanceBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::GoToWaypoint>
{
public:
  using GoToAction = as2_msgs::action::GoToWaypoint;

  explicit GoToCollisionAvoidanceBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);
  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);
  void collision_callback(const std_msgs::msg::UInt8::SharedPtr msg);

  bool process_goal(
    std::shared_ptr<const GoToAction::Goal> goal, GoToAction::Goal & new_goal);

  bool on_activate(std::shared_ptr<const GoToAction::Goal> goal) override;
  bool on_modify(std::shared_ptr<const GoToAction::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const GoToAction::Goal> & goal,
    std::shared_ptr<GoToAction::Feedback> & feedback_msg,
    std::shared_ptr<GoToAction::Result> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<go_to_base::GoToBase>> loader_;
  std::shared_ptr<go_to_base::GoToBase> go_to_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr collision_sub_;

  bool collision_detected_ = false;
  uint8_t colliding_agent_ = 0;
};

#endif  // VERIFIER__GO_TO_COLLISION_AVOIDANCE_BEHAVIOR_HPP_
