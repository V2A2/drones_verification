#include "collision_avoidance_behavior/go_to_collision_avoidance_behavior.hpp"

GoToCollisionAvoidanceBehavior::GoToCollisionAvoidanceBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<GoToAction>("GoToCollisionAvoidanceBehavior", options)
{
  declare_parameter<std::string>("plugin_name", "go_to_plugin_position");
  declare_parameter<double>("go_to_speed");
  declare_parameter<double>("go_to_threshold");

  loader_ = std::make_shared<pluginlib::ClassLoader<go_to_base::GoToBase>>(
    "as2_behaviors_motion", "go_to_base::GoToBase");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  try {
    std::string plugin_name = get_parameter("plugin_name").as_string() + "::Plugin";
    go_to_plugin_ = loader_->createSharedInstance(plugin_name);

    go_to_base::go_to_plugin_params params;
    params.go_to_speed = get_parameter("go_to_speed").as_double();
    params.go_to_threshold = get_parameter("go_to_threshold").as_double();
    go_to_plugin_->initialize(this, tf_handler_, params);

    RCLCPP_INFO(get_logger(), "GO TO COLLISION AVOIDANCE PLUGIN LOADED: %s", plugin_name.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "The plugin failed to load. Error: %s", ex.what());
    throw;
  }

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  platform_info_sub_ = create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&GoToCollisionAvoidanceBehavior::platform_info_callback, this, std::placeholders::_1));

  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&GoToCollisionAvoidanceBehavior::state_callback, this, std::placeholders::_1));

  // Published by collision_checker: SystemID of the agent whose reachable set intersects ours
  collision_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "collision_detected", 10,
    std::bind(&GoToCollisionAvoidanceBehavior::collision_callback, this, std::placeholders::_1));
}

void GoToCollisionAvoidanceBehavior::state_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  try {
    auto [pose_msg, twist_msg] =
      tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
    go_to_plugin_->state_callback(pose_msg, twist_msg);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not get transform: %s", ex.what());
  }
}

void GoToCollisionAvoidanceBehavior::platform_info_callback(
  const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  go_to_plugin_->platform_info_callback(msg);
}

void GoToCollisionAvoidanceBehavior::collision_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  colliding_agent_ = msg->data;
  collision_detected_ = true;
}

bool GoToCollisionAvoidanceBehavior::process_goal(
  std::shared_ptr<const GoToAction::Goal> goal, GoToAction::Goal & new_goal)
{
  if (goal->target_pose.header.frame_id == "") {
    RCLCPP_ERROR(get_logger(), "Target pose frame_id is empty");
    return false;
  }

  if ((fabs(new_goal.target_pose.point.x) + fabs(new_goal.target_pose.point.y) +
    fabs(new_goal.target_pose.point.z)) == 0.0f)
  {
    RCLCPP_WARN(get_logger(), "GoToCollisionAvoidance: Target point is zero");
  } else if (new_goal.target_pose.point.z <= 0.0f) {
    RCLCPP_WARN(get_logger(), "GoToCollisionAvoidance: Target height is below 0.0");
  }

  if (!tf_handler_->tryConvert(new_goal.target_pose, "earth")) {
    RCLCPP_ERROR(get_logger(), "GoToCollisionAvoidance: can not get target position in earth frame");
    return false;
  }

  geometry_msgs::msg::QuaternionStamped q;
  q.header = goal->target_pose.header;
  as2::frame::eulerToQuaternion(0.0f, 0.0f, new_goal.yaw.angle, q.quaternion);

  if (!tf_handler_->tryConvert(q, "earth")) {
    RCLCPP_ERROR(
      get_logger(), "GoToCollisionAvoidance: can not get target orientation in earth frame");
    return false;
  }

  new_goal.yaw.angle = as2::frame::getYawFromQuaternion(q.quaternion);

  new_goal.max_speed =
    (goal->max_speed != 0.0f) ? goal->max_speed : get_parameter("go_to_speed").as_double();

  return true;
}

bool GoToCollisionAvoidanceBehavior::on_activate(std::shared_ptr<const GoToAction::Goal> goal)
{
  GoToAction::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  // Only collisions reported after activation fail the goal
  collision_detected_ = false;
  return go_to_plugin_->on_activate(std::make_shared<const GoToAction::Goal>(new_goal));
}

bool GoToCollisionAvoidanceBehavior::on_modify(std::shared_ptr<const GoToAction::Goal> goal)
{
  GoToAction::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  collision_detected_ = false;
  return go_to_plugin_->on_modify(std::make_shared<const GoToAction::Goal>(new_goal));
}

bool GoToCollisionAvoidanceBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return go_to_plugin_->on_deactivate(message);
}

bool GoToCollisionAvoidanceBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return go_to_plugin_->on_pause(message);
}

bool GoToCollisionAvoidanceBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return go_to_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus GoToCollisionAvoidanceBehavior::on_run(
  const std::shared_ptr<const GoToAction::Goal> & goal,
  std::shared_ptr<GoToAction::Feedback> & feedback_msg,
  std::shared_ptr<GoToAction::Result> & result_msg)
{
  if (collision_detected_) {
    RCLCPP_WARN(
      get_logger(), "Collision with agent %d detected, aborting GoTo and holding position",
      colliding_agent_);
    feedback_msg = std::make_shared<GoToAction::Feedback>();
    result_msg = std::make_shared<GoToAction::Result>();
    result_msg->go_to_success = false;
    // on_execution_end() is called with FAILURE: the GoTo plugins hover on any non-success end
    return as2_behavior::ExecutionStatus::FAILURE;
  }
  return go_to_plugin_->on_run(goal, feedback_msg, result_msg);
}

void GoToCollisionAvoidanceBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  go_to_plugin_->on_execution_end(state);
}
