#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
using PoseStamped = geometry_msgs::msg::PoseStamped;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Clock = std::chrono::steady_clock;

std::array<double, 3> vector_to_array(
  const std::vector<double> & values,
  const std::array<double, 3> & fallback,
  const rclcpp::Logger & logger,
  const char * parameter_name)
{
  if (values.size() == 3) {
    return {values[0], values[1], values[2]};
  }

  RCLCPP_WARN(
    logger,
    "Parameter '%s' must contain exactly three values; using defaults",
    parameter_name);
  return fallback;
}

double quaternion_angular_distance(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  const double dot = std::abs((a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w));
  const double clamped_dot = std::clamp(dot, 0.0, 1.0);
  return 2.0 * std::acos(clamped_dot);
}

double position_distance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

bool normalize_quaternion(geometry_msgs::msg::Quaternion & q)
{
  const double norm_squared = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
  if (norm_squared < 1e-12) {
    return false;
  }

  const double norm = std::sqrt(norm_squared);
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return true;
}

template<typename T>
T declare_or_get_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }

  T value = default_value;
  if (!node.get_parameter(name, value)) {
    RCLCPP_WARN(
      node.get_logger(),
      "Parameter '%s' could not be read; using default",
      name.c_str());
  }
  return value;
}
}  // namespace

class ViveMoveItServer : public rclcpp::Node
{
public:
  explicit ViveMoveItServer(const rclcpp::NodeOptions & options)
  : Node("vive_moveit_server", options)
  {
    arm_group_ = declare_or_get_parameter<std::string>(*this, "arm_group", "arm_torso");
    end_effector_link_ =
      declare_or_get_parameter<std::string>(*this, "end_effector_link", "");

    const auto head_input_topic =
      declare_or_get_parameter<std::string>(*this, "head_input_topic", "/vive/head_pose");
    const auto head_output_topic =
      declare_or_get_parameter<std::string>(
      *this,
      "head_output_topic",
      "/vive/robot_head_pose");
    const auto hand_target_topic =
      declare_or_get_parameter<std::string>(
      *this,
      "hand_target_topic",
      "/vive/hand_target_pose");
    const auto arm_command_topic =
      declare_or_get_parameter<std::string>(
      *this,
      "arm_command_topic",
      "/arm_controller/command");

    execution_mode_ =
      declare_or_get_parameter<std::string>(*this, "execution_mode", "moveit");
    async_execution_ = declare_or_get_parameter<bool>(*this, "async_execution", true);
    pose_reference_frame_ =
      declare_or_get_parameter<std::string>(*this, "pose_reference_frame", "");
    planning_time_sec_ =
      declare_or_get_parameter<double>(*this, "planning_time_sec", 0.35);
    max_velocity_scaling_factor_ =
      declare_or_get_parameter<double>(*this, "max_velocity_scaling_factor", 0.25);
    max_acceleration_scaling_factor_ =
      declare_or_get_parameter<double>(*this, "max_acceleration_scaling_factor", 0.25);
    min_plan_interval_sec_ =
      declare_or_get_parameter<double>(*this, "min_plan_interval_sec", 0.25);
    position_deadband_m_ =
      declare_or_get_parameter<double>(*this, "position_deadband_m", 0.01);
    orientation_deadband_rad_ =
      declare_or_get_parameter<double>(*this, "orientation_deadband_rad", 0.035);

    hand_position_scale_ = vector_to_array(
      declare_or_get_parameter<std::vector<double>>(
        *this,
        "hand_position_scale",
        std::vector<double>{1.0, 1.0, 1.0}),
      {1.0, 1.0, 1.0},
      get_logger(),
      "hand_position_scale");
    hand_position_offset_ = vector_to_array(
      declare_or_get_parameter<std::vector<double>>(
        *this,
        "hand_position_offset",
        std::vector<double>{0.0, 0.0, 0.0}),
      {0.0, 0.0, 0.0},
      get_logger(),
      "hand_position_offset");

    head_publisher_ = create_publisher<PoseStamped>(head_output_topic, 10);
    trajectory_publisher_ = create_publisher<JointTrajectory>(arm_command_topic, 10);

    head_subscription_ = create_subscription<PoseStamped>(
      head_input_topic,
      10,
      [this](const PoseStamped::SharedPtr msg) {
        head_publisher_->publish(*msg);
      });

    hand_subscription_ = create_subscription<PoseStamped>(
      hand_target_topic,
      10,
      [this](const PoseStamped::SharedPtr msg) {
        pending_hand_target_ = *msg;
      });

    plan_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ViveMoveItServer::maybe_plan_to_latest_target, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening for head poses on '%s' and hand targets on '%s'",
      head_input_topic.c_str(),
      hand_target_topic.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Head poses are forwarded unchanged to '%s'",
      head_output_topic.c_str());
  }

  bool configure_move_group()
  {
    if (move_group_) {
      return true;
    }

    try {
      move_group_ = std::make_unique<MoveGroupInterface>(shared_from_this(), arm_group_);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "MoveIt group '%s' is not ready yet: %s",
        arm_group_.c_str(),
        error.what());
      return false;
    }

    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
    if (!end_effector_link_.empty()) {
      move_group_->setEndEffectorLink(end_effector_link_);
    }
    if (!pose_reference_frame_.empty()) {
      move_group_->setPoseReferenceFrame(pose_reference_frame_);
    }

    RCLCPP_INFO(
      get_logger(),
      "MoveIt group '%s' ready; planning frame='%s', end effector='%s', mode='%s'",
      arm_group_.c_str(),
      move_group_->getPlanningFrame().c_str(),
      move_group_->getEndEffectorLink().c_str(),
      execution_mode_.c_str());
    return true;
  }

private:
  void maybe_plan_to_latest_target()
  {
    if (planning_ || !pending_hand_target_) {
      return;
    }
    if (!configure_move_group()) {
      return;
    }

    const auto now = Clock::now();
    if (
      last_plan_started_ != Clock::time_point{} &&
      std::chrono::duration<double>(now - last_plan_started_).count() <
        min_plan_interval_sec_)
    {
      return;
    }

    PoseStamped target = apply_hand_target_adjustments(*pending_hand_target_);
    if (!target.header.frame_id.empty() && !pose_reference_frame_.empty()) {
      target.header.frame_id = pose_reference_frame_;
    }
    if (target.header.frame_id.empty()) {
      target.header.frame_id = move_group_->getPlanningFrame();
    }

    if (!normalize_quaternion(target.pose.orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring hand target with invalid orientation quaternion");
      pending_hand_target_.reset();
      return;
    }

    if (is_inside_deadband(target)) {
      pending_hand_target_.reset();
      return;
    }

    planning_ = true;
    last_plan_started_ = now;
    try {
      plan_and_execute(target);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "MoveIt planning/execution error: %s",
        error.what());
      if (move_group_) {
        move_group_->clearPoseTargets();
      }
      pending_hand_target_.reset();
    }
    planning_ = false;
  }

  PoseStamped apply_hand_target_adjustments(const PoseStamped & input) const
  {
    PoseStamped target = input;
    target.pose.position.x =
      (target.pose.position.x * hand_position_scale_[0]) + hand_position_offset_[0];
    target.pose.position.y =
      (target.pose.position.y * hand_position_scale_[1]) + hand_position_offset_[1];
    target.pose.position.z =
      (target.pose.position.z * hand_position_scale_[2]) + hand_position_offset_[2];
    return target;
  }

  bool is_inside_deadband(const PoseStamped & target) const
  {
    if (!last_commanded_target_) {
      return false;
    }

    const double linear_delta =
      position_distance(target.pose.position, last_commanded_target_->pose.position);
    const double angular_delta =
      quaternion_angular_distance(
        target.pose.orientation,
        last_commanded_target_->pose.orientation);

    return linear_delta < position_deadband_m_ &&
      angular_delta < orientation_deadband_rad_;
  }

  void plan_and_execute(const PoseStamped & target)
  {
    move_group_->setStartStateToCurrentState();

    const bool accepted = end_effector_link_.empty() ?
      move_group_->setPoseTarget(target) :
      move_group_->setPoseTarget(target, end_effector_link_);

    if (!accepted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "MoveIt rejected the hand target pose");
      pending_hand_target_.reset();
      move_group_->clearPoseTargets();
      return;
    }

    MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "MoveIt could not plan to the latest hand target");
      pending_hand_target_.reset();
      move_group_->clearPoseTargets();
      return;
    }

    bool command_accepted = false;
    if (execution_mode_ == "plan_only") {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "MoveIt plan succeeded; execution_mode=plan_only");
      command_accepted = true;
    } else if (execution_mode_ == "trajectory_topic") {
      trajectory_publisher_->publish(plan.trajectory_.joint_trajectory);
      command_accepted = true;
    } else if (execution_mode_ == "moveit") {
      const bool executed = async_execution_ ?
        static_cast<bool>(move_group_->asyncExecute(plan)) :
        static_cast<bool>(move_group_->execute(plan));
      if (!executed) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "MoveIt failed to execute the planned hand trajectory");
      } else {
        command_accepted = true;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unknown execution_mode '%s'; plan was not sent",
        execution_mode_.c_str());
    }

    if (command_accepted) {
      last_commanded_target_ = target;
    }
    pending_hand_target_.reset();
    move_group_->clearPoseTargets();
  }

  std::string arm_group_;
  std::string end_effector_link_;
  std::string execution_mode_;
  std::string pose_reference_frame_;
  bool async_execution_{true};
  double planning_time_sec_{0.35};
  double max_velocity_scaling_factor_{0.25};
  double max_acceleration_scaling_factor_{0.25};
  double min_plan_interval_sec_{0.25};
  double position_deadband_m_{0.01};
  double orientation_deadband_rad_{0.035};
  std::array<double, 3> hand_position_scale_{1.0, 1.0, 1.0};
  std::array<double, 3> hand_position_offset_{0.0, 0.0, 0.0};

  rclcpp::Subscription<PoseStamped>::SharedPtr head_subscription_;
  rclcpp::Subscription<PoseStamped>::SharedPtr hand_subscription_;
  rclcpp::Publisher<PoseStamped>::SharedPtr head_publisher_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  std::unique_ptr<MoveGroupInterface> move_group_;

  std::optional<PoseStamped> pending_hand_target_;
  std::optional<PoseStamped> last_commanded_target_;
  Clock::time_point last_plan_started_{};
  bool planning_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto node = std::make_shared<ViveMoveItServer>(options);

  node->configure_move_group();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
