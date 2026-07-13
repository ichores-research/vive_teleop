#include "vive_moveit_server/control_math.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vive_moveit_server
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

bool hasTime(const SteadyTime& time)
{
  return time.time_since_epoch().count() != 0;
}

double secondsBetween(const SteadyTime& newer, const SteadyTime& older)
{
  return std::chrono::duration<double>(newer - older).count();
}

std::chrono::nanoseconds timerPeriod(double rate_hz)
{
  const double safe_rate = std::max(1.0, rate_hz);
  return std::chrono::nanoseconds(
    static_cast<std::int64_t>(std::llround(1.0e9 / safe_rate)));
}

CartesianPose poseFromMessage(const geometry_msgs::msg::Pose& message)
{
  CartesianPose pose;
  pose.position = Eigen::Vector3d(
    message.position.x, message.position.y, message.position.z);
  pose.orientation = Eigen::Quaterniond(
    message.orientation.w,
    message.orientation.x,
    message.orientation.y,
    message.orientation.z);
  return pose;
}

geometry_msgs::msg::Pose poseToMessage(const CartesianPose& pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.position.x();
  message.position.y = pose.position.y();
  message.position.z = pose.position.z();
  message.orientation.x = pose.orientation.x();
  message.orientation.y = pose.orientation.y();
  message.orientation.z = pose.orientation.z();
  message.orientation.w = pose.orientation.w();
  return message;
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

struct RealtimeResult
{
  bool scheduling_active{ false };
  bool memory_lock_active{ false };
  bool affinity_active{ false };
};

}  // namespace

class ViveMoveItServer final : public rclcpp::Node
{
public:
  ViveMoveItServer()
  : Node("vive_moveit_server"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this, false)
  {
    declareAndLoadParameters();
    createRosInterfaces();

    arm_control_timer_ = create_wall_timer(
      timerPeriod(arm_control_rate_hz_),
      std::bind(&ViveMoveItServer::updateArmControl, this));
    head_timer_ = create_wall_timer(
      timerPeriod(head_publish_rate_hz_),
      std::bind(&ViveMoveItServer::updateHeadControl, this));
    base_timer_ = create_wall_timer(
      timerPeriod(base_publish_rate_hz_),
      std::bind(&ViveMoveItServer::updateBaseControl, this));

    RCLCPP_INFO(
      get_logger(),
      "C++ teleop ready: 6-DoF arm loop %.1f Hz, Servo group '%s', tool '%s'",
      arm_control_rate_hz_, arm_group_.c_str(), end_effector_link_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Controller top frame is clutch-mapped to the tool frame; alignment RPY "
      "is [%.3f, %.3f, %.3f] rad",
      controller_to_tool_rpy_.x(),
      controller_to_tool_rpy_.y(),
      controller_to_tool_rpy_.z());
  }

  ~ViveMoveItServer() override
  {
    stopAllMotion();
  }

  bool realtimeSchedulingRequested() const
  {
    return enable_realtime_scheduling_;
  }

  bool memoryLockRequested() const
  {
    return lock_memory_;
  }

  int realtimePriority() const
  {
    return realtime_priority_;
  }

  int cpuAffinity() const
  {
    return cpu_affinity_;
  }

  void setRealtimeStatus(const RealtimeResult& result)
  {
    set_parameter(rclcpp::Parameter(
      "realtime_scheduling_active", result.scheduling_active));
    set_parameter(rclcpp::Parameter(
      "memory_lock_active", result.memory_lock_active));
    set_parameter(rclcpp::Parameter(
      "cpu_affinity_active", result.affinity_active));
  }

  void stopAllMotion()
  {
    if (shutdown_halt_sent_.exchange(true)) {
      return;
    }
    haltArm(false, "shutdown");
    haltBaseImmediately();
  }

private:
  Eigen::Vector3d declareVector3(
    const std::string& name,
    const std::array<double, 3>& defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, std::vector<double>(defaults.begin(), defaults.end()));
    if (values.size() != 3 ||
        !std::all_of(values.begin(), values.end(), [](double value) {
          return std::isfinite(value);
        })) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter '%s' must contain three finite values; using defaults",
        name.c_str());
      return Eigen::Vector3d(defaults[0], defaults[1], defaults[2]);
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  std::array<double, 2> declarePair(
    const std::string& name,
    const std::array<double, 2>& defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, std::vector<double>(defaults.begin(), defaults.end()));
    if (values.size() != 2 || !std::isfinite(values[0]) ||
        !std::isfinite(values[1])) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter '%s' must contain two finite values; using defaults",
        name.c_str());
      return defaults;
    }
    return { values[0], values[1] };
  }

  double finiteOrDefault(
    const char* name,
    double value,
    double fallback)
  {
    if (std::isfinite(value)) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "Parameter '%s' is not finite; using %.6f",
      name, fallback);
    return fallback;
  }

  double positiveOrDefault(
    const char* name,
    double value,
    double fallback)
  {
    if (std::isfinite(value) && value > 0.0) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "Parameter '%s' must be positive; using %.6f",
      name, fallback);
    return fallback;
  }

  double nonnegativeOrDefault(
    const char* name,
    double value,
    double fallback)
  {
    if (std::isfinite(value) && value >= 0.0) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "Parameter '%s' must be nonnegative; using %.6f",
      name, fallback);
    return fallback;
  }

  void declareAndLoadParameters()
  {
    arm_group_ = declare_parameter<std::string>("arm_group", "arm");
    end_effector_link_ = declare_parameter<std::string>(
      "end_effector_link", "arm_tool_link");
    planning_frame_ = declare_parameter<std::string>(
      "pose_reference_frame", "base_footprint");

    head_input_topic_ = declare_parameter<std::string>(
      "head_input_topic", "/vive/head_pose");
    head_command_topic_ = declare_parameter<std::string>(
      "head_command_topic", "/head_controller/joint_trajectory");
    hand_target_topic_ = declare_parameter<std::string>(
      "hand_target_topic", "/vive/hand_target_pose");
    hand_target_active_topic_ = declare_parameter<std::string>(
      "hand_target_active_topic", "/vive/hand_target_active");
    gripper_input_topic_ = declare_parameter<std::string>(
      "gripper_input_topic", "/vive/gripper_opening");
    gripper_command_topic_ = declare_parameter<std::string>(
      "gripper_command_topic", "/gripper_controller/joint_trajectory");
    base_input_topic_ = declare_parameter<std::string>(
      "base_input_topic", "/vive/base_command");
    base_active_topic_ = declare_parameter<std::string>(
      "base_active_topic", "/vive/base_active");
    base_command_topic_ = declare_parameter<std::string>(
      "base_command_topic", "/key_vel");
    base_command_frame_ = declare_parameter<std::string>(
      "base_command_frame", "base_footprint");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");

    servo_twist_topic_ = declare_parameter<std::string>(
      "servo_twist_topic", "/servo_node/delta_twist_cmds");
    servo_pose_target_topic_ = declare_parameter<std::string>(
      "servo_pose_target_topic", "/servo_node/pose_target_cmds");
    servo_pose_active_topic_ = declare_parameter<std::string>(
      "servo_pose_active_topic", "/servo_node/pose_target_active");
    servo_start_service_ = declare_parameter<std::string>(
      "servo_start_service", "/servo_node/start_servo");

    arm_control_rate_hz_ = declare_parameter<double>(
      "arm_control_rate_hz", 100.0);
    hand_target_timeout_sec_ = declare_parameter<double>(
      "hand_target_timeout_sec", 0.12);
    robot_state_timeout_sec_ = declare_parameter<double>(
      "robot_state_timeout_sec", 0.20);
    require_deadman_repress_after_timeout_ = declare_parameter<bool>(
      "require_deadman_repress_after_timeout", true);
    arm_halt_command_count_ = declare_parameter<int>(
      "arm_halt_command_count", 4);

    max_hand_target_distance_m_ = declare_parameter<double>(
      "max_hand_target_distance_m", 1.5);
    min_hand_target_z_m_ = declare_parameter<double>(
      "min_hand_target_z_m", 0.2);
    max_hand_target_z_m_ = declare_parameter<double>(
      "max_hand_target_z_m", 1.6);
    hand_position_scale_ = declareVector3(
      "hand_position_scale", { 1.0, 1.0, 1.0 });
    controller_top_offset_ = declareVector3(
      "controller_top_offset_m", { 0.0, 0.0, 0.0 });
    controller_to_tool_rpy_ = declareVector3(
      "controller_to_tool_rotation_rpy_rad", { 0.0, 0.0, 0.0 });
    controller_to_tool_rotation_ = quaternionFromRpy(controller_to_tool_rpy_);

    linear_gain_ = declare_parameter<double>("linear_gain", 5.0);
    angular_gain_ = declare_parameter<double>("angular_gain", 2.5);
    max_linear_velocity_mps_ = declare_parameter<double>(
      "max_linear_velocity_mps", 0.35);
    max_angular_velocity_radps_ = declare_parameter<double>(
      "max_angular_velocity_radps", 1.20);
    max_linear_acceleration_mps2_ = declare_parameter<double>(
      "max_linear_acceleration_mps2", 2.0);
    max_angular_acceleration_radps2_ = declare_parameter<double>(
      "max_angular_acceleration_radps2", 6.0);
    position_deadband_m_ = declare_parameter<double>(
      "position_deadband_m", 0.0005);
    orientation_deadband_rad_ = declare_parameter<double>(
      "orientation_deadband_rad", 0.003);
    target_filter_cutoff_hz_ = declare_parameter<double>(
      "target_filter_cutoff_hz", 12.0);
    linear_feedforward_gain_ = declare_parameter<double>(
      "linear_feedforward_gain", 1.0);
    angular_feedforward_gain_ = declare_parameter<double>(
      "angular_feedforward_gain", 1.0);
    feedforward_filter_alpha_ = declare_parameter<double>(
      "feedforward_filter_alpha", 0.35);
    feedforward_timeout_sec_ = declare_parameter<double>(
      "feedforward_timeout_sec", 0.05);
    feedforward_reset_gap_sec_ = declare_parameter<double>(
      "feedforward_reset_gap_sec", 0.12);
    linear_feedforward_stop_velocity_mps_ = declare_parameter<double>(
      "linear_feedforward_stop_velocity_mps", 0.005);
    angular_feedforward_stop_velocity_radps_ = declare_parameter<double>(
      "angular_feedforward_stop_velocity_radps", 0.02);

    head_joint_names_ = declare_parameter<std::vector<std::string>>(
      "head_joint_names", { "head_1_joint", "head_2_joint" });
    if (head_joint_names_.size() != 2) {
      RCLCPP_WARN(
        get_logger(),
        "head_joint_names must contain pan and tilt; using TIAGo defaults");
      head_joint_names_ = { "head_1_joint", "head_2_joint" };
    }
    head_publish_rate_hz_ = declare_parameter<double>(
      "head_publish_rate_hz", 20.0);
    head_command_duration_sec_ = declare_parameter<double>(
      "head_command_duration_sec", 0.10);
    head_deadband_rad_ = declare_parameter<double>(
      "head_deadband_rad", 0.002);
    head_limit_scale_ = declare_parameter<double>("head_limit_scale", 0.9);
    head_pan_sign_ = declare_parameter<double>("head_pan_sign", -1.0);
    head_tilt_sign_ = declare_parameter<double>("head_tilt_sign", -1.0);
    head_pan_limits_rad_ = declarePair(
      "head_pan_limits_rad", { -1.24, 1.24 });
    head_tilt_limits_rad_ = declarePair(
      "head_tilt_limits_rad", { -0.98, 0.72 });

    gripper_joint_names_ = declare_parameter<std::vector<std::string>>(
      "gripper_joint_names",
      { "gripper_right_finger_joint", "gripper_left_finger_joint" });
    gripper_min_position_m_ = declare_parameter<double>(
      "gripper_min_position_m", 0.0);
    gripper_max_position_m_ = declare_parameter<double>(
      "gripper_max_position_m", 0.045);
    gripper_deadband_m_ = declare_parameter<double>(
      "gripper_deadband_m", 0.0005);
    gripper_command_duration_sec_ = declare_parameter<double>(
      "gripper_command_duration_sec", 0.15);
    gripper_max_velocity_mps_ = declare_parameter<double>(
      "gripper_max_velocity_mps", 0.04);
    joint_state_timeout_sec_ = declare_parameter<double>(
      "joint_state_timeout_sec", 0.25);

    base_publish_rate_hz_ = declare_parameter<double>(
      "base_publish_rate_hz", 30.0);
    base_input_timeout_sec_ = declare_parameter<double>(
      "base_input_timeout_sec", 0.15);
    base_halt_command_count_ = declare_parameter<int>(
      "base_halt_command_count", 3);
    base_max_linear_velocity_mps_ = declare_parameter<double>(
      "base_max_linear_velocity_mps", 0.25);
    base_max_angular_velocity_radps_ = declare_parameter<double>(
      "base_max_angular_velocity_radps", 0.6);
    base_max_linear_acceleration_mps2_ = declare_parameter<double>(
      "base_max_linear_acceleration_mps2", 0.5);
    base_max_angular_acceleration_radps2_ = declare_parameter<double>(
      "base_max_angular_acceleration_radps2", 1.2);
    base_max_linear_deceleration_mps2_ = declare_parameter<double>(
      "base_max_linear_deceleration_mps2", 1.0);
    base_max_angular_deceleration_radps2_ = declare_parameter<double>(
      "base_max_angular_deceleration_radps2", 2.4);

    enable_realtime_scheduling_ = declare_parameter<bool>(
      "enable_realtime_scheduling", true);
    realtime_priority_ = declare_parameter<int>("realtime_priority", 40);
    lock_memory_ = declare_parameter<bool>("lock_memory", true);
    cpu_affinity_ = declare_parameter<int>("cpu_affinity", -1);
    declare_parameter<bool>("realtime_scheduling_active", false);
    declare_parameter<bool>("memory_lock_active", false);
    declare_parameter<bool>("cpu_affinity_active", false);

    arm_control_rate_hz_ = std::max(
      20.0,
      positiveOrDefault("arm_control_rate_hz", arm_control_rate_hz_, 100.0));
    hand_target_timeout_sec_ = positiveOrDefault(
      "hand_target_timeout_sec", hand_target_timeout_sec_, 0.12);
    robot_state_timeout_sec_ = positiveOrDefault(
      "robot_state_timeout_sec", robot_state_timeout_sec_, 0.20);
    max_hand_target_distance_m_ = positiveOrDefault(
      "max_hand_target_distance_m", max_hand_target_distance_m_, 1.5);
    min_hand_target_z_m_ = finiteOrDefault(
      "min_hand_target_z_m", min_hand_target_z_m_, 0.2);
    max_hand_target_z_m_ = finiteOrDefault(
      "max_hand_target_z_m", max_hand_target_z_m_, 1.6);
    if (min_hand_target_z_m_ >= max_hand_target_z_m_ ||
        min_hand_target_z_m_ > max_hand_target_distance_m_ ||
        max_hand_target_z_m_ < -max_hand_target_distance_m_) {
      RCLCPP_WARN(
        get_logger(),
        "Cartesian Z limits do not intersect the spherical workspace; "
        "using the default 1.5 m sphere and [0.2, 1.6] m Z limits");
      max_hand_target_distance_m_ = 1.5;
      min_hand_target_z_m_ = 0.2;
      max_hand_target_z_m_ = 1.6;
    }
    linear_gain_ = nonnegativeOrDefault("linear_gain", linear_gain_, 5.0);
    angular_gain_ = nonnegativeOrDefault("angular_gain", angular_gain_, 2.5);
    max_linear_velocity_mps_ = positiveOrDefault(
      "max_linear_velocity_mps", max_linear_velocity_mps_, 0.35);
    max_angular_velocity_radps_ = positiveOrDefault(
      "max_angular_velocity_radps", max_angular_velocity_radps_, 1.2);
    max_linear_acceleration_mps2_ = positiveOrDefault(
      "max_linear_acceleration_mps2", max_linear_acceleration_mps2_, 2.0);
    max_angular_acceleration_radps2_ = positiveOrDefault(
      "max_angular_acceleration_radps2", max_angular_acceleration_radps2_, 6.0);
    position_deadband_m_ = nonnegativeOrDefault(
      "position_deadband_m", position_deadband_m_, 0.0005);
    orientation_deadband_rad_ = nonnegativeOrDefault(
      "orientation_deadband_rad", orientation_deadband_rad_, 0.003);
    target_filter_cutoff_hz_ = nonnegativeOrDefault(
      "target_filter_cutoff_hz", target_filter_cutoff_hz_, 12.0);
    linear_feedforward_gain_ = nonnegativeOrDefault(
      "linear_feedforward_gain", linear_feedforward_gain_, 1.0);
    angular_feedforward_gain_ = nonnegativeOrDefault(
      "angular_feedforward_gain", angular_feedforward_gain_, 1.0);
    feedforward_filter_alpha_ = finiteOrDefault(
      "feedforward_filter_alpha", feedforward_filter_alpha_, 0.35);
    feedforward_timeout_sec_ = nonnegativeOrDefault(
      "feedforward_timeout_sec", feedforward_timeout_sec_, 0.05);
    feedforward_reset_gap_sec_ = nonnegativeOrDefault(
      "feedforward_reset_gap_sec", feedforward_reset_gap_sec_, 0.12);
    linear_feedforward_stop_velocity_mps_ = nonnegativeOrDefault(
      "linear_feedforward_stop_velocity_mps",
      linear_feedforward_stop_velocity_mps_, 0.005);
    angular_feedforward_stop_velocity_radps_ = nonnegativeOrDefault(
      "angular_feedforward_stop_velocity_radps",
      angular_feedforward_stop_velocity_radps_, 0.02);
    head_publish_rate_hz_ = std::max(1.0, head_publish_rate_hz_);
    base_publish_rate_hz_ = std::max(1.0, base_publish_rate_hz_);
    arm_halt_command_count_ = std::max(1, arm_halt_command_count_);
    base_halt_command_count_ = std::max(1, base_halt_command_count_);
    feedforward_filter_alpha_ = clamp(feedforward_filter_alpha_, 0.0, 1.0);
  }

  void createRosInterfaces()
  {
    const auto newest_input_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    const auto gate_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile();
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile();

    head_trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      head_command_topic_, rclcpp::QoS(10));
    gripper_trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      gripper_command_topic_, rclcpp::QoS(10));
    base_velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      base_command_topic_, rclcpp::QoS(10));
    servo_twist_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      servo_twist_topic_, command_qos);
    servo_pose_target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      servo_pose_target_topic_, rclcpp::QoS(10));
    servo_pose_active_publisher_ = create_publisher<std_msgs::msg::Bool>(
      servo_pose_active_topic_, rclcpp::QoS(10));

    head_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      head_input_topic_,
      newest_input_qos,
      std::bind(&ViveMoveItServer::onHeadPose, this, std::placeholders::_1));
    hand_target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      hand_target_topic_,
      newest_input_qos,
      std::bind(&ViveMoveItServer::onHandTarget, this, std::placeholders::_1));
    hand_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      hand_target_active_topic_,
      gate_qos,
      std::bind(&ViveMoveItServer::onHandActive, this, std::placeholders::_1));
    gripper_subscription_ = create_subscription<std_msgs::msg::Float64>(
      gripper_input_topic_,
      newest_input_qos,
      std::bind(&ViveMoveItServer::onGripperOpening, this, std::placeholders::_1));
    base_command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      base_input_topic_,
      newest_input_qos,
      std::bind(&ViveMoveItServer::onBaseCommand, this, std::placeholders::_1));
    base_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      base_active_topic_,
      gate_qos,
      std::bind(&ViveMoveItServer::onBaseActive, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ViveMoveItServer::onJointState, this, std::placeholders::_1));

    servo_start_client_ = create_client<std_srvs::srv::Trigger>(servo_start_service_);
  }

  void onHeadPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    latest_head_pose_ = *message;
    new_head_pose_ = true;
  }

  void updateHeadControl()
  {
    if (!new_head_pose_ || !latest_head_pose_) {
      return;
    }
    new_head_pose_ = false;

    CartesianPose head = poseFromMessage(latest_head_pose_->pose);
    if (!normalizeQuaternion(head.orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring HMD pose with an invalid quaternion");
      return;
    }

    const double x = head.orientation.x();
    const double y = head.orientation.y();
    const double z = head.orientation.z();
    const double w = head.orientation.w();
    double pan = head_pan_sign_ * std::atan2(
      2.0 * ((w * y) + (x * z)),
      1.0 - 2.0 * ((y * y) + (x * x)));
    const double tilt_term = clamp(2.0 * ((w * x) - (z * y)), -1.0, 1.0);
    double tilt = head_tilt_sign_ * std::asin(tilt_term);

    const double scale = clamp(head_limit_scale_, 0.0, 1.0);
    pan = clamp(
      pan,
      std::min(head_pan_limits_rad_[0], head_pan_limits_rad_[1]) * scale,
      std::max(head_pan_limits_rad_[0], head_pan_limits_rad_[1]) * scale);
    tilt = clamp(
      tilt,
      std::min(head_tilt_limits_rad_[0], head_tilt_limits_rad_[1]) * scale,
      std::max(head_tilt_limits_rad_[0], head_tilt_limits_rad_[1]) * scale);
    if (!std::isfinite(pan) || !std::isfinite(tilt)) {
      return;
    }
    if (last_head_pan_ && last_head_tilt_ &&
        std::abs(pan - *last_head_pan_) < std::max(0.0, head_deadband_rad_) &&
        std::abs(tilt - *last_head_tilt_) < std::max(0.0, head_deadband_rad_)) {
      return;
    }

    trajectory_msgs::msg::JointTrajectory command;
    command.header.stamp = now();
    command.joint_names = head_joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = { pan, tilt };
    point.time_from_start = rclcpp::Duration::from_seconds(
      std::max(0.02, head_command_duration_sec_));
    command.points.push_back(std::move(point));
    head_trajectory_publisher_->publish(command);
    last_head_pan_ = pan;
    last_head_tilt_ = tilt;
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const std::size_t count = std::min(message->name.size(), message->position.size());
    bool received_valid = false;
    for (std::size_t index = 0; index < count; ++index) {
      if (!std::isfinite(message->position[index])) {
        continue;
      }
      joint_positions_[message->name[index]] = message->position[index];
      received_valid = true;
    }
    if (received_valid) {
      last_joint_state_received_ = SteadyClock::now();
    }
  }

  void onGripperOpening(const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-finite gripper opening");
      return;
    }
    if (gripper_joint_names_.size() != 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "gripper_joint_names must contain exactly two joints");
      return;
    }
    if (!hasTime(last_joint_state_received_) ||
        secondsBetween(SteadyClock::now(), last_joint_state_received_) >
          std::max(0.02, joint_state_timeout_sec_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for fresh gripper joint state");
      return;
    }

    std::array<double, 2> current_positions{};
    for (std::size_t index = 0; index < gripper_joint_names_.size(); ++index) {
      const auto position = joint_positions_.find(gripper_joint_names_[index]);
      if (position == joint_positions_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for both gripper joint positions");
        return;
      }
      current_positions[index] = position->second;
    }

    const double lower = std::min(gripper_min_position_m_, gripper_max_position_m_);
    const double upper = std::max(gripper_min_position_m_, gripper_max_position_m_);
    const double target = lower + (upper - lower) * clamp(message->data, 0.0, 1.0);
    const double current = 0.5 * (current_positions[0] + current_positions[1]);
    const double deadband = std::max(0.0, gripper_deadband_m_);
    if ((!last_gripper_target_ && std::abs(target - current) <= deadband) ||
        (last_gripper_target_ && std::abs(target - *last_gripper_target_) <= deadband)) {
      last_gripper_target_ = target;
      return;
    }

    double duration = std::max(0.02, gripper_command_duration_sec_);
    if (gripper_max_velocity_mps_ > 1e-9) {
      duration = std::max(
        duration, std::abs(target - current) / gripper_max_velocity_mps_);
    }
    if (!std::isfinite(duration)) {
      return;
    }

    trajectory_msgs::msg::JointTrajectory command;
    command.header.stamp = now();
    command.joint_names = gripper_joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = { target, target };
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    command.points.push_back(std::move(point));
    gripper_trajectory_publisher_->publish(command);
    last_gripper_target_ = target;
  }

  void onBaseActive(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool requested = message->data;
    const bool rising_edge = requested && !base_active_input_;
    if (!requested) {
      base_rearm_required_ = false;
      base_command_enabled_ = false;
      pending_base_command_.reset();
      if (base_active_input_ || std::abs(last_base_linear_) > 0.0 ||
          std::abs(last_base_angular_) > 0.0) {
        haltBaseImmediately();
      }
    } else if (rising_edge && !base_rearm_required_) {
      base_command_enabled_ = true;
      pending_base_command_.reset();
      base_enabled_time_ = SteadyClock::now();
    }
    base_active_input_ = requested;
  }

  void onBaseCommand(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    if (!base_command_enabled_) {
      return;
    }
    if (!message->header.frame_id.empty() &&
        message->header.frame_id != base_command_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring base command in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), base_command_frame_.c_str());
      return;
    }
    const double linear = message->twist.linear.x;
    const double angular = message->twist.angular.z;
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      return;
    }
    pending_base_command_ = std::make_pair(
      clamp(
        linear,
        -std::max(0.0, base_max_linear_velocity_mps_),
        std::max(0.0, base_max_linear_velocity_mps_)),
      clamp(
        angular,
        -std::max(0.0, base_max_angular_velocity_radps_),
        std::max(0.0, base_max_angular_velocity_radps_)));
    last_base_command_received_ = SteadyClock::now();
  }

  void updateBaseControl()
  {
    const SteadyTime current_time = SteadyClock::now();
    if (!base_command_enabled_) {
      publishRemainingBaseHalts();
      return;
    }

    const double timeout = std::max(0.02, base_input_timeout_sec_);
    std::pair<double, double> target{ 0.0, 0.0 };
    if (pending_base_command_) {
      if (secondsBetween(current_time, last_base_command_received_) > timeout) {
        base_command_enabled_ = false;
        base_rearm_required_ = true;
        pending_base_command_.reset();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Base input timed out; release and press the click gate to rearm");
        haltBaseImmediately();
        return;
      }
      target = *pending_base_command_;
    } else if (!hasTime(base_enabled_time_) ||
               secondsBetween(current_time, base_enabled_time_) > timeout) {
      base_command_enabled_ = false;
      base_rearm_required_ = true;
      haltBaseImmediately();
      return;
    }

    const double dt = hasTime(last_base_update_)
      ? clamp(secondsBetween(current_time, last_base_update_), 0.0, 0.1)
      : 1.0 / base_publish_rate_hz_;
    last_base_update_ = current_time;
    const double linear = approachVelocity(
      last_base_linear_,
      target.first,
      base_max_linear_acceleration_mps2_,
      base_max_linear_deceleration_mps2_,
      dt);
    const double angular = approachVelocity(
      last_base_angular_,
      target.second,
      base_max_angular_acceleration_radps2_,
      base_max_angular_deceleration_radps2_,
      dt);
    publishBaseVelocity(linear, angular);
  }

  void publishBaseVelocity(double linear, double angular)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    base_velocity_publisher_->publish(command);
    last_base_linear_ = linear;
    last_base_angular_ = angular;
  }

  void haltBaseImmediately()
  {
    last_base_linear_ = 0.0;
    last_base_angular_ = 0.0;
    last_base_update_ = SteadyClock::now();
    publishBaseVelocity(0.0, 0.0);
    base_halt_commands_remaining_ = std::max(0, base_halt_command_count_ - 1);
  }

  void publishRemainingBaseHalts()
  {
    if (base_halt_commands_remaining_ <= 0) {
      return;
    }
    publishBaseVelocity(0.0, 0.0);
    --base_halt_commands_remaining_;
  }

  void onHandActive(const std_msgs::msg::Bool::SharedPtr message)
  {
    const SteadyTime current_time = SteadyClock::now();
    const bool requested = message->data;
    const bool rising_edge = requested && !hand_active_input_;
    last_hand_gate_received_ = current_time;

    if (!requested) {
      hand_active_input_ = false;
      arm_rearm_required_ = false;
      arm_requested_ = false;
      hand_pose_available_ = false;
      haltArm(false, "deadman released");
      if (!effective_arm_active_) {
        publishArmActive(false);
      }
      return;
    }

    hand_active_input_ = true;
    if (arm_rearm_required_) {
      return;
    }
    if (rising_edge || !require_deadman_repress_after_timeout_) {
      arm_requested_ = true;
      hand_pose_available_ = false;
      new_hand_pose_ = false;
    }
  }

  void onHandTarget(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() &&
        message->header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring hand target in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }

    CartesianPose controller_pose = poseFromMessage(message->pose);
    if (!isFinite(controller_pose) || !normalizeQuaternion(controller_pose.orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring hand target with non-finite position or invalid quaternion");
      return;
    }
    controller_pose = controllerControlPose(
      controller_pose, controller_top_offset_, controller_to_tool_rotation_);
    if (!isFinite(controller_pose)) {
      return;
    }

    latest_controller_pose_ = controller_pose;
    last_hand_target_received_ = SteadyClock::now();
    hand_pose_available_ = true;
    new_hand_pose_ = true;
  }

  bool lookupToolPose(CartesianPose& pose)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        planning_frame_, end_effector_link_, tf2::TimePointZero);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for tool TF '%s' -> '%s': %s",
        planning_frame_.c_str(), end_effector_link_.c_str(), error.what());
      return false;
    }

    pose.position = Eigen::Vector3d(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    pose.orientation = Eigen::Quaterniond(
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z);
    if (!isFinite(pose) || !normalizeQuaternion(pose.orientation)) {
      return false;
    }

    const rclcpp::Time transform_stamp(transform.header.stamp);
    if (robot_state_timeout_sec_ > 0.0 && transform_stamp.nanoseconds() > 0) {
      const double age = (now() - transform_stamp).seconds();
      if (!std::isfinite(age) || age > robot_state_timeout_sec_ || age < -0.05) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Tool TF is stale or clock-skewed (age %.3f s)", age);
        return false;
      }
    }
    last_valid_tool_tf_ = SteadyClock::now();
    return true;
  }

  void ensureServoStarted(const SteadyTime& current_time)
  {
    if (servo_started_ || servo_start_request_in_flight_) {
      return;
    }
    if (hasTime(last_servo_start_attempt_) &&
        secondsBetween(current_time, last_servo_start_attempt_) < 1.0) {
      return;
    }
    last_servo_start_attempt_ = current_time;
    if (!servo_start_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for MoveIt Servo start service '%s'",
        servo_start_service_.c_str());
      return;
    }

    servo_start_request_in_flight_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    servo_start_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        servo_start_request_in_flight_ = false;
        try {
          const auto response = future.get();
          servo_started_ = response->success;
          if (servo_started_) {
            RCLCPP_INFO(get_logger(), "MoveIt Servo started");
          } else {
            RCLCPP_WARN(
              get_logger(), "MoveIt Servo start rejected: %s",
              response->message.c_str());
          }
        } catch (const std::exception& error) {
          RCLCPP_WARN(
            get_logger(), "MoveIt Servo start request failed: %s", error.what());
        }
      });
  }

  void updateArmControl()
  {
    const SteadyTime current_time = SteadyClock::now();
    publishRemainingArmHalts();
    ensureServoStarted(current_time);

    if (!arm_requested_ || arm_rearm_required_) {
      return;
    }

    const double timeout = std::max(0.02, hand_target_timeout_sec_);
    if (!hand_pose_available_) {
      if (hasTime(last_hand_gate_received_) &&
          secondsBetween(current_time, last_hand_gate_received_) > timeout) {
        haltArm(true, "deadman active without a fresh pose");
      }
      return;
    }
    if (secondsBetween(current_time, last_hand_target_received_) > timeout) {
      haltArm(true, "hand target timed out");
      return;
    }
    if (!servo_started_) {
      return;
    }

    if (!arm_tracking_) {
      CartesianPose tool_pose;
      if (!lookupToolPose(tool_pose)) {
        return;
      }
      controller_anchor_ = latest_controller_pose_;
      tool_anchor_ = tool_pose;
      filtered_target_ = tool_pose;
      previous_filtered_target_ = tool_pose;
      last_target_sample_time_ = last_hand_target_received_;
      target_linear_velocity_.setZero();
      target_angular_velocity_.setZero();
      last_linear_command_.setZero();
      last_angular_command_.setZero();
      last_arm_control_tick_ = current_time;
      new_hand_pose_ = false;
      arm_halt_commands_remaining_ = 0;
      arm_tracking_ = true;
      publishArmActive(true);
      publishMappedTarget(filtered_target_);
      RCLCPP_INFO(
        get_logger(),
        "Deadman anchored: controller top frame now maps to '%s'",
        end_effector_link_.c_str());
    }

    if (new_hand_pose_) {
      updateMappedTarget();
      new_hand_pose_ = false;
      if (!arm_tracking_) {
        return;
      }
    }

    CartesianPose current_tool_pose;
    if (!lookupToolPose(current_tool_pose)) {
      if (hasTime(last_valid_tool_tf_) &&
          secondsBetween(current_time, last_valid_tool_tf_) >
            std::max(0.02, robot_state_timeout_sec_)) {
        haltArm(true, "tool transform timed out");
      }
      return;
    }

    const Eigen::Vector3d linear_error =
      filtered_target_.position - current_tool_pose.position;
    const Eigen::Vector3d angular_error = orientationError(
      filtered_target_.orientation, current_tool_pose.orientation);
    Eigen::Vector3d linear_command = poseFeedback(
      linear_error, linear_gain_, position_deadband_m_);
    Eigen::Vector3d angular_command = poseFeedback(
      angular_error, angular_gain_, orientation_deadband_rad_);

    const double target_age = secondsBetween(current_time, last_hand_target_received_);
    if (feedforward_timeout_sec_ <= 0.0 || target_age <= feedforward_timeout_sec_) {
      linear_command += linear_feedforward_gain_ * target_linear_velocity_;
      angular_command += angular_feedforward_gain_ * target_angular_velocity_;
    }
    linear_command = clampVectorNorm(linear_command, max_linear_velocity_mps_);
    angular_command = clampVectorNorm(angular_command, max_angular_velocity_radps_);
    if (!isFinite(linear_command) || !isFinite(angular_command)) {
      haltArm(true, "non-finite Cartesian command");
      return;
    }

    const double control_dt = hasTime(last_arm_control_tick_)
      ? clamp(secondsBetween(current_time, last_arm_control_tick_), 0.0, 0.05)
      : 1.0 / arm_control_rate_hz_;
    last_arm_control_tick_ = current_time;
    linear_command = rateLimitVector(
      last_linear_command_,
      linear_command,
      max_linear_acceleration_mps2_,
      control_dt);
    angular_command = rateLimitVector(
      last_angular_command_,
      angular_command,
      max_angular_acceleration_radps2_,
      control_dt);

    publishServoTwist(linear_command, angular_command);
    last_linear_command_ = linear_command;
    last_angular_command_ = angular_command;
  }

  void updateMappedTarget()
  {
    CartesianPose raw_target = mapControllerDeltaToTool(
      latest_controller_pose_,
      controller_anchor_,
      tool_anchor_,
      hand_position_scale_);
    if (!isFinite(raw_target) || !constrainTarget(raw_target)) {
      haltArm(true, "invalid mapped target");
      return;
    }

    const double sample_dt = secondsBetween(
      last_hand_target_received_, last_target_sample_time_);
    const bool valid_dt = sample_dt > 1e-4 &&
      (feedforward_reset_gap_sec_ <= 0.0 || sample_dt <= feedforward_reset_gap_sec_);
    const CartesianPose next_target = valid_dt
      ? lowPassPose(
        filtered_target_, raw_target, target_filter_cutoff_hz_, sample_dt)
      : raw_target;

    if (valid_dt) {
      Eigen::Vector3d linear_velocity =
        (next_target.position - previous_filtered_target_.position) / sample_dt;
      Eigen::Vector3d angular_velocity = orientationError(
        next_target.orientation, previous_filtered_target_.orientation) / sample_dt;
      linear_velocity = clampVectorNorm(linear_velocity, max_linear_velocity_mps_);
      angular_velocity = clampVectorNorm(angular_velocity, max_angular_velocity_radps_);
      if (linear_velocity.norm() <=
          std::max(0.0, linear_feedforward_stop_velocity_mps_)) {
        target_linear_velocity_.setZero();
      } else {
        target_linear_velocity_ =
          (1.0 - feedforward_filter_alpha_) * target_linear_velocity_ +
          feedforward_filter_alpha_ * linear_velocity;
      }
      if (angular_velocity.norm() <=
          std::max(0.0, angular_feedforward_stop_velocity_radps_)) {
        target_angular_velocity_.setZero();
      } else {
        target_angular_velocity_ =
          (1.0 - feedforward_filter_alpha_) * target_angular_velocity_ +
          feedforward_filter_alpha_ * angular_velocity;
      }
    } else {
      target_linear_velocity_.setZero();
      target_angular_velocity_.setZero();
    }

    filtered_target_ = next_target;
    previous_filtered_target_ = next_target;
    last_target_sample_time_ = last_hand_target_received_;
    publishMappedTarget(filtered_target_);
    publishArmActive(true);
  }

  bool constrainTarget(CartesianPose& target)
  {
    if (!isFinite(target)) {
      return false;
    }
    const Eigen::Vector3d unconstrained = target.position;
    if (!constrainWorkspacePosition(
        target.position,
        max_hand_target_distance_m_,
        min_hand_target_z_m_,
        max_hand_target_z_m_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Mapped hand target or workspace limits are invalid");
      return false;
    }
    if (!target.position.isApprox(unconstrained, 1e-12)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Clamping mapped hand target to the configured Cartesian workspace");
    }
    return true;
  }

  void publishMappedTarget(const CartesianPose& target)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = planning_frame_;
    message.pose = poseToMessage(target);
    servo_pose_target_publisher_->publish(message);
  }

  void publishArmActive(bool active)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    servo_pose_active_publisher_->publish(message);
    effective_arm_active_ = active;
  }

  void publishServoTwist(
    const Eigen::Vector3d& linear,
    const Eigen::Vector3d& angular)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    command.twist.linear.x = linear.x();
    command.twist.linear.y = linear.y();
    command.twist.linear.z = linear.z();
    command.twist.angular.x = angular.x();
    command.twist.angular.y = angular.y();
    command.twist.angular.z = angular.z();
    servo_twist_publisher_->publish(command);
  }

  void publishZeroServoTwist()
  {
    publishServoTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  void publishRemainingArmHalts()
  {
    if (arm_halt_commands_remaining_ <= 0) {
      return;
    }
    publishZeroServoTwist();
    --arm_halt_commands_remaining_;
  }

  void haltArm(bool timeout, const char* reason)
  {
    const bool had_state = arm_tracking_ || arm_requested_ || effective_arm_active_ ||
      last_linear_command_.norm() > 0.0 || last_angular_command_.norm() > 0.0;
    arm_tracking_ = false;
    arm_requested_ = false;
    hand_pose_available_ = false;
    new_hand_pose_ = false;
    target_linear_velocity_.setZero();
    target_angular_velocity_.setZero();
    last_linear_command_.setZero();
    last_angular_command_.setZero();
    controller_anchor_ = CartesianPose{};
    tool_anchor_ = CartesianPose{};
    last_target_sample_time_ = SteadyTime{};
    last_arm_control_tick_ = SteadyTime{};
    if (timeout && require_deadman_repress_after_timeout_) {
      arm_rearm_required_ = true;
    }
    if (effective_arm_active_ || had_state) {
      publishArmActive(false);
      publishZeroServoTwist();
      arm_halt_commands_remaining_ = std::max(0, arm_halt_command_count_ - 1);
    }
    if (had_state) {
      RCLCPP_INFO(get_logger(), "Arm pursuit halted: %s", reason);
    }
  }

  // Parameters.
  std::string arm_group_;
  std::string end_effector_link_;
  std::string planning_frame_;
  std::string head_input_topic_;
  std::string head_command_topic_;
  std::string hand_target_topic_;
  std::string hand_target_active_topic_;
  std::string gripper_input_topic_;
  std::string gripper_command_topic_;
  std::string base_input_topic_;
  std::string base_active_topic_;
  std::string base_command_topic_;
  std::string base_command_frame_;
  std::string joint_state_topic_;
  std::string servo_twist_topic_;
  std::string servo_pose_target_topic_;
  std::string servo_pose_active_topic_;
  std::string servo_start_service_;

  double arm_control_rate_hz_{ 100.0 };
  double hand_target_timeout_sec_{ 0.12 };
  double robot_state_timeout_sec_{ 0.20 };
  bool require_deadman_repress_after_timeout_{ true };
  int arm_halt_command_count_{ 4 };
  double max_hand_target_distance_m_{ 1.5 };
  double min_hand_target_z_m_{ 0.2 };
  double max_hand_target_z_m_{ 1.6 };
  Eigen::Vector3d hand_position_scale_{ Eigen::Vector3d::Ones() };
  Eigen::Vector3d controller_top_offset_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d controller_to_tool_rpy_{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond controller_to_tool_rotation_{ Eigen::Quaterniond::Identity() };
  double linear_gain_{ 5.0 };
  double angular_gain_{ 2.5 };
  double max_linear_velocity_mps_{ 0.35 };
  double max_angular_velocity_radps_{ 1.2 };
  double max_linear_acceleration_mps2_{ 2.0 };
  double max_angular_acceleration_radps2_{ 6.0 };
  double position_deadband_m_{ 0.0005 };
  double orientation_deadband_rad_{ 0.003 };
  double target_filter_cutoff_hz_{ 12.0 };
  double linear_feedforward_gain_{ 1.0 };
  double angular_feedforward_gain_{ 1.0 };
  double feedforward_filter_alpha_{ 0.35 };
  double feedforward_timeout_sec_{ 0.05 };
  double feedforward_reset_gap_sec_{ 0.12 };
  double linear_feedforward_stop_velocity_mps_{ 0.005 };
  double angular_feedforward_stop_velocity_radps_{ 0.02 };

  std::vector<std::string> head_joint_names_;
  double head_publish_rate_hz_{ 20.0 };
  double head_command_duration_sec_{ 0.10 };
  double head_deadband_rad_{ 0.002 };
  double head_limit_scale_{ 0.9 };
  double head_pan_sign_{ -1.0 };
  double head_tilt_sign_{ -1.0 };
  std::array<double, 2> head_pan_limits_rad_{ -1.24, 1.24 };
  std::array<double, 2> head_tilt_limits_rad_{ -0.98, 0.72 };

  std::vector<std::string> gripper_joint_names_;
  double gripper_min_position_m_{ 0.0 };
  double gripper_max_position_m_{ 0.045 };
  double gripper_deadband_m_{ 0.0005 };
  double gripper_command_duration_sec_{ 0.15 };
  double gripper_max_velocity_mps_{ 0.04 };
  double joint_state_timeout_sec_{ 0.25 };

  double base_publish_rate_hz_{ 30.0 };
  double base_input_timeout_sec_{ 0.15 };
  int base_halt_command_count_{ 3 };
  double base_max_linear_velocity_mps_{ 0.25 };
  double base_max_angular_velocity_radps_{ 0.6 };
  double base_max_linear_acceleration_mps2_{ 0.5 };
  double base_max_angular_acceleration_radps2_{ 1.2 };
  double base_max_linear_deceleration_mps2_{ 1.0 };
  double base_max_angular_deceleration_radps2_{ 2.4 };

  bool enable_realtime_scheduling_{ true };
  int realtime_priority_{ 40 };
  bool lock_memory_{ true };
  int cpu_affinity_{ -1 };

  // ROS interfaces.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    head_trajectory_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    gripper_trajectory_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    servo_pose_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_pose_active_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    head_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    hand_target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hand_active_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    base_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_active_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::TimerBase::SharedPtr arm_control_timer_;
  rclcpp::TimerBase::SharedPtr head_timer_;
  rclcpp::TimerBase::SharedPtr base_timer_;

  // Head and gripper state.
  std::optional<geometry_msgs::msg::PoseStamped> latest_head_pose_;
  bool new_head_pose_{ false };
  std::optional<double> last_head_pan_;
  std::optional<double> last_head_tilt_;
  std::unordered_map<std::string, double> joint_positions_;
  SteadyTime last_joint_state_received_{};
  std::optional<double> last_gripper_target_;

  // Base state.
  bool base_active_input_{ false };
  bool base_command_enabled_{ false };
  bool base_rearm_required_{ false };
  std::optional<std::pair<double, double>> pending_base_command_;
  SteadyTime base_enabled_time_{};
  SteadyTime last_base_command_received_{};
  SteadyTime last_base_update_{};
  double last_base_linear_{ 0.0 };
  double last_base_angular_{ 0.0 };
  int base_halt_commands_remaining_{ 0 };

  // Arm state. The single-threaded executor is intentional: all of these are
  // latest-value slots, not queues, and the 100 Hz loop never waits for TF or a
  // service response.
  bool hand_active_input_{ false };
  bool arm_requested_{ false };
  bool arm_rearm_required_{ false };
  bool arm_tracking_{ false };
  bool effective_arm_active_{ false };
  bool hand_pose_available_{ false };
  bool new_hand_pose_{ false };
  CartesianPose latest_controller_pose_;
  CartesianPose controller_anchor_;
  CartesianPose tool_anchor_;
  CartesianPose filtered_target_;
  CartesianPose previous_filtered_target_;
  Eigen::Vector3d target_linear_velocity_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d target_angular_velocity_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d last_linear_command_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d last_angular_command_{ Eigen::Vector3d::Zero() };
  SteadyTime last_hand_gate_received_{};
  SteadyTime last_hand_target_received_{};
  SteadyTime last_target_sample_time_{};
  SteadyTime last_arm_control_tick_{};
  SteadyTime last_valid_tool_tf_{};
  int arm_halt_commands_remaining_{ 0 };

  bool servo_started_{ false };
  bool servo_start_request_in_flight_{ false };
  SteadyTime last_servo_start_attempt_{};
  std::atomic<bool> shutdown_halt_sent_{ false };
};

RealtimeResult configureRealtime(const std::shared_ptr<ViveMoveItServer>& node)
{
  RealtimeResult result;

  if (node->cpuAffinity() >= 0) {
    if (node->cpuAffinity() >= CPU_SETSIZE) {
      RCLCPP_WARN(
        node->get_logger(),
        "CPU affinity %d is outside the supported range [0, %d)",
        node->cpuAffinity(), CPU_SETSIZE);
    } else {
      cpu_set_t cpu_set;
      CPU_ZERO(&cpu_set);
      CPU_SET(node->cpuAffinity(), &cpu_set);
      const int affinity_result = pthread_setaffinity_np(
        pthread_self(), sizeof(cpu_set), &cpu_set);
      if (affinity_result == 0) {
        result.affinity_active = true;
      } else {
        RCLCPP_WARN(
          node->get_logger(),
          "Could not pin teleop executor to CPU %d: %s",
          node->cpuAffinity(), std::strerror(affinity_result));
      }
    }
  }

  if (node->memoryLockRequested()) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      result.memory_lock_active = true;
      RCLCPP_INFO(node->get_logger(), "Process memory locking enabled");
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "Could not lock process memory: %s",
        std::strerror(errno));
    }
  }

  if (node->realtimeSchedulingRequested()) {
    sched_param parameters{};
    parameters.sched_priority = clamp(
      node->realtimePriority(),
      sched_get_priority_min(SCHED_FIFO),
      sched_get_priority_max(SCHED_FIFO));
    const int scheduling_result = pthread_setschedparam(
      pthread_self(), SCHED_FIFO, &parameters);
    if (scheduling_result == 0) {
      result.scheduling_active = true;
      RCLCPP_INFO(
        node->get_logger(),
        "SCHED_FIFO enabled for teleop executor at priority %d",
        parameters.sched_priority);
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "Could not enable SCHED_FIFO: %s; continuing with normal scheduling",
        std::strerror(scheduling_result));
    }
  }
  return result;
}

}  // namespace vive_moveit_server

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vive_moveit_server::ViveMoveItServer>();
  const auto realtime = vive_moveit_server::configureRealtime(node);
  node->setRealtimeStatus(realtime);

  // A single executor gives the high-rate arm loop deterministic ownership of
  // all latest-value state. DDS receive threads remain separate inside rmw.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  node->stopAllMotion();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
