#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vive_moveit_server
{

struct CartesianPose
{
  Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
};

bool normalizeQuaternion(Eigen::Quaterniond& quaternion);
bool isFinite(const Eigen::Vector3d& vector);
bool isFinite(const CartesianPose& pose);

Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy);

CartesianPose controllerControlPose(
  const CartesianPose& controller_pose,
  const Eigen::Vector3d& controller_top_offset,
  const Eigen::Quaterniond& controller_to_tool_rotation);

CartesianPose mapControllerDeltaToTool(
  const CartesianPose& controller_pose,
  const CartesianPose& controller_anchor,
  const CartesianPose& tool_anchor,
  const Eigen::Vector3d& position_scale);

Eigen::Vector3d orientationError(
  const Eigen::Quaterniond& target,
  const Eigen::Quaterniond& current);

Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d& vector, double limit);
bool constrainWorkspacePosition(
  Eigen::Vector3d& position,
  double maximum_distance,
  double minimum_z,
  double maximum_z);
Eigen::Vector3d poseFeedback(
  const Eigen::Vector3d& error,
  double gain,
  double deadband);
Eigen::Vector3d rateLimitVector(
  const Eigen::Vector3d& current,
  const Eigen::Vector3d& target,
  double maximum_rate,
  double dt);
CartesianPose lowPassPose(
  const CartesianPose& previous,
  const CartesianPose& target,
  double cutoff_hz,
  double dt);
double approachVelocity(
  double current,
  double target,
  double acceleration,
  double deceleration,
  double dt);

}  // namespace vive_moveit_server
