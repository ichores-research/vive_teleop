#include "vive_moveit_server/control_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vive_moveit_server
{

bool normalizeQuaternion(Eigen::Quaterniond& quaternion)
{
  const Eigen::Vector4d coefficients(
    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
  if (!coefficients.allFinite()) {
    return false;
  }

  const double scale = coefficients.cwiseAbs().maxCoeff();
  if (!std::isfinite(scale) || scale <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const Eigen::Vector4d scaled = coefficients / scale;
  const double scaled_norm = scaled.norm();
  if (!std::isfinite(scaled_norm) || scaled_norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const Eigen::Vector4d normalized = scaled / scaled_norm;
  quaternion = Eigen::Quaterniond(
    normalized.w(), normalized.x(), normalized.y(), normalized.z());
  return true;
}

bool isFinite(const Eigen::Vector3d& vector)
{
  return vector.allFinite();
}

bool isFinite(const CartesianPose& pose)
{
  Eigen::Quaterniond orientation = pose.orientation;
  return isFinite(pose.position) && normalizeQuaternion(orientation);
}

Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy)
{
  Eigen::Quaterniond result =
    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  normalizeQuaternion(result);
  return result;
}

CartesianPose controllerControlPose(
  const CartesianPose& controller_pose,
  const Eigen::Vector3d& controller_top_offset,
  const Eigen::Quaterniond& controller_to_tool_rotation)
{
  CartesianPose result = controller_pose;
  Eigen::Quaterniond controller_orientation = controller_pose.orientation;
  Eigen::Quaterniond alignment = controller_to_tool_rotation;
  if (!normalizeQuaternion(controller_orientation) || !normalizeQuaternion(alignment)) {
    result.orientation = Eigen::Quaterniond(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
    return result;
  }

  result.position += controller_orientation * controller_top_offset;
  result.orientation = controller_orientation * alignment;
  normalizeQuaternion(result.orientation);
  return result;
}

CartesianPose mapControllerDeltaToTool(
  const CartesianPose& controller_pose,
  const CartesianPose& controller_anchor,
  const CartesianPose& tool_anchor,
  const Eigen::Vector3d& position_scale)
{
  CartesianPose result = tool_anchor;
  result.position += position_scale.cwiseProduct(
    controller_pose.position - controller_anchor.position);

  Eigen::Quaterniond current = controller_pose.orientation;
  Eigen::Quaterniond anchor = controller_anchor.orientation;
  Eigen::Quaterniond tool = tool_anchor.orientation;
  if (!normalizeQuaternion(current) || !normalizeQuaternion(anchor) ||
      !normalizeQuaternion(tool)) {
    result.orientation = Eigen::Quaterniond(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
    return result;
  }

  // The delta is expressed in the controller's local frame and then applied
  // in the tool's local frame. This identifies the controller's configured
  // "top" frame with the end-effector frame throughout the clutch interval.
  Eigen::Quaterniond body_delta = anchor.conjugate() * current;
  normalizeQuaternion(body_delta);
  result.orientation = tool * body_delta;
  normalizeQuaternion(result.orientation);
  return result;
}

Eigen::Vector3d orientationError(
  const Eigen::Quaterniond& target,
  const Eigen::Quaterniond& current)
{
  Eigen::Quaterniond normalized_target = target;
  Eigen::Quaterniond normalized_current = current;
  if (!normalizeQuaternion(normalized_target) ||
      !normalizeQuaternion(normalized_current)) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }

  Eigen::Quaterniond error = normalized_target * normalized_current.conjugate();
  normalizeQuaternion(error);
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vector(error.x(), error.y(), error.z());
  const double vector_norm = vector.norm();
  if (vector_norm < 1e-12) {
    return Eigen::Vector3d::Zero();
  }

  const double angle = 2.0 * std::atan2(vector_norm, std::max(0.0, error.w()));
  return vector * (angle / vector_norm);
}

Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d& vector, double limit)
{
  if (!isFinite(vector) || !std::isfinite(limit) || limit <= 0.0) {
    return vector;
  }
  const double norm = vector.norm();
  if (norm <= limit || norm <= 1e-12) {
    return vector;
  }
  return vector * (limit / norm);
}

bool constrainWorkspacePosition(
  Eigen::Vector3d& position,
  double maximum_distance,
  double minimum_z,
  double maximum_z)
{
  if (!isFinite(position) || !std::isfinite(maximum_distance) ||
      !std::isfinite(minimum_z) || !std::isfinite(maximum_z)) {
    return false;
  }

  double lower_z = -std::numeric_limits<double>::infinity();
  double upper_z = std::numeric_limits<double>::infinity();
  if (minimum_z < maximum_z) {
    lower_z = minimum_z;
    upper_z = maximum_z;
  }

  if (maximum_distance > 0.0) {
    lower_z = std::max(lower_z, -maximum_distance);
    upper_z = std::min(upper_z, maximum_distance);
  }
  if (lower_z > upper_z) {
    return false;
  }
  position.z() = std::clamp(position.z(), lower_z, upper_z);

  if (maximum_distance > 0.0) {
    const double horizontal_distance = std::hypot(position.x(), position.y());
    const double maximum_horizontal_distance = std::sqrt(std::max(
      0.0,
      (maximum_distance * maximum_distance) -
      (position.z() * position.z())));
    if (horizontal_distance > maximum_horizontal_distance) {
      if (horizontal_distance <= std::numeric_limits<double>::epsilon()) {
        position.x() = 0.0;
        position.y() = 0.0;
      } else {
        const double horizontal_scale =
          maximum_horizontal_distance / horizontal_distance;
        position.x() *= horizontal_scale;
        position.y() *= horizontal_scale;
      }
    }
  }
  return isFinite(position);
}

Eigen::Vector3d poseFeedback(
  const Eigen::Vector3d& error,
  double gain,
  double deadband)
{
  if (!isFinite(error) || !std::isfinite(gain) || !std::isfinite(deadband)) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  if (error.norm() <= std::max(0.0, deadband)) {
    return Eigen::Vector3d::Zero();
  }
  return error * gain;
}

Eigen::Vector3d rateLimitVector(
  const Eigen::Vector3d& current,
  const Eigen::Vector3d& target,
  double maximum_rate,
  double dt)
{
  if (!isFinite(current) || !isFinite(target) || !std::isfinite(maximum_rate) ||
      !std::isfinite(dt) || maximum_rate <= 0.0 || dt <= 0.0) {
    return target;
  }
  const Eigen::Vector3d delta = target - current;
  const double maximum_delta = maximum_rate * dt;
  if (delta.norm() <= maximum_delta || delta.norm() <= 1e-12) {
    return target;
  }
  return current + delta.normalized() * maximum_delta;
}

CartesianPose lowPassPose(
  const CartesianPose& previous,
  const CartesianPose& target,
  double cutoff_hz,
  double dt)
{
  if (!std::isfinite(cutoff_hz) || !std::isfinite(dt) || cutoff_hz <= 0.0 ||
      dt <= 0.0) {
    return target;
  }

  constexpr double pi = 3.14159265358979323846;
  const double alpha = std::clamp(
    1.0 - std::exp(-2.0 * pi * cutoff_hz * dt), 0.0, 1.0);
  CartesianPose result;
  result.position = previous.position + alpha * (target.position - previous.position);

  Eigen::Quaterniond previous_orientation = previous.orientation;
  Eigen::Quaterniond target_orientation = target.orientation;
  if (!normalizeQuaternion(previous_orientation) ||
      !normalizeQuaternion(target_orientation)) {
    return target;
  }
  if (previous_orientation.dot(target_orientation) < 0.0) {
    target_orientation.coeffs() *= -1.0;
  }
  result.orientation = previous_orientation.slerp(alpha, target_orientation);
  normalizeQuaternion(result.orientation);
  return result;
}

double approachVelocity(
  double current,
  double target,
  double acceleration,
  double deceleration,
  double dt)
{
  if (!std::isfinite(current) || !std::isfinite(target) ||
      !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
      !std::isfinite(dt)) {
    return target;
  }
  if (current == target) {
    return target;
  }

  const bool slowing = (current * target < 0.0) || std::abs(target) < std::abs(current);
  const double rate = slowing ? deceleration : acceleration;
  const double maximum_delta = std::max(0.0, rate) * std::max(0.0, dt);
  const double difference = target - current;
  if (std::abs(difference) <= maximum_delta) {
    return target;
  }
  return current + std::copysign(maximum_delta, difference);
}

}  // namespace vive_moveit_server
