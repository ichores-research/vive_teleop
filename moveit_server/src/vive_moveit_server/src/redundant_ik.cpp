#include "vive_moveit_server/redundant_ik.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace vive_moveit_server {
namespace {

bool validDimensions(const Eigen::MatrixXd &jacobian,
                     const Eigen::VectorXd &cartesian_velocity,
                     const Eigen::VectorXd &joint_position,
                     const Eigen::VectorXd &lower_limits,
                     const Eigen::VectorXd &upper_limits,
                     const Eigen::VectorXd &velocity_limits,
                     const Eigen::VectorXd &secondary_velocity) {
  const Eigen::Index joints = jacobian.cols();
  return jacobian.rows() == 6 && joints > 0 &&
         cartesian_velocity.size() == jacobian.rows() &&
         joint_position.size() == joints && lower_limits.size() == joints &&
         upper_limits.size() == joints && velocity_limits.size() == joints &&
         (secondary_velocity.size() == 0 ||
          secondary_velocity.size() == joints);
}

double adaptiveDamping(const Eigen::VectorXd &singular_values,
                       const RedundantIkOptions &options) {
  if (singular_values.size() == 0) {
    return std::max(0.0, options.maximum_damping);
  }
  const double threshold = std::max(0.0, options.damping_threshold);
  const double maximum = std::max(0.0, options.maximum_damping);
  const double minimum = std::max(0.0, singular_values.minCoeff());
  if (threshold <= 0.0 || minimum >= threshold) {
    return 0.0;
  }
  const double ratio = minimum / threshold;
  return maximum * (1.0 - ratio * ratio);
}

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = decomposition.singularValues();
  if (singular_values.size() == 0) {
    return Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
  }

  const double tolerance =
      std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max(matrix.rows(), matrix.cols())) *
      singular_values.maxCoeff();
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular_values.size());
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    if (singular_values[index] > tolerance) {
      inverse[index] = 1.0 / singular_values[index];
    }
  }
  return decomposition.matrixV() * inverse.asDiagonal() *
         decomposition.matrixU().transpose();
}

Eigen::MatrixXd weightedDampedInverse(const Eigen::MatrixXd &jacobian,
                                      const Eigen::VectorXd &inverse_weight,
                                      const RedundantIkOptions &options) {
  const Eigen::VectorXd square_root = inverse_weight.cwiseSqrt();
  const Eigen::MatrixXd weighted = jacobian * square_root.asDiagonal();
  const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
      weighted, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = decomposition.singularValues();
  if (singular_values.size() == 0) {
    return Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());
  }

  const double damping = adaptiveDamping(singular_values, options);
  const double tolerance =
      std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max(weighted.rows(), weighted.cols())) *
      singular_values.maxCoeff();
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular_values.size());
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    const double singular = singular_values[index];
    if (damping > 0.0) {
      inverse[index] = singular / (singular * singular + damping * damping);
    } else if (singular > tolerance) {
      inverse[index] = 1.0 / singular;
    }
  }
  return square_root.asDiagonal() * decomposition.matrixV() *
         inverse.asDiagonal() * decomposition.matrixU().transpose();
}

Eigen::VectorXd solveOrientationFirst(const Eigen::MatrixXd &jacobian,
                                      const Eigen::VectorXd &cartesian_velocity,
                                      const Eigen::VectorXd &inverse_weight,
                                      const Eigen::VectorXd &secondary_velocity,
                                      const RedundantIkOptions &options) {
  const Eigen::Index joints = jacobian.cols();
  const Eigen::MatrixXd angular_jacobian = jacobian.bottomRows(3);
  const Eigen::MatrixXd linear_jacobian = jacobian.topRows(3);

  const Eigen::MatrixXd angular_inverse =
      weightedDampedInverse(angular_jacobian, inverse_weight, options);
  const Eigen::VectorXd rotation = angular_inverse * cartesian_velocity.tail(3);

  // Use an exact projector for task priority even when adaptive damping is
  // active in the velocity solution. Translation therefore cannot change an
  // achievable angular command.
  const Eigen::MatrixXd angular_nullspace =
      Eigen::MatrixXd::Identity(joints, joints) -
      pseudoInverse(angular_jacobian) * angular_jacobian;
  const Eigen::MatrixXd projected_linear = linear_jacobian * angular_nullspace;
  const Eigen::VectorXd linear_residual =
      cartesian_velocity.head(3) - linear_jacobian * rotation;
  const Eigen::VectorXd translation =
      angular_nullspace *
      weightedDampedInverse(projected_linear, inverse_weight, options) *
      linear_residual;

  const Eigen::MatrixXd task_nullspace =
      angular_nullspace * (Eigen::MatrixXd::Identity(joints, joints) -
                           pseudoInverse(projected_linear) * projected_linear);
  return rotation + translation + task_nullspace * secondary_velocity;
}

Eigen::VectorXd solveAllowedJoints(const Eigen::MatrixXd &jacobian,
                                   const Eigen::VectorXd &cartesian_velocity,
                                   const Eigen::VectorXd &inverse_weight,
                                   const Eigen::VectorXd &secondary_velocity,
                                   const std::vector<bool> &allowed,
                                   const RedundantIkOptions &options) {
  Eigen::Index allowed_count = 0;
  for (const bool is_allowed : allowed) {
    allowed_count += is_allowed ? 1 : 0;
  }
  Eigen::VectorXd result = Eigen::VectorXd::Zero(jacobian.cols());
  if (allowed_count == 0) {
    return result;
  }

  Eigen::MatrixXd reduced_jacobian(jacobian.rows(), allowed_count);
  Eigen::VectorXd reduced_weight(allowed_count);
  Eigen::VectorXd reduced_secondary(allowed_count);
  Eigen::Index reduced_index = 0;
  for (Eigen::Index index = 0; index < jacobian.cols(); ++index) {
    if (!allowed[static_cast<std::size_t>(index)]) {
      continue;
    }
    reduced_jacobian.col(reduced_index) = jacobian.col(index);
    reduced_weight[reduced_index] = inverse_weight[index];
    reduced_secondary[reduced_index] = secondary_velocity[index];
    ++reduced_index;
  }

  const Eigen::VectorXd reduced_result =
      solveOrientationFirst(reduced_jacobian, cartesian_velocity,
                            reduced_weight, reduced_secondary, options);
  reduced_index = 0;
  for (Eigen::Index index = 0; index < jacobian.cols(); ++index) {
    if (allowed[static_cast<std::size_t>(index)]) {
      result[index] = reduced_result[reduced_index++];
    }
  }
  return result;
}

std::vector<Eigen::Index>
jointPriorityOrder(const Eigen::VectorXd &motion_weights) {
  std::vector<Eigen::Index> priority(
      static_cast<std::size_t>(motion_weights.size()));
  std::iota(priority.begin(), priority.end(), Eigen::Index{0});
  std::stable_sort(
      priority.begin(), priority.end(),
      [&motion_weights](const Eigen::Index left, const Eigen::Index right) {
        return motion_weights[left] < motion_weights[right];
      });
  return priority;
}

bool taskResidualSatisfied(const Eigen::VectorXd &cartesian_velocity,
                           const Eigen::VectorXd &joint_velocity,
                           const Eigen::MatrixXd &jacobian) {
  const Eigen::VectorXd residual =
      cartesian_velocity - jacobian * joint_velocity;
  constexpr double absolute_tolerance = 1e-6;
  constexpr double relative_tolerance = 1e-4;
  const double linear_tolerance =
      absolute_tolerance +
      relative_tolerance * cartesian_velocity.head(3).norm();
  const double angular_tolerance =
      absolute_tolerance +
      relative_tolerance * cartesian_velocity.tail(3).norm();
  return residual.head(3).norm() <= linear_tolerance &&
         residual.tail(3).norm() <= angular_tolerance;
}

struct BoundedSolve {
  Eigen::VectorXd joint_velocity;
  std::vector<bool> velocity_limited;
  std::vector<bool> position_limited;
  std::vector<bool> blocked;
};

BoundedSolve solveBoundedSelection(
    const Eigen::MatrixXd &jacobian, const Eigen::VectorXd &cartesian_velocity,
    const Eigen::VectorXd &inverse_weight,
    const Eigen::VectorXd &secondary_velocity,
    const std::vector<bool> &selected,
    const Eigen::VectorXd &lower_velocity_bound,
    const Eigen::VectorXd &upper_velocity_bound,
    const Eigen::VectorXd &physical_velocity_bound,
    const std::vector<bool> &lower_position_limited,
    const std::vector<bool> &upper_position_limited,
    const std::vector<bool> &lower_blocked,
    const std::vector<bool> &upper_blocked, const RedundantIkOptions &options) {
  const Eigen::Index joints = jacobian.cols();
  BoundedSolve result{
      Eigen::VectorXd::Zero(joints),
      std::vector<bool>(static_cast<std::size_t>(joints), false),
      std::vector<bool>(static_cast<std::size_t>(joints), false),
      std::vector<bool>(static_cast<std::size_t>(joints), false)};
  std::vector<bool> free = selected;
  Eigen::VectorXd fixed_velocity = Eigen::VectorXd::Zero(joints);

  for (Eigen::Index iteration = 0; iteration <= joints; ++iteration) {
    const Eigen::VectorXd residual =
        cartesian_velocity - jacobian * fixed_velocity;
    const Eigen::VectorXd free_velocity = solveAllowedJoints(
        jacobian, residual, inverse_weight, secondary_velocity, free, options);
    const Eigen::VectorXd candidate = fixed_velocity + free_velocity;

    Eigen::Index worst_index = -1;
    double worst_normalized_violation = 0.0;
    bool clamp_lower = false;
    for (Eigen::Index index = 0; index < joints; ++index) {
      if (!free[static_cast<std::size_t>(index)]) {
        continue;
      }
      double violation = 0.0;
      bool lower = false;
      if (candidate[index] < lower_velocity_bound[index] - 1e-12) {
        violation = lower_velocity_bound[index] - candidate[index];
        lower = true;
      } else if (candidate[index] > upper_velocity_bound[index] + 1e-12) {
        violation = candidate[index] - upper_velocity_bound[index];
      }
      const double normalized =
          violation / std::max(1e-12, physical_velocity_bound[index]);
      if (normalized > worst_normalized_violation) {
        worst_normalized_violation = normalized;
        worst_index = index;
        clamp_lower = lower;
      }
    }

    if (worst_index < 0) {
      result.joint_velocity = candidate;
      break;
    }

    const std::size_t bounded_index = static_cast<std::size_t>(worst_index);
    fixed_velocity[worst_index] = clamp_lower
                                      ? lower_velocity_bound[worst_index]
                                      : upper_velocity_bound[worst_index];
    free[bounded_index] = false;
    const bool limited_by_position =
        clamp_lower ? lower_position_limited[bounded_index]
                    : upper_position_limited[bounded_index];
    if (limited_by_position) {
      result.position_limited[bounded_index] = true;
      result.blocked[bounded_index] = clamp_lower
                                          ? lower_blocked[bounded_index]
                                          : upper_blocked[bounded_index];
    } else {
      result.velocity_limited[bounded_index] = true;
    }
    result.joint_velocity = fixed_velocity;
  }
  return result;
}

BoundedSolve solvePrioritizedBounded(
    const Eigen::MatrixXd &jacobian, const Eigen::VectorXd &cartesian_velocity,
    const Eigen::VectorXd &inverse_weight,
    const Eigen::VectorXd &motion_weights,
    const Eigen::VectorXd &secondary_velocity,
    const Eigen::VectorXd &lower_velocity_bound,
    const Eigen::VectorXd &upper_velocity_bound,
    const Eigen::VectorXd &physical_velocity_bound,
    const std::vector<bool> &lower_position_limited,
    const std::vector<bool> &upper_position_limited,
    const std::vector<bool> &lower_blocked,
    const std::vector<bool> &upper_blocked, const RedundantIkOptions &options,
    int &selected_joint_count) {
  const Eigen::Index joints = jacobian.cols();
  const std::vector<Eigen::Index> priority = jointPriorityOrder(motion_weights);
  std::vector<bool> selected(static_cast<std::size_t>(joints), false);
  std::size_t required_depth = 0;
  for (std::size_t depth = 0; depth < priority.size(); ++depth) {
    if (std::abs(secondary_velocity[priority[depth]]) > 1e-12) {
      required_depth = depth + 1;
    }
  }

  BoundedSolve result{
      Eigen::VectorXd::Zero(joints),
      std::vector<bool>(static_cast<std::size_t>(joints), false),
      std::vector<bool>(static_cast<std::size_t>(joints), false),
      std::vector<bool>(static_cast<std::size_t>(joints), false)};
  for (std::size_t depth = 0; depth < priority.size(); ++depth) {
    selected[static_cast<std::size_t>(priority[depth])] = true;
    result = solveBoundedSelection(
        jacobian, cartesian_velocity, inverse_weight, secondary_velocity,
        selected, lower_velocity_bound, upper_velocity_bound,
        physical_velocity_bound, lower_position_limited, upper_position_limited,
        lower_blocked, upper_blocked, options);
    selected_joint_count = static_cast<int>(depth + 1);
    if (depth + 1 >= required_depth &&
        taskResidualSatisfied(cartesian_velocity, result.joint_velocity,
                              jacobian)) {
      break;
    }
  }
  return result;
}

} // namespace

double minimumSingularValue(const Eigen::MatrixXd &jacobian) {
  if (jacobian.rows() == 0 || jacobian.cols() == 0 || !jacobian.allFinite()) {
    return 0.0;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (decomposition.singularValues().size() == 0) {
    return 0.0;
  }
  return std::max(0.0, decomposition.singularValues().minCoeff());
}

double normalizedUpwardSlope(const Eigen::Vector3d &start,
                             const Eigen::Vector3d &end) {
  if (!start.allFinite() || !end.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Vector3d displacement = end - start;
  const double length = displacement.norm();
  if (!std::isfinite(length) || length <= 1e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::clamp(displacement.z() / length, -1.0, 1.0);
}

double upwardSegmentPenalty(const Eigen::Vector3d &start,
                            const Eigen::Vector3d &end,
                            double activation_ratio) {
  const double slope = normalizedUpwardSlope(start, end);
  if (!std::isfinite(slope)) {
    return 0.0;
  }
  const double activation =
      std::clamp(std::isfinite(activation_ratio) ? activation_ratio : 0.0,
                 -1.0, 1.0);
  const double excess = std::max(0.0, slope - activation);
  return 0.5 * excess * excess;
}

double lowElbowPreferenceCost(const Eigen::Vector3d &start,
                              const Eigen::Vector3d &elbow,
                              double desired_low_slope,
                              double reward_weight) {
  const double slope = normalizedUpwardSlope(start, elbow);
  if (!std::isfinite(slope) || !std::isfinite(desired_low_slope)) {
    return 0.0;
  }
  const double target = std::clamp(desired_low_slope, -1.0, 1.0);
  const double weight =
      std::isfinite(reward_weight) ? std::max(0.0, reward_weight) : 0.0;
  const double excess = std::max(0.0, slope - target);
  return 0.5 * weight * excess * excess;
}

Eigen::VectorXd projectIntoTaskNullspace(
    const Eigen::MatrixXd &jacobian,
    const Eigen::VectorXd &joint_velocity) {
  if (jacobian.cols() <= 0 || joint_velocity.size() != jacobian.cols() ||
      !jacobian.allFinite() || !joint_velocity.allFinite()) {
    return Eigen::VectorXd{};
  }
  if (jacobian.rows() == 0) {
    return joint_velocity;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
      jacobian, Eigen::ComputeFullV);
  const Eigen::VectorXd singular_values = decomposition.singularValues();
  const double maximum = singular_values.size() > 0
                             ? singular_values.maxCoeff()
                             : 0.0;
  const double tolerance =
      std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max(jacobian.rows(), jacobian.cols())) *
      maximum;
  Eigen::Index rank = 0;
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    rank += singular_values[index] > tolerance ? 1 : 0;
  }

  Eigen::VectorXd projected = Eigen::VectorXd::Zero(jacobian.cols());
  for (Eigen::Index index = rank;
       index < decomposition.matrixV().cols(); ++index) {
    const Eigen::VectorXd direction = decomposition.matrixV().col(index);
    projected += direction * direction.dot(joint_velocity);
  }
  return projected;
}

Eigen::VectorXd removeOpposingSecondaryComponent(
    const Eigen::VectorXd &higher_priority,
    const Eigen::VectorXd &candidate) {
  if (higher_priority.size() == 0 ||
      higher_priority.size() != candidate.size() ||
      !higher_priority.allFinite() || !candidate.allFinite()) {
    return Eigen::VectorXd{};
  }
  const double squared_norm = higher_priority.squaredNorm();
  if (squared_norm <= 1e-16) {
    return candidate;
  }
  const double projection = candidate.dot(higher_priority) / squared_norm;
  return projection < 0.0
             ? candidate - projection * higher_priority
             : candidate;
}

Eigen::VectorXd inwardJointLimitRecoveryVelocity(
    const Eigen::VectorXd &joint_position,
    const Eigen::VectorXd &lower_limits,
    const Eigen::VectorXd &upper_limits,
    const Eigen::VectorXd &velocity_limits,
    const Eigen::VectorXd &margins,
    double release_buffer,
    double gain,
    double maximum_velocity,
    double release_tolerance) {
  const Eigen::Index joints = joint_position.size();
  if (joints <= 0 || lower_limits.size() != joints ||
      upper_limits.size() != joints || velocity_limits.size() != joints ||
      margins.size() != joints || !joint_position.allFinite() ||
      !lower_limits.allFinite() || !upper_limits.allFinite() ||
      !velocity_limits.allFinite() || !margins.allFinite() ||
      (upper_limits.array() <= lower_limits.array()).any() ||
      (velocity_limits.array() <= 0.0).any() ||
      (margins.array() < 0.0).any() || !std::isfinite(release_buffer) ||
      !std::isfinite(gain) || !std::isfinite(maximum_velocity) ||
      !std::isfinite(release_tolerance) || release_buffer < 0.0 ||
      gain <= 0.0 || maximum_velocity <= 0.0 ||
      release_tolerance < 0.0) {
    return Eigen::VectorXd{};
  }

  Eigen::VectorXd recovery = Eigen::VectorXd::Zero(joints);
  for (Eigen::Index index = 0; index < joints; ++index) {
    const double lower_target =
        lower_limits[index] + margins[index] + release_buffer;
    const double upper_target =
        upper_limits[index] - margins[index] - release_buffer;
    if (lower_target >= upper_target) {
      return Eigen::VectorXd{};
    }

    double error = 0.0;
    if (joint_position[index] < lower_target - release_tolerance) {
      error = lower_target - joint_position[index];
    } else if (joint_position[index] > upper_target + release_tolerance) {
      error = upper_target - joint_position[index];
    }
    const double velocity_cap =
        std::min(maximum_velocity, 0.5 * velocity_limits[index]);
    recovery[index] = std::clamp(
        gain * error, -velocity_cap, velocity_cap);
  }
  return recovery;
}

RedundantIkResult solveRedundantVelocityIk(
    const Eigen::MatrixXd &jacobian, const Eigen::VectorXd &cartesian_velocity,
    const Eigen::VectorXd &joint_position, const Eigen::VectorXd &lower_limits,
    const Eigen::VectorXd &upper_limits, const Eigen::VectorXd &velocity_limits,
    const Eigen::VectorXd &secondary_velocity,
    const RedundantIkOptions &options) {
  RedundantIkResult result;
  if (!validDimensions(jacobian, cartesian_velocity, joint_position,
                       lower_limits, upper_limits, velocity_limits,
                       secondary_velocity) ||
      !jacobian.allFinite() || !cartesian_velocity.allFinite() ||
      !joint_position.allFinite() || !lower_limits.allFinite() ||
      !upper_limits.allFinite() || !velocity_limits.allFinite()) {
    return result;
  }

  const Eigen::Index joints = jacobian.cols();
  if ((upper_limits.array() <= lower_limits.array()).any() ||
      (velocity_limits.array() <= 0.0).any()) {
    return result;
  }

  if ((options.joint_motion_weights.size() != 0 &&
       options.joint_motion_weights.size() != joints) ||
      (options.joint_limit_margins.size() != 0 &&
       options.joint_limit_margins.size() != joints)) {
    return result;
  }

  Eigen::VectorXd motion_weights = Eigen::VectorXd::Ones(joints);
  if (options.joint_motion_weights.size() == joints) {
    motion_weights = options.joint_motion_weights;
  }
  if (!motion_weights.allFinite() || (motion_weights.array() <= 0.0).any()) {
    return result;
  }

  Eigen::VectorXd margins =
      Eigen::VectorXd::Constant(joints, options.joint_limit_margin);
  if (options.joint_limit_margins.size() == joints) {
    margins = options.joint_limit_margins;
  }
  if (!margins.allFinite() || (margins.array() < 0.0).any()) {
    return result;
  }

  const double activation =
      std::clamp(options.joint_limit_activation_ratio, 0.0, 0.99);
  const double limit_weight = std::max(0.0, options.joint_limit_weight);
  Eigen::VectorXd inverse_weight = Eigen::VectorXd::Ones(joints);
  Eigen::VectorXd centering_velocity = Eigen::VectorXd::Zero(joints);
  for (Eigen::Index index = 0; index < joints; ++index) {
    const double range = upper_limits[index] - lower_limits[index];
    if (2.0 * margins[index] >= range) {
      return result;
    }
    const double safe_lower = lower_limits[index] + margins[index];
    const double safe_upper = upper_limits[index] - margins[index];
    const double midpoint = 0.5 * (safe_lower + safe_upper);
    const double half_range = 0.5 * (safe_upper - safe_lower);
    const double normalized =
        std::clamp((joint_position[index] - midpoint) / half_range, -1.0, 1.0);
    const double magnitude = std::abs(normalized);
    double weight = motion_weights[index];
    if (magnitude > activation) {
      const double transition = (magnitude - activation) / (1.0 - activation);
      weight += limit_weight * transition * transition;
      centering_velocity[index] = -std::max(0.0, options.joint_centering_gain) *
                                  std::copysign(transition, normalized);
    }
    inverse_weight[index] = 1.0 / weight;
  }

  const Eigen::MatrixXd weighted_jacobian =
      jacobian * inverse_weight.cwiseSqrt().asDiagonal();
  const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
      weighted_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (decomposition.singularValues().size() == 0) {
    return result;
  }
  result.minimum_singular_value =
      std::max(0.0, decomposition.singularValues().minCoeff());
  const double maximum_singular_value =
      std::max(0.0, decomposition.singularValues().maxCoeff());
  if (result.minimum_singular_value > 1e-12) {
    result.condition_number =
        maximum_singular_value / result.minimum_singular_value;
  }

  result.damping = adaptiveDamping(decomposition.singularValues(), options);

  Eigen::VectorXd secondary = centering_velocity;
  if (secondary_velocity.size() == joints) {
    if (!secondary_velocity.allFinite()) {
      return result;
    }
    // Singularity escape and other caller-supplied safety objectives outrank
    // routine joint centering. Retain compatible centering motion, but never
    // let it cancel the direction that is meant to recover dexterity.
    const Eigen::VectorXd compatible_centering =
        removeOpposingSecondaryComponent(secondary_velocity,
                                         centering_velocity);
    if (compatible_centering.size() != joints) {
      return result;
    }
    secondary = secondary_velocity + compatible_centering;
  }
  const double maximum_secondary =
      std::max(0.0, options.maximum_secondary_velocity);
  if (maximum_secondary > 0.0 && secondary.norm() > maximum_secondary) {
    secondary *= maximum_secondary / secondary.norm();
  }

  const double requested_velocity_scale =
      std::clamp(options.joint_velocity_scale, 1e-3, 1.0);
  const double lookahead = std::max(1e-3, options.joint_limit_lookahead);
  Eigen::VectorXd physical_velocity_bound(joints);
  Eigen::VectorXd lower_velocity_bound(joints);
  Eigen::VectorXd upper_velocity_bound(joints);
  std::vector<bool> lower_position_limited(static_cast<std::size_t>(joints),
                                           false);
  std::vector<bool> upper_position_limited(static_cast<std::size_t>(joints),
                                           false);
  std::vector<bool> lower_blocked(static_cast<std::size_t>(joints), false);
  std::vector<bool> upper_blocked(static_cast<std::size_t>(joints), false);
  for (Eigen::Index index = 0; index < joints; ++index) {
    const double physical_bound =
        requested_velocity_scale * velocity_limits[index];
    physical_velocity_bound[index] = physical_bound;
    const double safe_lower = lower_limits[index] + margins[index];
    const double safe_upper = upper_limits[index] - margins[index];
    const double lower_position_bound = -std::clamp(
        (joint_position[index] - safe_lower) / lookahead, 0.0, physical_bound);
    const double upper_position_bound = std::clamp(
        (safe_upper - joint_position[index]) / lookahead, 0.0, physical_bound);
    lower_velocity_bound[index] = lower_position_bound;
    upper_velocity_bound[index] = upper_position_bound;
    const std::size_t bounded_index = static_cast<std::size_t>(index);
    lower_position_limited[bounded_index] =
        lower_position_bound > -physical_bound + 1e-12;
    upper_position_limited[bounded_index] =
        upper_position_bound < physical_bound - 1e-12;
    lower_blocked[bounded_index] =
        joint_position[index] <= safe_lower + 1e-12 &&
        lower_position_bound >= -1e-12;
    upper_blocked[bounded_index] =
        joint_position[index] >= safe_upper - 1e-12 &&
        upper_position_bound <= 1e-12;
  }

  const BoundedSolve bounded = solvePrioritizedBounded(
      jacobian, cartesian_velocity, inverse_weight, motion_weights, secondary,
      lower_velocity_bound, upper_velocity_bound, physical_velocity_bound,
      lower_position_limited, upper_position_limited, lower_blocked,
      upper_blocked, options, result.selected_joint_count);
  result.joint_velocity = bounded.joint_velocity;
  for (Eigen::Index index = 0; index < joints; ++index) {
    const std::size_t bounded_index = static_cast<std::size_t>(index);
    result.velocity_limited_joint_count +=
        bounded.velocity_limited[bounded_index] ? 1 : 0;
    result.position_limited_joint_count +=
        bounded.position_limited[bounded_index] ? 1 : 0;
    result.blocked_joint_count += bounded.blocked[bounded_index] ? 1 : 0;
  }
  if (!result.joint_velocity.allFinite()) {
    result.joint_velocity.resize(0);
    return result;
  }
  const Eigen::VectorXd residual =
      cartesian_velocity - jacobian * result.joint_velocity;
  result.linear_residual_norm = residual.head(3).norm();
  result.angular_residual_norm = residual.tail(3).norm();
  result.valid = result.joint_velocity.allFinite();
  return result;
}

} // namespace vive_moveit_server
