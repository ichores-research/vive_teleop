#pragma once

#include <Eigen/Core>

#include <limits>

namespace vive_moveit_server {

struct RedundantIkOptions {
  double damping_threshold{0.08};
  double maximum_damping{0.08};
  // Larger values make a joint more expensive. They also define the strict
  // activation order: the lowest-cost joint is tried first and progressively
  // higher-cost joints are admitted only while task residual remains.
  Eigen::VectorXd joint_motion_weights;
  double joint_limit_activation_ratio{0.65};
  double joint_limit_weight{40.0};
  double joint_centering_gain{0.35};
  double maximum_secondary_velocity{0.40};
  double joint_velocity_scale{0.95};
  // Per-joint margins override joint_limit_margin when present. This permits
  // a larger hard safety band on the three wrist joints.
  Eigen::VectorXd joint_limit_margins;
  double joint_limit_margin{0.08};
  double joint_limit_lookahead{0.45};
};

struct RedundantIkResult {
  bool valid{false};
  Eigen::VectorXd joint_velocity;
  double minimum_singular_value{0.0};
  double condition_number{std::numeric_limits<double>::infinity()};
  double damping{0.0};
  // Residuals are measured after every permitted priority level and local
  // bound have been applied.
  double linear_residual_norm{std::numeric_limits<double>::infinity()};
  double angular_residual_norm{std::numeric_limits<double>::infinity()};
  // Number of leading priority levels admitted for the final solution.
  int selected_joint_count{0};
  int velocity_limited_joint_count{0};
  int position_limited_joint_count{0};
  int blocked_joint_count{0};
};

// Orientation-first hierarchical resolved-rate IK. The angular command is the
// primary task; translation is solved only in its null space. Joints are
// admitted in ascending joint_motion_weights order. Per-joint velocity and
// predictive position bounds saturate a constrained joint and leave the
// remaining Cartesian residual for the next joint instead of slowing every
// joint together. Limit barriers, null-space centering, and hard margins keep
// the wrist away from position bounds. secondary_velocity may contain a
// higher-priority configuration-space objective, such as singularity escape;
// internal joint centering is filtered so it cannot oppose that objective.
// Pass an empty vector when no external objective is needed.
RedundantIkResult solveRedundantVelocityIk(
    const Eigen::MatrixXd &jacobian, const Eigen::VectorXd &cartesian_velocity,
    const Eigen::VectorXd &joint_position, const Eigen::VectorXd &lower_limits,
    const Eigen::VectorXd &upper_limits, const Eigen::VectorXd &velocity_limits,
    const Eigen::VectorXd &secondary_velocity,
    const RedundantIkOptions &options);

double minimumSingularValue(const Eigen::MatrixXd &jacobian);

// Returns the segment's vertical displacement divided by its length. Positive
// values point upward in the planning frame; invalid or zero-length segments
// return NaN.
double normalizedUpwardSlope(const Eigen::Vector3d &start,
                             const Eigen::Vector3d &end);

// Smooth one-sided cost for a robot link that points farther upward than the
// configured normalized slope. Horizontal and downward links have zero cost
// for nonnegative activation ratios.
double upwardSegmentPenalty(const Eigen::Vector3d &start,
                            const Eigen::Vector3d &end,
                            double activation_ratio);

// Smooth one-sided cost for elbow height relative to the preceding arm link.
// Every posture at or below desired_low_slope has zero cost; higher postures
// are penalized quadratically. This keeps the elbow low without continuously
// driving the redundant arm configuration toward straight down.
double lowElbowPreferenceCost(const Eigen::Vector3d &start,
                              const Eigen::Vector3d &elbow,
                              double desired_low_slope,
                              double reward_weight);

// Orthogonally projects a joint-space preference into the instantaneous task
// null space. This lets posture objectives recruit all seven joints without
// changing the achievable Cartesian hand velocity.
Eigen::VectorXd projectIntoTaskNullspace(
    const Eigen::MatrixXd &jacobian,
    const Eigen::VectorXd &joint_velocity);

// Removes only the component of candidate that opposes higher_priority.
// Compatible and orthogonal motion is retained, so a lower-priority posture
// objective can help but can never cancel singularity recovery.
Eigen::VectorXd removeOpposingSecondaryComponent(
    const Eigen::VectorXd &higher_priority,
    const Eigen::VectorXd &candidate);

// Produces an inward-only joint command for positions inside their protected
// margin plus release buffer. Safe joints remain stationary. The result is
// empty for invalid inputs and a zero vector when no recovery is needed.
Eigen::VectorXd inwardJointLimitRecoveryVelocity(
    const Eigen::VectorXd &joint_position,
    const Eigen::VectorXd &lower_limits,
    const Eigen::VectorXd &upper_limits,
    const Eigen::VectorXd &velocity_limits,
    const Eigen::VectorXd &margins,
    double release_buffer,
    double gain,
    double maximum_velocity,
    double release_tolerance = 0.005);

} // namespace vive_moveit_server
