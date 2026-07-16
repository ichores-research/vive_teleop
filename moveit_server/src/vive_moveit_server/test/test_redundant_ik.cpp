#include "vive_moveit_server/redundant_ik.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>

namespace vive_moveit_server {
namespace {

constexpr double kTolerance = 1e-7;

RedundantIkOptions testOptions() {
  RedundantIkOptions options;
  options.damping_threshold = 0.0;
  options.maximum_damping = 0.0;
  options.joint_limit_activation_ratio = 0.5;
  options.joint_limit_weight = 100.0;
  options.joint_centering_gain = 0.0;
  options.maximum_secondary_velocity = 1.0;
  options.joint_velocity_scale = 1.0;
  options.joint_limit_margin = 0.0;
  options.joint_limit_lookahead = 0.25;
  options.joint_motion_weights.resize(7);
  // Strict activation order: J7 -> J6 -> J5 -> J3 -> J2 -> J4 -> J1.
  options.joint_motion_weights << 12.0, 3.0, 2.0, 4.0, 1.0, 0.5, 0.25;
  return options;
}

TEST(VisibilityObjective, PenalizesOnlySegmentsAboveTheUpwardThreshold) {
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  EXPECT_DOUBLE_EQ(
      upwardSegmentPenalty(origin, Eigen::Vector3d(1.0, 0.0, 0.0), 0.10),
      0.0);
  EXPECT_DOUBLE_EQ(
      upwardSegmentPenalty(origin, Eigen::Vector3d(0.4, 0.0, -0.3), 0.10),
      0.0);
  EXPECT_DOUBLE_EQ(
      upwardSegmentPenalty(origin, Eigen::Vector3d(1.0, 0.0, 0.05), 0.10),
      0.0);

  const double vertical_penalty =
      upwardSegmentPenalty(origin, Eigen::Vector3d(0.0, 0.0, 1.0), 0.10);
  EXPECT_NEAR(vertical_penalty, 0.5 * 0.9 * 0.9, kTolerance);
  EXPECT_NEAR(
      vertical_penalty,
      upwardSegmentPenalty(origin, Eigen::Vector3d(0.0, 0.0, 0.2), 0.10),
      kTolerance);
}

TEST(VisibilityObjective, RewardsALowElbowWithoutDrivingPastTheTarget) {
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  constexpr double target = -0.20;
  constexpr double weight = 0.40;
  EXPECT_NEAR(
      lowElbowPreferenceCost(
          origin, Eigen::Vector3d(0.0, 0.0, -1.0), target, weight),
      0.0, kTolerance);
  EXPECT_NEAR(
      lowElbowPreferenceCost(
          origin, Eigen::Vector3d(0.8, 0.0, -0.6), target, weight),
      0.0, kTolerance);
  EXPECT_NEAR(
      lowElbowPreferenceCost(
          origin, Eigen::Vector3d(1.0, 0.0, 0.0), target, weight),
      0.5 * weight * 0.2 * 0.2, kTolerance);
  EXPECT_NEAR(
      lowElbowPreferenceCost(
          origin, Eigen::Vector3d(0.0, 0.0, 1.0), target, weight),
      0.5 * weight * 1.2 * 1.2, kTolerance);
}

TEST(VisibilityObjective, RejectsInvalidOrZeroLengthSegments) {
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  EXPECT_TRUE(std::isnan(normalizedUpwardSlope(origin, origin)));
  EXPECT_DOUBLE_EQ(upwardSegmentPenalty(origin, origin, 0.10), 0.0);
  EXPECT_DOUBLE_EQ(lowElbowPreferenceCost(origin, origin, -0.20, 0.40), 0.0);

  Eigen::Vector3d invalid = origin;
  invalid.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isnan(normalizedUpwardSlope(origin, invalid)));
  EXPECT_DOUBLE_EQ(upwardSegmentPenalty(origin, invalid, 0.10), 0.0);
  EXPECT_DOUBLE_EQ(
      lowElbowPreferenceCost(origin, invalid, -0.20, 0.40), 0.0);
  EXPECT_DOUBLE_EQ(
      lowElbowPreferenceCost(
          origin, Eigen::Vector3d::UnitZ(),
          std::numeric_limits<double>::quiet_NaN(), 0.40),
      0.0);
}

TEST(VisibilityObjective, ProjectsPostureMotionWithoutChangingTheTask) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.leftCols(6).setIdentity();
  jacobian.col(6).setOnes();
  Eigen::VectorXd preference = Eigen::VectorXd::Zero(7);
  preference[2] = 1.0;

  const Eigen::VectorXd projected =
      projectIntoTaskNullspace(jacobian, preference);

  ASSERT_EQ(projected.size(), 7);
  EXPECT_LT((jacobian * projected).norm(), kTolerance);
  EXPECT_GT(projected.norm(), 0.1);
  EXPECT_GT(std::abs(projected[6]), 0.01);
  EXPECT_GT(preference.dot(projected), 0.0);
}

TEST(VisibilityObjective, CannotCancelHigherPrioritySingularityRecovery) {
  Eigen::VectorXd singularity_recovery(3);
  singularity_recovery << 1.0, 0.0, 0.0;
  Eigen::VectorXd opposing_visibility(3);
  opposing_visibility << -2.0, 3.0, 0.0;
  const Eigen::VectorXd filtered = removeOpposingSecondaryComponent(
      singularity_recovery, opposing_visibility);

  ASSERT_EQ(filtered.size(), 3);
  EXPECT_NEAR(filtered[0], 0.0, kTolerance);
  EXPECT_NEAR(filtered[1], 3.0, kTolerance);
  EXPECT_GE(filtered.dot(singularity_recovery), -kTolerance);

  Eigen::VectorXd compatible_visibility(3);
  compatible_visibility << 2.0, 3.0, 0.0;
  EXPECT_TRUE(removeOpposingSecondaryComponent(
                  singularity_recovery, compatible_visibility)
                  .isApprox(compatible_visibility, kTolerance));
}

TEST(JointLimitRecovery, MovesOnlyInwardAndHonorsVelocityCaps) {
  Eigen::VectorXd position(3);
  position << -0.90, 0.95, 0.0;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(3, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(3, 1.0);
  Eigen::VectorXd velocity(3);
  velocity << 1.0, 0.20, 1.0;
  const Eigen::VectorXd margins = Eigen::VectorXd::Constant(3, 0.20);

  const Eigen::VectorXd recovery = inwardJointLimitRecoveryVelocity(
      position, lower, upper, velocity, margins,
      0.03, 2.0, 0.18);

  ASSERT_EQ(recovery.size(), 3);
  EXPECT_GT(recovery[0], 0.0);
  EXPECT_NEAR(recovery[0], 0.18, kTolerance);
  EXPECT_LT(recovery[1], 0.0);
  EXPECT_NEAR(recovery[1], -0.10, kTolerance);
  EXPECT_NEAR(recovery[2], 0.0, kTolerance);
}

TEST(JointLimitRecovery, ReturnsZeroAfterProtectedClearanceIsRestored) {
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(3, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(3, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(3);
  const Eigen::VectorXd margins = Eigen::VectorXd::Constant(3, 0.20);

  const Eigen::VectorXd recovery = inwardJointLimitRecoveryVelocity(
      position, lower, upper, velocity, margins,
      0.03, 2.0, 0.18);

  ASSERT_EQ(recovery.size(), 3);
  EXPECT_TRUE(recovery.isZero(kTolerance));
}

TEST(VisibilityObjective, SolverKeepsTrackingWhileApplyingFullArmBias) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.leftCols(6).setIdentity();
  jacobian.col(6).setOnes();
  Eigen::VectorXd preference = Eigen::VectorXd::Zero(7);
  preference[2] = 0.25;
  const Eigen::VectorXd secondary =
      projectIntoTaskNullspace(jacobian, preference);
  Eigen::VectorXd cartesian(6);
  cartesian << 0.10, -0.05, 0.02, 0.03, -0.04, 0.06;
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -2.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 2.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);

  const RedundantIkResult result = solveRedundantVelocityIk(
      jacobian, cartesian, position, lower, upper, velocity, secondary,
      testOptions());
  Eigen::VectorXd depth_only = Eigen::VectorXd::Zero(7);
  // Force the same seven-joint selection without materially changing its
  // primary solution, so the difference isolates the posture objective.
  depth_only[0] = 1e-10;
  const RedundantIkResult baseline = solveRedundantVelocityIk(
      jacobian, cartesian, position, lower, upper, velocity, depth_only,
      testOptions());

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(baseline.valid);
  EXPECT_EQ(result.selected_joint_count, 7);
  EXPECT_EQ(baseline.selected_joint_count, 7);
  EXPECT_TRUE((jacobian * result.joint_velocity).isApprox(cartesian,
                                                          kTolerance));
  EXPECT_GT(
      secondary.dot(result.joint_velocity - baseline.joint_velocity), 0.0);
}

TEST(RedundantIk, UsesRedundantJointInsteadOfJointNearLimit) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.leftCols(6).setIdentity();
  jacobian(0, 6) = 1.0;
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[0] = 0.5;
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  position[0] = 0.95;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, testOptions());

  ASSERT_TRUE(result.valid);
  EXPECT_LT(std::abs(result.joint_velocity[0]), 0.02);
  EXPECT_GT(result.joint_velocity[6], 0.48);
  EXPECT_TRUE((jacobian * result.joint_velocity).isApprox(cartesian, 2e-4));
}

TEST(RedundantIk, PreservesOrientationWhenTranslationConflicts) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian(3, 0) = 1.0;
  jacobian(4, 1) = 1.0;
  jacobian(5, 2) = 1.0;
  // Linear X can only use the same joint as angular X, so its request cannot
  // be met without corrupting the primary orientation task.
  jacobian(0, 0) = 1.0;
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[0] = -0.5;
  cartesian[3] = 0.5;
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -2.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 2.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, testOptions());

  ASSERT_TRUE(result.valid);
  const Eigen::VectorXd achieved = jacobian * result.joint_velocity;
  EXPECT_NEAR(achieved[3], cartesian[3], kTolerance);
  EXPECT_NE(achieved[0], cartesian[0]);
}

TEST(RedundantIk, PrefersJointSevenForToolAxisRotation) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian(0, 2) = 1.0;
  jacobian(1, 3) = 1.0;
  jacobian(2, 5) = 1.0;
  jacobian(3, 1) = 1.0;
  jacobian(4, 4) = 1.0;
  jacobian(5, 0) = 1.0;
  jacobian(5, 6) = 1.0;
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[5] = 0.5;
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -2.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 2.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);
  RedundantIkOptions options = testOptions();

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, options);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_joint_count, 1);
  EXPECT_GT(result.joint_velocity[6], 0.48);
  EXPECT_GT(result.joint_velocity[6], 40.0 * result.joint_velocity[0]);
  EXPECT_NEAR((jacobian * result.joint_velocity)[5], 0.5, kTolerance);
}

TEST(RedundantIk, SaturatesAndFallsThroughPriorityOneJointAtATime) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian(5, 6) = 1.0;
  jacobian(5, 5) = 1.0;
  jacobian(5, 4) = 1.0;
  jacobian(5, 2) = 1.0;
  jacobian(5, 1) = 1.0;
  jacobian(5, 3) = 1.0;
  jacobian(5, 0) = 1.0;
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[5] = 0.7;
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -2.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 2.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Constant(7, 0.2);

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, testOptions());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_joint_count, 4);
  // J7 and J6 require active clamping; J5 lands exactly on its bound.
  EXPECT_EQ(result.velocity_limited_joint_count, 2);
  EXPECT_NEAR(result.joint_velocity[6], 0.2, kTolerance);
  EXPECT_NEAR(result.joint_velocity[5], 0.2, kTolerance);
  EXPECT_NEAR(result.joint_velocity[4], 0.2, kTolerance);
  EXPECT_NEAR(result.joint_velocity[2], 0.1, kTolerance);
  EXPECT_NEAR(result.joint_velocity[1], 0.0, kTolerance);
  EXPECT_NEAR(result.joint_velocity[3], 0.0, kTolerance);
  EXPECT_NEAR(result.joint_velocity[0], 0.0, kTolerance);
  EXPECT_NEAR((jacobian * result.joint_velocity)[5], 0.7, kTolerance);
}

TEST(RedundantIk, ActivatesRequestedJointHierarchyWithoutUsingJointOne) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.col(0).setOnes();
  jacobian(0, 2) = 1.0; // J3: first translation fallback.
  jacobian(1, 1) = 1.0; // J2: second translation fallback.
  jacobian(2, 3) = 1.0; // J4: third translation fallback.
  jacobian(3, 4) = 1.0; // J5: first non-roll wrist fallback.
  jacobian(4, 5) = 1.0; // J6.
  jacobian(5, 6) = 1.0; // J7.
  Eigen::VectorXd cartesian(6);
  cartesian << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -2.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 2.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, testOptions());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_joint_count, 6);
  EXPECT_NEAR(result.joint_velocity[6], 0.60, kTolerance);
  EXPECT_NEAR(result.joint_velocity[5], 0.50, kTolerance);
  EXPECT_NEAR(result.joint_velocity[4], 0.40, kTolerance);
  EXPECT_NEAR(result.joint_velocity[2], 0.10, kTolerance);
  EXPECT_NEAR(result.joint_velocity[1], 0.20, kTolerance);
  EXPECT_NEAR(result.joint_velocity[3], 0.30, kTolerance);
  EXPECT_NEAR(result.joint_velocity[0], 0.0, kTolerance);
  EXPECT_TRUE(
      (jacobian * result.joint_velocity).isApprox(cartesian, kTolerance));
}

TEST(RedundantIk, RedistributesBeforeThePointTwoRadianWristMargin) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian(0, 2) = 1.0;
  jacobian(1, 3) = 1.0;
  jacobian(2, 5) = 1.0;
  jacobian(3, 0) = 1.0;
  jacobian(4, 1) = 1.0;
  jacobian(5, 4) = 1.0;
  jacobian(5, 6) = 1.0;
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[5] = 0.4;
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  position[6] = 0.85;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);
  RedundantIkOptions options = testOptions();
  options.joint_limit_margins.resize(7);
  options.joint_limit_margins << 0.08, 0.08, 0.08, 0.10, 0.20, 0.20, 0.20;

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, options);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.blocked_joint_count, 1);
  EXPECT_NEAR(result.joint_velocity[6], 0.0, kTolerance);
  EXPECT_NEAR(result.joint_velocity[4], 0.4, kTolerance);
  EXPECT_NEAR((jacobian * result.joint_velocity)[5], 0.4, kTolerance);
}

TEST(RedundantIk, AllowsRecoveryFromTheProtectedWristBand) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian(3, 0) = 1.0;
  jacobian(4, 1) = 1.0;
  jacobian(5, 4) = 1.0;
  jacobian(5, 6) = 1.0;
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[5] = -0.4;
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  position[6] = 0.85;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);
  RedundantIkOptions options = testOptions();
  options.joint_limit_margins.resize(7);
  options.joint_limit_margins << 0.08, 0.08, 0.08, 0.10, 0.20, 0.20, 0.20;

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, options);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.blocked_joint_count, 0);
  EXPECT_LT(result.joint_velocity[6], -0.001);
  EXPECT_NEAR((jacobian * result.joint_velocity)[5], -0.4, kTolerance);
}

TEST(RedundantIk, CentersAFreeJointInTheJacobianNullspace) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.leftCols(6).setIdentity();
  const Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  position[6] = 0.8;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);
  RedundantIkOptions options = testOptions();
  options.joint_centering_gain = 0.5;

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, options);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE((jacobian * result.joint_velocity).isZero(kTolerance));
  EXPECT_LT(result.joint_velocity[6], -0.29);
}

TEST(RedundantIk, ExternalSecondaryTakesPriorityOverJointCentering) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.leftCols(6).setIdentity();
  const Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  position[6] = 0.8;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);
  Eigen::VectorXd singularity_escape = Eigen::VectorXd::Zero(7);
  singularity_escape[6] = 0.5;
  RedundantIkOptions options = testOptions();
  options.joint_centering_gain = 0.5;

  const RedundantIkResult result = solveRedundantVelocityIk(
      jacobian, cartesian, position, lower, upper, velocity,
      singularity_escape, options);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE((jacobian * result.joint_velocity).isZero(kTolerance));
  EXPECT_NEAR(result.joint_velocity[6], singularity_escape[6], kTolerance);
}

TEST(RedundantIk, AddsDampingAtASingularityWithoutProducingNan) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.topLeftCorner(5, 5).setIdentity();
  Eigen::VectorXd cartesian = Eigen::VectorXd::Ones(6);
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);
  RedundantIkOptions options = testOptions();
  options.damping_threshold = 0.1;
  options.maximum_damping = 0.08;

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, options);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.damping, 0.079);
  EXPECT_TRUE(result.joint_velocity.allFinite());
  EXPECT_TRUE(std::isinf(result.condition_number));
}

TEST(RedundantIk, SaturatesPreferredJointsAndPreservesSpeedWithFallbacks) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian(0, 6) = 1.0; // J7 reaches its predictive position bound.
  jacobian(0, 2) = 1.0; // J3 reaches its velocity bound.
  jacobian(0, 1) = 1.0; // J2 supplies the remaining command.
  Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  cartesian[0] = 0.5;
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  position[6] = 0.89;
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(7, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(7, 1.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Constant(7, 0.5);
  RedundantIkOptions options = testOptions();
  options.joint_limit_weight = 0.0;
  options.joint_velocity_scale = 0.5;
  options.joint_limit_margin = 0.1;
  options.joint_limit_lookahead = 0.25;

  const RedundantIkResult result =
      solveRedundantVelocityIk(jacobian, cartesian, position, lower, upper,
                               velocity, Eigen::VectorXd{}, options);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.position_limited_joint_count, 1);
  EXPECT_EQ(result.velocity_limited_joint_count, 1);
  EXPECT_NEAR(result.joint_velocity[6], 0.04, kTolerance);
  EXPECT_NEAR(result.joint_velocity[2], 0.25, kTolerance);
  EXPECT_NEAR(result.joint_velocity[1], 0.21, kTolerance);
  EXPECT_NEAR((jacobian * result.joint_velocity)[0], 0.5, kTolerance);
  EXPECT_NEAR(result.linear_residual_norm, 0.0, kTolerance);
}

TEST(RedundantIk, RejectsMismatchedOrInvalidLimits) {
  const Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  const Eigen::VectorXd cartesian = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd position = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd bad_lower = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd bad_upper = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Ones(7);

  EXPECT_FALSE(solveRedundantVelocityIk(jacobian, cartesian, position,
                                        bad_lower, bad_upper, velocity,
                                        Eigen::VectorXd{}, testOptions())
                   .valid);
}

} // namespace
} // namespace vive_moveit_server
