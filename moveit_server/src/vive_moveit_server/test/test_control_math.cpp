#include "vive_moveit_server/control_math.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

namespace vive_moveit_server
{
namespace
{

constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;

TEST(ControlMath, NormalizesHugeFiniteQuaternionWithoutOverflow)
{
  Eigen::Quaterniond quaternion(1e308, 1e308, 1e308, 1e308);
  ASSERT_TRUE(normalizeQuaternion(quaternion));
  EXPECT_NEAR(quaternion.w(), 0.5, kTolerance);
  EXPECT_NEAR(quaternion.x(), 0.5, kTolerance);
  EXPECT_NEAR(quaternion.y(), 0.5, kTolerance);
  EXPECT_NEAR(quaternion.z(), 0.5, kTolerance);
}

TEST(ControlMath, RejectsNonFiniteQuaternion)
{
  Eigen::Quaterniond quaternion(
    1.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_FALSE(normalizeQuaternion(quaternion));
}

TEST(ControlMath, MapsTranslationOneToOneFromLiveToolAnchor)
{
  CartesianPose controller_anchor;
  controller_anchor.position = Eigen::Vector3d(10.0, 20.0, 30.0);
  CartesianPose controller = controller_anchor;
  controller.position = Eigen::Vector3d(10.1, 19.7, 30.2);
  CartesianPose tool_anchor;
  tool_anchor.position = Eigen::Vector3d(0.5, -0.2, 0.8);

  const CartesianPose target = mapControllerDeltaToTool(
    controller, controller_anchor, tool_anchor, Eigen::Vector3d::Ones());

  EXPECT_TRUE(target.position.isApprox(Eigen::Vector3d(0.6, -0.5, 1.0), kTolerance));
}

TEST(ControlMath, PositionScaleChangesOnlyControllerDisplacement)
{
  CartesianPose controller_anchor;
  controller_anchor.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  CartesianPose controller = controller_anchor;
  controller.position += Eigen::Vector3d(0.1, -0.2, 0.3);
  CartesianPose tool_anchor;
  tool_anchor.position = Eigen::Vector3d(0.5, 0.0, 0.8);

  const CartesianPose target = mapControllerDeltaToTool(
    controller,
    controller_anchor,
    tool_anchor,
    Eigen::Vector3d(2.0, 0.5, -1.0));

  EXPECT_TRUE(target.position.isApprox(Eigen::Vector3d(0.7, -0.1, 0.5), kTolerance));
}

TEST(ControlMath, ControllerTopRotationIsAppliedInToolLocalFrame)
{
  CartesianPose controller_anchor;
  CartesianPose controller;
  controller.orientation = Eigen::AngleAxisd(
    kPi / 2.0, Eigen::Vector3d::UnitX());
  CartesianPose tool_anchor;
  tool_anchor.orientation = Eigen::AngleAxisd(
    kPi / 2.0, Eigen::Vector3d::UnitZ());

  const CartesianPose target = mapControllerDeltaToTool(
    controller, controller_anchor, tool_anchor, Eigen::Vector3d::Ones());
  const Eigen::Quaterniond expected =
    tool_anchor.orientation * controller.orientation;

  EXPECT_LT(target.orientation.angularDistance(expected), kTolerance);
}

TEST(ControlMath, ControllerAlignmentDefinesVirtualToolAxes)
{
  CartesianPose anchor;
  CartesianPose current;
  current.orientation = Eigen::AngleAxisd(
    kPi / 3.0, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond alignment(Eigen::AngleAxisd(
    kPi / 2.0, Eigen::Vector3d::UnitZ()));
  const CartesianPose aligned_anchor = controllerControlPose(
    anchor, Eigen::Vector3d::Zero(), alignment);
  const CartesianPose aligned_current = controllerControlPose(
    current, Eigen::Vector3d::Zero(), alignment);
  CartesianPose tool_anchor;

  const CartesianPose target = mapControllerDeltaToTool(
    aligned_current,
    aligned_anchor,
    tool_anchor,
    Eigen::Vector3d::Ones());
  const Eigen::Quaterniond expected =
    alignment.conjugate() * current.orientation * alignment;

  EXPECT_LT(target.orientation.angularDistance(expected), kTolerance);
}

TEST(ControlMath, ControllerTopOffsetMakesRotationMoveTheControlPoint)
{
  CartesianPose anchor;
  CartesianPose current;
  current.orientation = Eigen::AngleAxisd(
    kPi / 2.0, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d top_offset(0.1, 0.0, 0.0);
  const CartesianPose top_anchor = controllerControlPose(
    anchor, top_offset, Eigen::Quaterniond::Identity());
  const CartesianPose top_current = controllerControlPose(
    current, top_offset, Eigen::Quaterniond::Identity());

  EXPECT_TRUE(top_anchor.position.isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), kTolerance));
  EXPECT_TRUE(top_current.position.isApprox(Eigen::Vector3d(0.0, 0.1, 0.0), kTolerance));
}

TEST(ControlMath, OrientationErrorUsesShortestQuaternionPath)
{
  const Eigen::Quaterniond target(
    -std::cos(kPi / 4.0), 0.0, 0.0, std::sin(kPi / 4.0));
  const Eigen::Vector3d error = orientationError(
    target, Eigen::Quaterniond::Identity());
  EXPECT_TRUE(error.isApprox(Eigen::Vector3d(0.0, 0.0, -kPi / 2.0), kTolerance));
}

TEST(ControlMath, ClampVectorPreservesDirection)
{
  const Eigen::Vector3d result = clampVectorNorm(Eigen::Vector3d(3.0, 4.0, 0.0), 2.0);
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d(1.2, 1.6, 0.0), kTolerance));
}

TEST(ControlMath, WorkspaceConstraintPreservesSphereAfterMinimumZClamp)
{
  Eigen::Vector3d position(3.0, 0.0, 0.0);

  ASSERT_TRUE(constrainWorkspacePosition(position, 1.5, 0.2, 1.6));

  EXPECT_NEAR(position.z(), 0.2, kTolerance);
  EXPECT_NEAR(position.norm(), 1.5, kTolerance);
}

TEST(ControlMath, WorkspaceConstraintRejectsDisjointLimits)
{
  Eigen::Vector3d position(0.0, 0.0, 2.5);

  EXPECT_FALSE(constrainWorkspacePosition(position, 1.5, 2.0, 3.0));
}

TEST(ControlMath, PoseFeedbackUsesVectorDeadband)
{
  EXPECT_TRUE(poseFeedback(
    Eigen::Vector3d(0.0005, 0.0005, 0.0005), 5.0, 0.001).isZero());
  EXPECT_TRUE(poseFeedback(
    Eigen::Vector3d(0.001, 0.001, 0.0), 5.0, 0.001)
      .isApprox(Eigen::Vector3d(0.005, 0.005, 0.0), kTolerance));
}

TEST(ControlMath, RateLimitBoundsCartesianAcceleration)
{
  const Eigen::Vector3d result = rateLimitVector(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0), 2.0, 0.01);
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d(0.02, 0.0, 0.0), kTolerance));
}

TEST(ControlMath, BaseSlewUsesSeparateAccelerationAndDeceleration)
{
  EXPECT_NEAR(approachVelocity(0.0, 1.0, 0.5, 1.0, 0.1), 0.05, kTolerance);
  EXPECT_NEAR(approachVelocity(0.5, 0.0, 0.5, 1.0, 0.1), 0.4, kTolerance);
}

}  // namespace
}  // namespace vive_moveit_server
