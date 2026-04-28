// SPDX-License-Identifier: BSD-3-Clause
// Publishes SteamVR-tracked HMD pose using OpenVR (VRApplication_Background).

#include <cmath>
#include <memory>
#include <string>

#include <openvr.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

geometry_msgs::msg::Pose matrix34ToPose(const vr::HmdMatrix34_t &mat)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = mat.m[0][3];
  pose.position.y = mat.m[1][3];
  pose.position.z = mat.m[2][3];

  tf2::Matrix3x3 R(
    mat.m[0][0], mat.m[0][1], mat.m[0][2],
    mat.m[1][0], mat.m[1][1], mat.m[1][2],
    mat.m[2][0], mat.m[2][1], mat.m[2][2]);
  tf2::Quaternion q;
  R.getRotation(q);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

uint32_t findHmdIndex(vr::IVRSystem *sys)
{
  for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
    if (sys->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD) {
      return i;
    }
  }
  return vr::k_unTrackedDeviceIndexInvalid;
}

}  // namespace

class ViveHeadPoseNode : public rclcpp::Node
{
public:
  ViveHeadPoseNode()
  : rclcpp::Node("vive_head_pose")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "vive_hmd");
    world_frame_ = declare_parameter<std::string>("world_frame", "vive_world");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/vive/head_pose");
    tracking_universe_ = declare_parameter<std::string>("tracking_universe", "standing");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    rate_hz_ = declare_parameter<double>("rate_hz", 90.0);

    vr::EVRInitError err = vr::VRInitError_None;
    vr_system_ = vr::VR_Init(&err, vr::VRApplication_Background);
    if (err != vr::VRInitError_None || !vr_system_) {
      RCLCPP_FATAL(
        get_logger(), "OpenVR init failed: %s",
        vr::VR_GetVRInitErrorAsEnglishDescription(err));
      throw std::runtime_error("VR_Init failed");
    }

    hmd_index_ = findHmdIndex(vr_system_);
    if (hmd_index_ == vr::k_unTrackedDeviceIndexInvalid) {
      RCLCPP_WARN(get_logger(), "No HMD device index yet; will retry in timer.");
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ViveHeadPoseNode::onTimer, this));
  }

  ~ViveHeadPoseNode() override
  {
    if (vr_system_) {
      vr::VR_Shutdown();
      vr_system_ = nullptr;
    }
  }

private:
  void onTimer()
  {
    if (!vr_system_) {
      return;
    }
    if (hmd_index_ == vr::k_unTrackedDeviceIndexInvalid) {
      hmd_index_ = findHmdIndex(vr_system_);
      if (hmd_index_ == vr::k_unTrackedDeviceIndexInvalid) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Found HMD at device index %u", hmd_index_);
    }

    vr::ETrackingUniverseOrigin universe = vr::TrackingUniverseStanding;
    if (tracking_universe_ == "seated") {
      universe = vr::TrackingUniverseSeated;
    } else if (tracking_universe_ == "raw") {
      universe = vr::TrackingUniverseRawAndUncalibrated;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr_system_->GetDeviceToAbsoluteTrackingPose(universe, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);

    const auto &p = poses[hmd_index_];
    if (!p.bPoseIsValid || !p.bDeviceIsConnected) {
      return;
    }

    auto pose_msg = matrix34ToPose(p.mDeviceToAbsoluteTracking);
    auto stamped = geometry_msgs::msg::PoseStamped();
    stamped.header.stamp = now();
    stamped.header.frame_id = world_frame_;
    stamped.pose = pose_msg;

    pose_pub_->publish(stamped);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = stamped.header;
      tf.child_frame_id = frame_id_;
      tf.transform.translation.x = pose_msg.position.x;
      tf.transform.translation.y = pose_msg.position.y;
      tf.transform.translation.z = pose_msg.position.z;
      tf.transform.rotation = pose_msg.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::string frame_id_;
  std::string world_frame_;
  std::string pose_topic_;
  std::string tracking_universe_;
  bool publish_tf_{true};
  double rate_hz_{90.0};

  vr::IVRSystem *vr_system_{nullptr};
  uint32_t hmd_index_{vr::k_unTrackedDeviceIndexInvalid};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ViveHeadPoseNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("vive_head_pose"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
