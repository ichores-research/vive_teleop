#pragma once

#include "vive_dataset_recorder/recorder_state_machine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vive_dataset_recorder/msg/deadman_frame_state.hpp>
#include <vive_dataset_recorder/msg/recording_event.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace vive_dataset_recorder {

class RecorderController : public rclcpp::Node {
public:
  explicit RecorderController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RecorderController() override;

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node() const {
    return recorder_;
  }
  void start();
  void stop(const std::string &reason);

private:
  using RecordingEvent = vive_dataset_recorder::msg::RecordingEvent;
  using DeadmanFrameState = vive_dataset_recorder::msg::DeadmanFrameState;
  using TimePoint = RecorderStateMachine::TimePoint;

  void validate_configuration();
  void configure_rosbag();
  void on_gate(const std_msgs::msg::Bool::SharedPtr message);
  void on_camera_frame(const sensor_msgs::msg::Image::ConstSharedPtr message);
  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr);
  void on_static_tf(const tf2_msgs::msg::TFMessage::ConstSharedPtr message);
  void on_tick();
  void on_bootstrap_tick();
  void publish_status();
  void execute(const std::vector<RecorderCommand> &commands);
  void publish_event(uint8_t event_type, const std::string &reason,
                     uint64_t window_id, uint64_t segment_id);
  void write_manifest(const std::string &status, const std::string &reason);
  static std::string json_escape(const std::string &value);

  std::string recording_mode_name_;
  std::string capture_gate_topic_;
  std::string camera_topic_;
  std::string frame_state_topic_;
  std::string event_topic_;
  std::string status_topic_;
  std::string output_root_;
  std::string session_id_;
  std::string storage_id_;
  std::string storage_config_uri_;
  std::vector<std::string> record_topics_;
  double post_roll_sec_;
  double gate_stale_timeout_sec_;
  double bootstrap_timeout_sec_;
  int64_t minimum_free_space_bytes_;
  int64_t max_cache_size_bytes_;
  int64_t max_bag_size_bytes_;
  int64_t max_bag_duration_sec_;

  std::filesystem::path session_path_;
  std::filesystem::path bag_path_;
  std::ofstream events_stream_;
  std::unique_ptr<RecorderStateMachine> state_machine_;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::thread recorder_thread_;
  std::mutex recorder_api_mutex_;
  std::atomic<bool> recorder_thread_finished_{false};
  std::atomic<bool> stopping_{false};
  std::string recorder_error_;

  rclcpp::CallbackGroup::SharedPtr controller_callback_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
      static_tf_subscription_;
  rclcpp::Publisher<RecordingEvent>::SharedPtr event_publisher_;
  rclcpp::Publisher<DeadmanFrameState>::SharedPtr frame_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr bootstrap_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  TimePoint started_at_{};
  bool camera_seen_{false};
  bool joint_state_seen_{false};
  bool static_tf_seen_{false};
  bool session_start_emitted_{false};
  uint64_t frame_sequence_{0};
};

} // namespace vive_dataset_recorder
