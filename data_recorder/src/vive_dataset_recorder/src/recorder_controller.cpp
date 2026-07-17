#include "vive_dataset_recorder/recorder_controller.hpp"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unordered_set>
#include <utility>

namespace vive_dataset_recorder {
namespace {
using namespace std::chrono_literals;

std::chrono::nanoseconds seconds_to_duration(double value) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(value));
}

std::string environment_or_empty(const char *name) {
  const char *value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string{value};
}

} // namespace

RecorderController::RecorderController(const rclcpp::NodeOptions &options)
    : Node("vive_dataset_recorder", options) {
  recording_mode_name_ =
      declare_parameter<std::string>("recording_mode", "deadman_window");
  capture_gate_topic_ = declare_parameter<std::string>(
      "capture_gate_topic", "/vive/hand_target_active");
  camera_topic_ = declare_parameter<std::string>(
      "camera_topic", "/head_front_camera/rgb/image_raw");
  frame_state_topic_ = declare_parameter<std::string>(
      "frame_state_topic", "/teleop/recording/deadman_frame_state");
  event_topic_ =
      declare_parameter<std::string>("event_topic", "/teleop/recording/events");
  status_topic_ = declare_parameter<std::string>("status_topic",
                                                 "/teleop/recording/status");
  output_root_ = declare_parameter<std::string>("output_root", "/recordings");
  session_id_ = declare_parameter<std::string>(
      "session_id", environment_or_empty("VIVE_TELEOP_SESSION_ID"));
  storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
  storage_config_uri_ =
      declare_parameter<std::string>("storage_config_uri", "");
  post_roll_sec_ = declare_parameter<double>("post_roll_sec", 0.75);
  gate_stale_timeout_sec_ =
      declare_parameter<double>("capture_gate_stale_timeout_sec", 0.25);
  bootstrap_timeout_sec_ =
      declare_parameter<double>("bootstrap_timeout_sec", 30.0);
  minimum_free_space_bytes_ =
      declare_parameter<int64_t>("minimum_free_space_bytes", 20000000000LL);
  max_cache_size_bytes_ =
      declare_parameter<int64_t>("max_cache_size_bytes", 104857600LL);
  max_bag_size_bytes_ = declare_parameter<int64_t>("max_bag_size_bytes", 0);
  max_bag_duration_sec_ = declare_parameter<int64_t>("max_bag_duration_sec", 0);
  record_topics_ = declare_parameter<std::vector<std::string>>(
      "record_topics", std::vector<std::string>{});

  validate_configuration();
  state_machine_ = std::make_unique<RecorderStateMachine>(
      parse_recording_mode(recording_mode_name_),
      seconds_to_duration(post_roll_sec_),
      seconds_to_duration(gate_stale_timeout_sec_));

  session_path_ = std::filesystem::path(output_root_) / session_id_;
  bag_path_ = session_path_ / "bag";
  std::filesystem::create_directories(session_path_);
  events_stream_.open(session_path_ / "events.jsonl",
                      std::ios::out | std::ios::app);
  if (!events_stream_) {
    throw std::runtime_error("cannot open recorder event index");
  }

  controller_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = controller_callback_group_;

  event_publisher_ = create_publisher<RecordingEvent>(
      event_topic_, rclcpp::QoS(100).reliable());
  frame_state_publisher_ = create_publisher<DeadmanFrameState>(
      frame_state_topic_, rclcpp::QoS(100).reliable());
  status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
  gate_subscription_ = create_subscription<std_msgs::msg::Bool>(
      capture_gate_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&RecorderController::on_gate, this, std::placeholders::_1),
      subscription_options);
  camera_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RecorderController::on_camera_frame, this,
                std::placeholders::_1),
      subscription_options);
  joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&RecorderController::on_joint_state, this,
                std::placeholders::_1),
      subscription_options);
  static_tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&RecorderController::on_static_tf, this, std::placeholders::_1),
      subscription_options);

  configure_rosbag();
  write_manifest("starting", "");
}

RecorderController::~RecorderController() {
  try {
    stop("destructor");
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Recorder shutdown failed: %s", error.what());
  }
}

void RecorderController::validate_configuration() {
  (void)parse_recording_mode(recording_mode_name_);
  if (session_id_.empty() || session_id_.find('/') != std::string::npos ||
      session_id_.find("..") != std::string::npos) {
    throw std::invalid_argument("session_id must be non-empty and path-safe");
  }
  if (record_topics_.empty()) {
    throw std::invalid_argument(
        "record_topics must be an explicit non-empty whitelist");
  }
  if (post_roll_sec_ < 0.0 || gate_stale_timeout_sec_ <= 0.0 ||
      bootstrap_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("recorder timeouts are invalid");
  }
  if (minimum_free_space_bytes_ < 0 || max_cache_size_bytes_ < 0 ||
      max_bag_size_bytes_ < 0 || max_bag_duration_sec_ < 0) {
    throw std::invalid_argument("recorder size limits cannot be negative");
  }
  const std::unordered_set<std::string> unique_topics(record_topics_.begin(),
                                                      record_topics_.end());
  if (unique_topics.size() != record_topics_.size()) {
    throw std::invalid_argument("record_topics contains duplicates");
  }
  for (const auto &topic : record_topics_) {
    if (topic.empty() || topic.front() != '/') {
      throw std::invalid_argument(
          "every record topic must be an absolute ROS topic");
    }
    if (topic.rfind("/vive/", 0) == 0 && topic != capture_gate_topic_) {
      throw std::invalid_argument("record_topics may not contain operator "
                                  "input other than the deadman topic: " +
                                  topic);
    }
  }
  for (const auto &required_topic :
       {camera_topic_, capture_gate_topic_, frame_state_topic_, event_topic_}) {
    if (unique_topics.count(required_topic) == 0) {
      throw std::invalid_argument("record_topics is missing required topic " +
                                  required_topic);
    }
  }

  const auto output = std::filesystem::path(output_root_);
  std::filesystem::create_directories(output);
  const auto space = std::filesystem::space(output);
  if (space.available < static_cast<uintmax_t>(minimum_free_space_bytes_)) {
    throw std::runtime_error(
        "recording output has less than minimum_free_space_bytes available");
  }
  const auto session = output / session_id_;
  if (std::filesystem::exists(session)) {
    throw std::runtime_error("recording session directory already exists: " +
                             session.string());
  }
}

void RecorderController::configure_rosbag() {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path_.string();
  storage_options.storage_id = storage_id_;
  storage_options.max_bagfile_size = static_cast<uint64_t>(max_bag_size_bytes_);
  storage_options.max_bagfile_duration =
      static_cast<uint64_t>(max_bag_duration_sec_);
  storage_options.max_cache_size = static_cast<uint64_t>(max_cache_size_bytes_);
  storage_options.storage_config_uri = storage_config_uri_;
  storage_options.snapshot_mode = false;

  rosbag2_transport::RecordOptions record_options;
  record_options.all = false;
  record_options.is_discovery_disabled = false;
  record_options.topics = record_topics_;
  record_options.rmw_serialization_format = "cdr";
  record_options.topic_polling_interval = 100ms;
  record_options.include_hidden_topics = false;
  record_options.include_unpublished_topics = true;
  record_options.ignore_leaf_topics = false;
  record_options.start_paused = false;
  auto &qos_overrides = record_options.topic_qos_profile_overrides;
  qos_overrides.insert_or_assign(camera_topic_, rclcpp::SensorDataQoS());
  qos_overrides.insert_or_assign("/head_front_camera/rgb/camera_info",
                                 rclcpp::SensorDataQoS());
  qos_overrides.insert_or_assign("/joint_states", rclcpp::SensorDataQoS());
  qos_overrides.insert_or_assign("/tf", rclcpp::QoS(100).best_effort());
  qos_overrides.insert_or_assign(
      "/tf_static", rclcpp::QoS(1).reliable().transient_local());
  qos_overrides.insert_or_assign(frame_state_topic_,
                                 rclcpp::QoS(100).reliable());
  qos_overrides.insert_or_assign(event_topic_, rclcpp::QoS(100).reliable());

  auto writer = std::make_shared<rosbag2_cpp::Writer>();
  recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      writer, storage_options, record_options, "vive_rosbag2_recorder");
}

void RecorderController::start() {
  started_at_ = std::chrono::steady_clock::now();
  state_machine_->start();
  publish_status();

  try {
    {
      std::lock_guard<std::mutex> lock(recorder_api_mutex_);
      recorder_->record();
    }
  } catch (const std::exception &error) {
    recorder_error_ = error.what();
    state_machine_->fail();
    RCLCPP_ERROR(get_logger(), "rosbag2 recorder failed to start: %s",
                 error.what());
    throw;
  }

  // Humble's Recorder::record() initializes subscriptions and returns. Keep
  // its node spinning independently so controller callbacks remain responsive
  // while rosbag receives and writes high-bandwidth camera messages.
  recorder_executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  recorder_executor_->add_node(recorder_);
  recorder_executor_thread_ = std::thread([this]() {
    try {
      recorder_executor_->spin();
    } catch (const std::exception &error) {
      recorder_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "rosbag2 recorder executor failed: %s",
                   error.what());
    }
    recorder_executor_finished_ = true;
  });

  tick_timer_ =
      create_wall_timer(25ms, std::bind(&RecorderController::on_tick, this),
                        controller_callback_group_);
  bootstrap_timer_ = create_wall_timer(
      100ms, std::bind(&RecorderController::on_bootstrap_tick, this),
      controller_callback_group_);
  status_timer_ = create_wall_timer(
      1s, std::bind(&RecorderController::publish_status, this),
      controller_callback_group_);
}

void RecorderController::stop(const std::string &reason) {
  if (stopping_.exchange(true)) {
    return;
  }
  if (tick_timer_) {
    tick_timer_->cancel();
  }
  if (bootstrap_timer_) {
    bootstrap_timer_->cancel();
  }
  if (status_timer_) {
    status_timer_->cancel();
  }

  if (!session_start_emitted_) {
    publish_event(RecordingEvent::SESSION_START, "startup", 0, 0);
    session_start_emitted_ = true;
  }
  execute(state_machine_->shutdown());
  publish_event(RecordingEvent::SESSION_END, reason,
                state_machine_->capture_window_id(),
                state_machine_->action_segment_id());
  if (rclcpp::ok()) {
    rclcpp::sleep_for(50ms);
  }
  // Let the still-running recorder executor consume the terminal events before
  // preventing further callbacks and finalizing the MCAP writer.
  if (recorder_executor_) {
    recorder_executor_->cancel();
  }
  if (recorder_executor_thread_.joinable()) {
    recorder_executor_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(recorder_api_mutex_);
    if (recorder_) {
      recorder_->stop();
    }
  }
  write_manifest(recorder_error_.empty() ? "complete" : "failed", reason);
  events_stream_.flush();
  events_stream_.close();
}

void RecorderController::on_gate(const std_msgs::msg::Bool::SharedPtr message) {
  execute(
      state_machine_->on_gate(message->data, std::chrono::steady_clock::now()));
}

void RecorderController::on_camera_frame(
    const sensor_msgs::msg::Image::ConstSharedPtr message) {
  camera_seen_ = true;
  DeadmanFrameState state;
  state.header = message->header;
  state.recorder_stamp = now();
  state.frame_sequence = ++frame_sequence_;
  state.operator_deadman = state_machine_->gate_held();
  state.gate_stale = state_machine_->gate_stale();
  frame_state_publisher_->publish(state);
}

void RecorderController::on_joint_state(
    const sensor_msgs::msg::JointState::ConstSharedPtr) {
  joint_state_seen_ = true;
}

void RecorderController::on_static_tf(
    const tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
  static_tf_seen_ = static_tf_seen_ || !message->transforms.empty();
}

void RecorderController::on_tick() {
  if (recorder_executor_finished_ && !stopping_) {
    state_machine_->fail();
    publish_event(RecordingEvent::RECORDER_FAILURE,
                  recorder_error_.empty() ? "recorder_executor_stopped"
                                          : recorder_error_,
                  state_machine_->capture_window_id(),
                  state_machine_->action_segment_id());
    publish_status();
    tick_timer_->cancel();
    return;
  }
  execute(state_machine_->tick(std::chrono::steady_clock::now()));
}

void RecorderController::on_bootstrap_tick() {
  if (!session_start_emitted_) {
    publish_event(RecordingEvent::SESSION_START, "startup", 0, 0);
    session_start_emitted_ = true;
  }
  const auto now_steady = std::chrono::steady_clock::now();
  const bool complete = camera_seen_ && joint_state_seen_ && static_tf_seen_;
  const bool timed_out =
      now_steady - started_at_ >= seconds_to_duration(bootstrap_timeout_sec_);
  if (!complete && !timed_out) {
    return;
  }

  const std::string reason =
      complete ? "required_samples_received" : "bootstrap_timeout";
  if (timed_out && !complete) {
    publish_event(RecordingEvent::RECORDER_WARNING, reason,
                  state_machine_->capture_window_id(),
                  state_machine_->action_segment_id());
  }
  publish_event(RecordingEvent::BOOTSTRAP_COMPLETE, reason,
                state_machine_->capture_window_id(),
                state_machine_->action_segment_id());
  execute(state_machine_->bootstrap_complete(now_steady));
  bootstrap_timer_->cancel();
  publish_status();
}

void RecorderController::publish_status() {
  std_msgs::msg::String status;
  status.data = to_string(state_machine_->state());
  status_publisher_->publish(status);
}

void RecorderController::execute(const std::vector<RecorderCommand> &commands) {
  for (const auto &command_value : commands) {
    switch (command_value.type) {
    case CommandType::Resume: {
      std::lock_guard<std::mutex> lock(recorder_api_mutex_);
      recorder_->resume();
    } break;
    case CommandType::Pause: {
      // Boundary events are published immediately before pause. This short,
      // recorder-local delay lets DDS deliver them to rosbag2 without ever
      // blocking the teleoperation control path.
      rclcpp::sleep_for(20ms);
      std::lock_guard<std::mutex> lock(recorder_api_mutex_);
      recorder_->pause();
    } break;
    case CommandType::OpenWindow:
      publish_event(RecordingEvent::WINDOW_START, command_value.reason,
                    command_value.capture_window_id,
                    command_value.action_segment_id);
      break;
    case CommandType::CloseWindow:
      publish_event(RecordingEvent::WINDOW_END, command_value.reason,
                    command_value.capture_window_id,
                    command_value.action_segment_id);
      break;
    case CommandType::OpenSegment:
      publish_event(RecordingEvent::SEGMENT_START, command_value.reason,
                    command_value.capture_window_id,
                    command_value.action_segment_id);
      break;
    case CommandType::CloseSegment:
      publish_event(RecordingEvent::SEGMENT_END, command_value.reason,
                    command_value.capture_window_id,
                    command_value.action_segment_id);
      break;
    case CommandType::Stop:
      break;
    }
  }
  if (!commands.empty()) {
    publish_status();
  }
}

void RecorderController::publish_event(uint8_t event_type,
                                       const std::string &reason,
                                       uint64_t window_id,
                                       uint64_t segment_id) {
  RecordingEvent event;
  event.header.stamp = now();
  event.header.frame_id = "recorder_clock";
  event.schema_version = 1;
  event.session_id = session_id_;
  event.capture_window_id = window_id;
  event.action_segment_id = segment_id;
  event.event_type = event_type;
  event.reason = reason;
  event.operator_deadman = state_machine_->gate_held();
  event.gate_stale = state_machine_->gate_stale();
  event_publisher_->publish(event);

  events_stream_ << "{\"stamp_ns\":"
                 << rclcpp::Time(event.header.stamp).nanoseconds()
                 << ",\"event_type\":" << static_cast<unsigned int>(event_type)
                 << ",\"session_id\":\"" << json_escape(session_id_)
                 << "\",\"capture_window_id\":" << window_id
                 << ",\"action_segment_id\":" << segment_id
                 << ",\"operator_deadman\":"
                 << (event.operator_deadman ? "true" : "false")
                 << ",\"gate_stale\":" << (event.gate_stale ? "true" : "false")
                 << ",\"reason\":\"" << json_escape(reason) << "\"}\n";
  events_stream_.flush();
}

void RecorderController::write_manifest(const std::string &status,
                                        const std::string &reason) {
  const auto temporary_path = session_path_ / "manifest.json.tmp";
  const auto final_path = session_path_ / "manifest.json";
  std::ofstream manifest(temporary_path, std::ios::out | std::ios::trunc);
  if (!manifest) {
    throw std::runtime_error("cannot write recorder manifest");
  }
  manifest << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"session_id\": \"" << json_escape(session_id_) << "\",\n"
           << "  \"status\": \"" << json_escape(status) << "\",\n"
           << "  \"reason\": \"" << json_escape(reason) << "\",\n"
           << "  \"recording_mode\": \"" << json_escape(recording_mode_name_)
           << "\",\n"
           << "  \"storage_id\": \"" << json_escape(storage_id_) << "\",\n"
           << "  \"bag_path\": \"bag\",\n"
           << "  \"operator_input_policy\": \"deadman_only\",\n"
           << "  \"record_topics\": [\n";
  for (std::size_t index = 0; index < record_topics_.size(); ++index) {
    manifest << "    \"" << json_escape(record_topics_[index]) << "\"";
    manifest << (index + 1 == record_topics_.size() ? "\n" : ",\n");
  }
  manifest << "  ]\n}\n";
  manifest.close();
  std::error_code error;
  std::filesystem::rename(temporary_path, final_path, error);
  if (error) {
    std::filesystem::remove(final_path, error);
    error.clear();
    std::filesystem::rename(temporary_path, final_path, error);
  }
  if (error) {
    throw std::runtime_error("cannot atomically finalize recorder manifest: " +
                             error.message());
  }
}

std::string RecorderController::json_escape(const std::string &value) {
  std::ostringstream escaped;
  for (const char character : value) {
    switch (character) {
    case '\\':
      escaped << "\\\\";
      break;
    case '"':
      escaped << "\\\"";
      break;
    case '\n':
      escaped << "\\n";
      break;
    case '\r':
      escaped << "\\r";
      break;
    case '\t':
      escaped << "\\t";
      break;
    default:
      escaped << character;
      break;
    }
  }
  return escaped.str();
}

} // namespace vive_dataset_recorder
