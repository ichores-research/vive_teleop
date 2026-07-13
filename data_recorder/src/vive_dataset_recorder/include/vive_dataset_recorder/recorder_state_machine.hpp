#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace vive_dataset_recorder {

enum class RecordingMode { DeadmanWindow, ContinuousSession };
enum class RecorderState {
  Starting,
  Bootstrap,
  Paused,
  Recording,
  PostRoll,
  Stopping,
  Failed
};
enum class CommandType {
  Resume,
  Pause,
  OpenWindow,
  CloseWindow,
  OpenSegment,
  CloseSegment,
  Stop
};

struct RecorderCommand {
  CommandType type;
  std::string reason;
  uint64_t capture_window_id{0};
  uint64_t action_segment_id{0};
};

class RecorderStateMachine {
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  RecorderStateMachine(RecordingMode mode, std::chrono::nanoseconds post_roll,
                       std::chrono::nanoseconds gate_stale_timeout);

  void start();
  std::vector<RecorderCommand> bootstrap_complete(TimePoint now);
  std::vector<RecorderCommand> on_gate(bool held, TimePoint now);
  std::vector<RecorderCommand> tick(TimePoint now);
  std::vector<RecorderCommand> shutdown();
  void fail();

  RecorderState state() const noexcept { return state_; }
  bool gate_held() const noexcept { return gate_held_; }
  bool gate_stale() const noexcept { return gate_stale_; }
  uint64_t capture_window_id() const noexcept { return capture_window_id_; }
  uint64_t action_segment_id() const noexcept { return action_segment_id_; }

private:
  RecorderCommand command(CommandType type,
                          const std::string &reason = "") const;
  RecorderCommand open_window();
  RecorderCommand open_segment();
  RecorderCommand close_segment(const std::string &reason);
  std::vector<RecorderCommand> handle_gate_release(const std::string &reason,
                                                   TimePoint now);

  RecordingMode mode_;
  RecorderState state_{RecorderState::Starting};
  std::chrono::nanoseconds post_roll_;
  std::chrono::nanoseconds gate_stale_timeout_;
  TimePoint last_gate_update_{};
  TimePoint post_roll_deadline_{};
  bool have_gate_sample_{false};
  bool gate_held_{false};
  bool gate_stale_{false};
  bool segment_open_{false};
  bool window_open_{false};
  uint64_t capture_window_id_{0};
  uint64_t action_segment_id_{0};
};

RecordingMode parse_recording_mode(const std::string &value);
const char *to_string(RecorderState state) noexcept;

} // namespace vive_dataset_recorder
