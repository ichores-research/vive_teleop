#include "vive_dataset_recorder/recorder_state_machine.hpp"

#include <stdexcept>
#include <utility>

namespace vive_dataset_recorder {

RecorderStateMachine::RecorderStateMachine(
    RecordingMode mode, std::chrono::nanoseconds post_roll,
    std::chrono::nanoseconds gate_stale_timeout)
    : mode_(mode), post_roll_(post_roll),
      gate_stale_timeout_(gate_stale_timeout) {
  if (post_roll < std::chrono::nanoseconds::zero() ||
      gate_stale_timeout <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("recorder durations must be positive");
  }
}

void RecorderStateMachine::start() {
  if (state_ != RecorderState::Starting) {
    throw std::logic_error("recorder can only start once");
  }
  state_ = RecorderState::Bootstrap;
}

std::vector<RecorderCommand>
RecorderStateMachine::bootstrap_complete(TimePoint now) {
  if (state_ != RecorderState::Bootstrap) {
    return {};
  }

  if (mode_ == RecordingMode::ContinuousSession) {
    state_ = RecorderState::Recording;
    window_open_ = true;
    ++capture_window_id_;
    std::vector<RecorderCommand> result{
        command(CommandType::OpenWindow, "session")};
    if (gate_held_) {
      result.push_back(open_segment());
    }
    return result;
  }

  if (gate_held_) {
    state_ = RecorderState::Recording;
    window_open_ = true;
    ++capture_window_id_;
    return {command(CommandType::OpenWindow), open_segment()};
  }

  state_ = RecorderState::Paused;
  post_roll_deadline_ = now;
  return {command(CommandType::Pause)};
}

std::vector<RecorderCommand> RecorderStateMachine::on_gate(bool held,
                                                           TimePoint now) {
  const bool rising = !gate_held_ && held;
  const bool falling = gate_held_ && !held;
  have_gate_sample_ = true;
  gate_held_ = held;
  gate_stale_ = false;
  last_gate_update_ = now;

  if (state_ == RecorderState::Starting || state_ == RecorderState::Bootstrap ||
      state_ == RecorderState::Stopping || state_ == RecorderState::Failed) {
    return {};
  }

  if (mode_ == RecordingMode::ContinuousSession) {
    if (rising && !segment_open_) {
      return {open_segment()};
    }
    if (falling && segment_open_) {
      return {close_segment("explicit_release")};
    }
    return {};
  }

  if (held && state_ == RecorderState::Paused) {
    state_ = RecorderState::Recording;
    window_open_ = true;
    ++capture_window_id_;
    return {command(CommandType::Resume), command(CommandType::OpenWindow),
            open_segment()};
  }
  if (held && state_ == RecorderState::PostRoll) {
    state_ = RecorderState::Recording;
    return {open_segment()};
  }
  if (!held && state_ == RecorderState::Recording && segment_open_) {
    return handle_gate_release("explicit_release", now);
  }
  return {};
}

std::vector<RecorderCommand> RecorderStateMachine::tick(TimePoint now) {
  if (gate_held_ && have_gate_sample_ &&
      now - last_gate_update_ >= gate_stale_timeout_) {
    gate_held_ = false;
    gate_stale_ = true;
    if (segment_open_) {
      if (mode_ == RecordingMode::ContinuousSession) {
        return {close_segment("gate_stale")};
      }
      return handle_gate_release("gate_stale", now);
    }
  }

  if (state_ == RecorderState::PostRoll && now >= post_roll_deadline_) {
    state_ = RecorderState::Paused;
    window_open_ = false;
    return {command(CommandType::CloseWindow, "post_roll_complete"),
            command(CommandType::Pause)};
  }
  return {};
}

std::vector<RecorderCommand> RecorderStateMachine::shutdown() {
  if (state_ == RecorderState::Stopping || state_ == RecorderState::Failed) {
    return {};
  }
  const bool was_paused = state_ == RecorderState::Paused;
  state_ = RecorderState::Stopping;
  std::vector<RecorderCommand> result;
  if (was_paused) {
    result.push_back(command(CommandType::Resume, "shutdown_events"));
  }
  if (segment_open_) {
    result.push_back(close_segment("shutdown"));
  }
  if (window_open_) {
    window_open_ = false;
    result.push_back(command(CommandType::CloseWindow, "shutdown"));
  }
  result.push_back(command(CommandType::Stop, "shutdown"));
  return result;
}

void RecorderStateMachine::fail() { state_ = RecorderState::Failed; }

RecorderCommand RecorderStateMachine::command(CommandType type,
                                              const std::string &reason) const {
  return {type, reason, capture_window_id_, action_segment_id_};
}

RecorderCommand RecorderStateMachine::open_segment() {
  segment_open_ = true;
  ++action_segment_id_;
  return command(CommandType::OpenSegment);
}

RecorderCommand RecorderStateMachine::close_segment(const std::string &reason) {
  segment_open_ = false;
  return command(CommandType::CloseSegment, reason);
}

std::vector<RecorderCommand>
RecorderStateMachine::handle_gate_release(const std::string &reason,
                                          TimePoint now) {
  state_ = RecorderState::PostRoll;
  post_roll_deadline_ = now + post_roll_;
  return {close_segment(reason)};
}

RecordingMode parse_recording_mode(const std::string &value) {
  if (value == "deadman_window") {
    return RecordingMode::DeadmanWindow;
  }
  if (value == "continuous_session") {
    return RecordingMode::ContinuousSession;
  }
  throw std::invalid_argument(
      "recording_mode must be deadman_window or continuous_session");
}

const char *to_string(RecorderState state) noexcept {
  switch (state) {
  case RecorderState::Starting:
    return "starting";
  case RecorderState::Bootstrap:
    return "bootstrap";
  case RecorderState::Paused:
    return "ready-paused";
  case RecorderState::Recording:
    return "recording";
  case RecorderState::PostRoll:
    return "post-roll";
  case RecorderState::Stopping:
    return "stopping";
  case RecorderState::Failed:
    return "failed";
  }
  return "unknown";
}

} // namespace vive_dataset_recorder
