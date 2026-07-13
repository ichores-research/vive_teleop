#include "vive_dataset_recorder/recorder_state_machine.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace vive_dataset_recorder {
namespace {
using namespace std::chrono_literals;

TEST(RecorderStateMachine, RecordsDeadmanWindowWithPostRoll) {
  RecorderStateMachine machine(RecordingMode::DeadmanWindow, 750ms, 250ms);
  const auto start = RecorderStateMachine::TimePoint{};
  machine.start();
  EXPECT_EQ(machine.bootstrap_complete(start).front().type, CommandType::Pause);

  const auto opened = machine.on_gate(true, start + 1ms);
  ASSERT_EQ(opened.size(), 3u);
  EXPECT_EQ(opened[0].type, CommandType::Resume);
  EXPECT_EQ(opened[1].type, CommandType::OpenWindow);
  EXPECT_EQ(opened[2].type, CommandType::OpenSegment);

  const auto released = machine.on_gate(false, start + 100ms);
  ASSERT_EQ(released.size(), 1u);
  EXPECT_EQ(released[0].reason, "explicit_release");
  EXPECT_TRUE(machine.tick(start + 849ms).empty());
  const auto paused = machine.tick(start + 850ms);
  ASSERT_EQ(paused.size(), 2u);
  EXPECT_EQ(paused[0].type, CommandType::CloseWindow);
  EXPECT_EQ(paused[1].type, CommandType::Pause);
}

TEST(RecorderStateMachine, RepressDuringPostRollKeepsWindowOpen) {
  RecorderStateMachine machine(RecordingMode::DeadmanWindow, 750ms, 250ms);
  const auto start = RecorderStateMachine::TimePoint{};
  machine.start();
  machine.bootstrap_complete(start);
  machine.on_gate(true, start + 1ms);
  const auto first_window = machine.capture_window_id();
  machine.on_gate(false, start + 2ms);

  const auto reopened = machine.on_gate(true, start + 3ms);
  ASSERT_EQ(reopened.size(), 1u);
  EXPECT_EQ(reopened[0].type, CommandType::OpenSegment);
  EXPECT_EQ(machine.capture_window_id(), first_window);
  EXPECT_EQ(machine.state(), RecorderState::Recording);
}

TEST(RecorderStateMachine, StaleTrueGateClosesSegment) {
  RecorderStateMachine machine(RecordingMode::DeadmanWindow, 750ms, 250ms);
  const auto start = RecorderStateMachine::TimePoint{};
  machine.start();
  machine.bootstrap_complete(start);
  machine.on_gate(true, start);

  EXPECT_TRUE(machine.tick(start + 249ms).empty());
  const auto stale = machine.tick(start + 250ms);
  ASSERT_EQ(stale.size(), 1u);
  EXPECT_EQ(stale[0].type, CommandType::CloseSegment);
  EXPECT_EQ(stale[0].reason, "gate_stale");
  EXPECT_TRUE(machine.gate_stale());
  EXPECT_FALSE(machine.gate_held());
}

TEST(RecorderStateMachine, ContinuousModeNeverPausesForGate) {
  RecorderStateMachine machine(RecordingMode::ContinuousSession, 750ms, 250ms);
  const auto start = RecorderStateMachine::TimePoint{};
  machine.start();
  const auto bootstrap = machine.bootstrap_complete(start);
  ASSERT_EQ(bootstrap.size(), 1u);
  EXPECT_EQ(bootstrap[0].type, CommandType::OpenWindow);
  EXPECT_EQ(machine.on_gate(true, start + 1ms).front().type,
            CommandType::OpenSegment);
  EXPECT_EQ(machine.on_gate(false, start + 2ms).front().type,
            CommandType::CloseSegment);
  EXPECT_EQ(machine.state(), RecorderState::Recording);
}

TEST(RecorderStateMachine, ShutdownResumesPausedWriterForTerminalEvents) {
  RecorderStateMachine machine(RecordingMode::DeadmanWindow, 750ms, 250ms);
  const auto start = RecorderStateMachine::TimePoint{};
  machine.start();
  machine.bootstrap_complete(start);

  const auto commands = machine.shutdown();
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_EQ(commands[0].type, CommandType::Resume);
  EXPECT_EQ(commands[1].type, CommandType::Stop);
}

} // namespace
} // namespace vive_dataset_recorder
