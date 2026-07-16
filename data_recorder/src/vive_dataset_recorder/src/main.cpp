#include "vive_dataset_recorder/recorder_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <csignal>
#include <exception>
#include <memory>

namespace {
volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int) { shutdown_requested = 1; }
} // namespace

int main(int argc, char **argv) {
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_shutdown);
  std::signal(SIGTERM, request_shutdown);
  try {
    auto controller =
        std::make_shared<vive_dataset_recorder::RecorderController>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 4);
    executor->add_node(controller);
    executor->add_node(controller->recorder_node());
    auto signal_monitor =
        std::make_shared<rclcpp::Node>("vive_dataset_recorder_signal_monitor");
    auto signal_timer = signal_monitor->create_wall_timer(50ms, [executor]() {
      if (shutdown_requested != 0) {
        executor->cancel();
      }
    });
    (void)signal_timer;
    executor->add_node(signal_monitor);
    controller->start();
    executor->spin();
    controller->stop("shutdown");
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("vive_dataset_recorder"), "%s",
                 error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
