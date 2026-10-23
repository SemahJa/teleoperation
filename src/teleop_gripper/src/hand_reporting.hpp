#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <quest_teleop_interfaces/msg/gripper_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace teleop_gripper {
// DDS and console I/O cannot hold up input handling or follow-up motion.
// Status is latest-only; diagnostic events have a bounded backlog.
class HandReporting {
 public:
  using Status = quest_teleop_interfaces::msg::GripperStatus;
  explicit HandReporting(rclcpp::Publisher<Status>::SharedPtr publisher)
      : publisher_(std::move(publisher)), thread_([this] { run(); }) {}
  ~HandReporting() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      finished_ = true;
    }
    ready_.notify_one();
    thread_.join();
  }
  void status(Status value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_ = std::move(value);
    }
    ready_.notify_one();
  }
  void log(std::function<void()> event) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (logs_.size() == 128)
        logs_.pop_front();
      logs_.push_back(std::move(event));
    }
    ready_.notify_one();
  }

 private:
  void run() {
    while (true) {
      std::deque<std::function<void()>> logs;
      std::optional<Status> status;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return finished_ || status_ || !logs_.empty(); });
        if (finished_ && !status_ && logs_.empty())
          return;
        logs.swap(logs_);
        status.swap(status_);
      }
      try {
        for (auto& event : logs)
          event();
        if (status && rclcpp::ok())
          publisher_->publish(*status);
      } catch (const std::exception& e) {
        if (rclcpp::ok())
          RCLCPP_WARN(rclcpp::get_logger("hand_reporting"), "Hand reporting failed: %s", e.what());
      }
    }
  }
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::function<void()>> logs_;
  std::optional<Status> status_;
  bool finished_{false};
  std::thread thread_;
};
}  // namespace teleop_gripper
