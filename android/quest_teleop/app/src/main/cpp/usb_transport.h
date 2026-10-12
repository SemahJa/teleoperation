#pragma once
#include "logger.h"


#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace quest_usb {
#ifndef QUEST_BRIDGE_PORT
#define QUEST_BRIDGE_PORT 45454
#endif
constexpr uint16_t kBridgePort = QUEST_BRIDGE_PORT;
constexpr size_t kMaximumPacketBytes = 4096;
constexpr uint64_t kMaxCommandAgeNs = 100'000'000ULL;


uint64_t monotonic_time_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) return 0;
  return static_cast<uint64_t>(value.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(value.tv_nsec);
}

std::string make_session_id() {
  std::random_device random;
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) output << std::setw(8) << random();
  return output.str();
}

bool send_all(int fd, const void* data, size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  size_t sent = 0;
  while (sent < size) {
    const ssize_t count = send(fd, bytes + sent, size - sent, MSG_NOSIGNAL);
    if (count < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (count == 0) return false;
    sent += static_cast<size_t>(count);
  }
  return true;
}

struct ControllerSample {
  std::string controller;
  uint64_t sequence{0};
  uint64_t acquisition_ns{0};
  std::string payload;
};

struct ClockSnapshot {
  bool valid{false};
  int64_t pc_minus_quest_ns{0};
  uint64_t uncertainty_ns{std::numeric_limits<uint64_t>::max()};
  uint64_t age_ns{std::numeric_limits<uint64_t>::max()};
};

class UsbTransport {
 public:
  UsbTransport() : worker_([this] { run(); }) {}
  ~UsbTransport() { stop(); }
  void stop() {
    if (stopping_.exchange(true)) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.clear();
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
  UsbTransport(const UsbTransport&) = delete;
  UsbTransport& operator=(const UsbTransport&) = delete;

  void submit(std::string controller, uint64_t sequence, uint64_t acquisition_ns,
              std::string payload) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.size() >= kMaximumPendingSamples) {
      pending_.clear();
      overflow_.store(true);
      disarm_request_.store(true);
      condition_.notify_one();
      return;
    }
    pending_.push_back({std::move(controller), sequence, acquisition_ns, std::move(payload)});
    condition_.notify_one();
  }
  bool connected() const { return connected_.load(); }
  bool take_disarm_request() { return disarm_request_.exchange(false); }
  ClockSnapshot clock_snapshot(uint64_t now_ns) const {
    ClockSnapshot snapshot;
    const uint64_t last = last_sync_ns_.load();
    if (last == 0 || now_ns < last) return snapshot;
    snapshot.age_ns = now_ns - last;
    const uint64_t base = uncertainty_ns_.load();
    const uint64_t drift = snapshot.age_ns / 20'000;  // 50 ppm.
    const uint64_t maximum = std::numeric_limits<uint64_t>::max();
    snapshot.uncertainty_ns = base > maximum - drift ? maximum : base + drift;
    snapshot.pc_minus_quest_ns = offset_ns_.load();
    snapshot.valid = clock_valid_.load() && snapshot.age_ns <= kMaxClockAgeNs &&
                     snapshot.uncertainty_ns <= kMaxClockUncertaintyNs;
    return snapshot;
  }

 private:
  static constexpr size_t kMaximumPendingSamples = 256;
  static constexpr uint64_t kMaxClockAgeNs = 3'000'000'000ULL;
  static constexpr uint64_t kMaxClockUncertaintyNs = 10'000'000ULL;
  static constexpr uint64_t kClockSyncPeriodNs = 1'000'000'000ULL;
  int connect_loopback() {
    const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100'000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(kBridgePort);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      close(fd);
      return -1;
    }
    const int no_delay = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)) != 0) {
      close(fd);
      return -1;
    }
    return fd;
  }

  bool send_frame(int fd, const std::string& payload) {
    if (payload.empty() || payload.size() > kMaximumPacketBytes) return false;
    const uint32_t header = htonl(static_cast<uint32_t>(payload.size()));
    std::string frame(sizeof(header), '\0');
    std::memcpy(frame.data(), &header, sizeof(header));
    frame += payload;
    return send_all(fd, frame.data(), frame.size());
  }

  static bool receive_all(int fd, void* destination, size_t size) {
    auto* bytes = static_cast<uint8_t*>(destination);
    size_t received = 0;
    while (received < size) {
      const ssize_t count = recv(fd, bytes + received, size - received, 0);
      if (count < 0) {
        if (errno == EINTR) continue;
        return false;
      }
      if (count == 0) return false;
      received += static_cast<size_t>(count);
    }
    return true;
  }

  static bool receive_frame(int fd, std::string& payload) {
    uint32_t wire_length = 0;
    if (!receive_all(fd, &wire_length, sizeof(wire_length))) return false;
    const uint32_t length = ntohl(wire_length);
    if (length == 0 || length > kMaximumPacketBytes) return false;
    payload.assign(length, '\0');
    return receive_all(fd, payload.data(), length);
  }

  static std::string json_string(const std::string& json, const char* key) {
    const std::string marker = std::string("\"") + key + "\":\"";
    const size_t start = json.find(marker);
    if (start == std::string::npos) throw std::runtime_error(std::string("missing ") + key);
    const size_t value_start = start + marker.size();
    const size_t end = json.find('"', value_start);
    if (end == std::string::npos) throw std::runtime_error(std::string("bad ") + key);
    return json.substr(value_start, end - value_start);
  }

  bool synchronize_clock(int fd) {
    const uint64_t q1 = monotonic_time_ns();
    const std::string sync_id = std::to_string(++sync_sequence_);
    const std::string request = "{\"version\":1,\"type\":\"clock_sync_request\",\"sync_id\":\"" +
        sync_id + "\",\"quest_send_time_ns\":\"" + std::to_string(q1) + "\"}";
    std::string reply;
    if (!send_frame(fd, request) || !receive_frame(fd, reply)) {
      clock_valid_.store(false);
      disarm_request_.store(true);
      return false;
    }
    const uint64_t q4 = monotonic_time_ns();
    try {
      if (json_string(reply, "type") != "clock_sync_reply" || json_string(reply, "sync_id") != sync_id ||
          std::stoull(json_string(reply, "quest_send_time_ns")) != q1) throw std::runtime_error("clock reply mismatch");
      const uint64_t p2 = std::stoull(json_string(reply, "pc_receive_time_ns"));
      const uint64_t p3 = std::stoull(json_string(reply, "pc_send_time_ns"));
      if (q4 < q1 || p3 < p2) throw std::runtime_error("clock timestamps reversed");
      const long double delay = static_cast<long double>(q4 - q1) - static_cast<long double>(p3 - p2);
      if (delay < 0.0L) throw std::runtime_error("negative clock round-trip delay");
      const long double offset = (static_cast<long double>(p2) - static_cast<long double>(q1) +
                                  static_cast<long double>(p3) - static_cast<long double>(q4)) / 2.0L;
      const long double uncertainty = delay / 2.0L + 100'000.0L;
      if (offset < static_cast<long double>(std::numeric_limits<int64_t>::min()) ||
          offset > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
        throw std::runtime_error("clock offset out of range");
      }
      offset_ns_.store(static_cast<int64_t>(std::llround(offset)));
      uncertainty_ns_.store(static_cast<uint64_t>(std::ceil(std::min(
          uncertainty, static_cast<long double>(std::numeric_limits<uint64_t>::max())))));
      last_sync_ns_.store(q4);
      const bool valid = uncertainty <= static_cast<long double>(kMaxClockUncertaintyNs);
      clock_valid_.store(valid);
      if (!valid) disarm_request_.store(true);
      LOGI("Clock sync %s: offset=%lld ns uncertainty=%llu ns RTT=%.3f ms",
           valid ? "valid" : "outside threshold",
           static_cast<long long>(offset_ns_.load()),
           static_cast<unsigned long long>(uncertainty_ns_.load()),
           static_cast<double>(q4 - q1) / 1.0e6);
      return true;
    } catch (const std::exception& error) {
      LOGE("Clock synchronization failed: %s", error.what());
      clock_valid_.store(false);
      disarm_request_.store(true);
      return false;
    }
  }

  void run() {
    int socket_fd = -1;
    bool has_connected_once = false;
    uint64_t last_sync_ns = 0;
    while (!stopping_.load()) {
      ControllerSample sample;
      bool have_sample = false;
      bool queue_overflow = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(25),
                            [this] { return stopping_.load() || !pending_.empty(); });
        if (stopping_.load()) break;
        queue_overflow = overflow_.exchange(false);
        if (!queue_overflow && !pending_.empty()) {
          const auto earliest = std::min_element(pending_.begin(), pending_.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.sequence < rhs.sequence; });
          sample = std::move(*earliest);
          pending_.erase(earliest);
          have_sample = true;
        }
      }
      if (queue_overflow) {
        LOGE("Controller telemetry queue overflow; dropping connection to avoid silent sample coalescing");
        if (socket_fd >= 0) {
          shutdown(socket_fd, SHUT_RDWR);
          close(socket_fd);
          socket_fd = -1;
        }
        connected_.store(false);
        clock_valid_.store(false);
        disarm_request_.store(true);
        last_sync_ns = 0;
        continue;
      }
      if (socket_fd < 0) {
        if (!have_sample) continue;
        socket_fd = connect_loopback();
        connected_.store(socket_fd >= 0);
        if (socket_fd < 0) continue;
        if (has_connected_once) disarm_request_.store(true);
        has_connected_once = true;
        last_sync_ns = 0;
      }
      const uint64_t now_ns = monotonic_time_ns();
      if (last_sync_ns == 0 || now_ns < last_sync_ns || now_ns - last_sync_ns >= kClockSyncPeriodNs) {
        if (!synchronize_clock(socket_fd)) {
          shutdown(socket_fd, SHUT_RDWR);
          close(socket_fd);
          socket_fd = -1;
          connected_.store(false);
          clock_valid_.store(false);
          continue;
        }
        last_sync_ns = monotonic_time_ns();
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.clear();  // Drop samples produced while synchronization was in flight.
        continue;
      }
      if (!have_sample) continue;
      const uint64_t send_time_ns = monotonic_time_ns();
      if (send_time_ns < sample.acquisition_ns ||
          send_time_ns - sample.acquisition_ns > kMaxCommandAgeNs) {
        disarm_request_.store(true);
        shutdown(socket_fd, SHUT_RDWR);
        close(socket_fd);
        socket_fd = -1;
        connected_.store(false);
        clock_valid_.store(false);
        continue;
      }
      if (!send_frame(socket_fd, sample.payload)) {
        shutdown(socket_fd, SHUT_RDWR);
        close(socket_fd);
        socket_fd = -1;
        connected_.store(false);
        clock_valid_.store(false);
        disarm_request_.store(true);
        last_sync_ns = 0;
      }
    }
    if (socket_fd >= 0) {
      shutdown(socket_fd, SHUT_RDWR);
      close(socket_fd);
    }
    connected_.store(false);
    clock_valid_.store(false);
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<ControllerSample> pending_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> clock_valid_{false};
  std::atomic<int64_t> offset_ns_{0};
  std::atomic<uint64_t> uncertainty_ns_{std::numeric_limits<uint64_t>::max()};
  std::atomic<uint64_t> last_sync_ns_{0};
  std::atomic<bool> disarm_request_{false};
  std::atomic<bool> overflow_{false};
  std::atomic<bool> stopping_{false};
  uint64_t sync_sequence_{0};
  std::thread worker_;
};

}  // namespace quest_usb
