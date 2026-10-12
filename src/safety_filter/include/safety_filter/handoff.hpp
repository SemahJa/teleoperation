#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace safety_filter {

// One producer and one consumer only. Neither side waits or overwrites a
// buffer owned by the other side. Overflow is reported, never silently hidden.
template<class T, std::size_t Capacity = 64>
class SpscQueue {
 public:
  static_assert(Capacity > 1, "queue needs two slots");
  static_assert(std::atomic<std::size_t>::is_always_lock_free,
                "realtime queue needs lock-free indices");
  bool push(const T& value) noexcept {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % Capacity;
    if (next == read_.load(std::memory_order_acquire)) return false;
    storage_[write] = value;
    write_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(T& value) noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return false;
    value = storage_[read];
    read_.store((read + 1) % Capacity, std::memory_order_release);
    return true;
  }
 private:
  std::array<T, Capacity> storage_{};
  alignas(64) std::atomic<std::size_t> write_{0};
  alignas(64) std::atomic<std::size_t> read_{0};
};

// Latest-only telemetry. The producer, middle slot, and consumer always own
// different slots; dropping old telemetry cannot block the torque callback.
template<class T>
class LatestSnapshot {
 public:
  static_assert(std::atomic<unsigned>::is_always_lock_free,
                "realtime snapshot needs lock-free indices");
  void publish(const T& value) noexcept {
    storage_[back_] = value;
    back_ = middle_.exchange(back_ | dirty, std::memory_order_acq_rel) & mask;
  }
  bool read(T& value) noexcept {
    if (!(middle_.load(std::memory_order_acquire) & dirty)) return false;
    front_ = middle_.exchange(front_, std::memory_order_acq_rel) & mask;
    value = storage_[front_];
    return true;
  }
 private:
  static constexpr unsigned dirty = 4, mask = 3;
  std::array<T, 3> storage_{};
  unsigned front_{0}, back_{2};
  std::atomic<unsigned> middle_{1};
};
}  // namespace safety_filter
