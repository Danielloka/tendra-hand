// Lock-free single-producer / single-consumer ring of fixed-size items. The sampler (core 1) is
// the only producer and the radio task (core 0) the only consumer, so a push never blocks and
// never takes a lock: when the ring is full the NEWEST frame is dropped and counted (the radio
// is behind; old frames still go out in order and the host sees the gap from `seq`).
#pragma once

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace tg {

template <typename T, size_t N>
class SpscRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");

 public:
  bool push(const T& item) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= N) {
      drops_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    memcpy(&buf_[head & (N - 1)], &item, sizeof(T));
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  bool pop(T& out) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    if (head == tail) return false;
    memcpy(&out, &buf_[tail & (N - 1)], sizeof(T));
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  size_t size() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  uint32_t drops() const { return drops_.load(std::memory_order_relaxed); }

 private:
  T buf_[N];
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
  std::atomic<uint32_t> drops_{0};
};

}  // namespace tg
