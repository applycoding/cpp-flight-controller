#pragma once

#include <atomic>
#include <cstddef>
#include <type_traits>

namespace fc {

// Single-producer, single-consumer ring. Capacity must be a power of two and
// at least 2. One slot is left empty so head == tail means empty, so the
// usable depth is Capacity - 1.
//
// The producer may only call push(). The consumer may only call pop() / popLatest().
// Indices are published with release stores and observed with acquire loads, so a
// successful pop sees the matching push's write.
template <typename T, std::size_t Capacity>
class SpscRingBuffer {
  static_assert(Capacity >= 2, "ring capacity must be at least 2");
  static_assert((Capacity & (Capacity - 1)) == 0, "ring capacity must be a power of two");
  static_assert(std::is_trivially_copyable_v<T>, "ring elements must be trivially copyable");

 public:
  bool push(const T& item) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & kMask;
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    slots_[head] = item;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T& item) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    item = slots_[tail];
    tail_.store((tail + 1) & kMask, std::memory_order_release);
    return true;
  }

  // Consumer side: drop queued items and keep the newest. False if the ring was empty.
  bool popLatest(T& item) {
    bool any = false;
    T next{};
    while (pop(next)) {
      item = next;
      any = true;
    }
    return any;
  }

  std::size_t capacity() const { return Capacity - 1; }

 private:
  static constexpr std::size_t kMask = Capacity - 1;

  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  T slots_[Capacity]{};
};

}  // namespace fc
