#pragma once

#include <chrono>

namespace fc {

// Wall-clock pacer for a periodic thread. If a deadline was already missed,
// the schedule is rebased instead of bursting to catch up.
class FixedRateLoop {
 public:
  explicit FixedRateLoop(double frequency_hz);

  // Sleep until the next tick. Returns the measured period, in seconds.
  double wait();

  double periodSeconds() const { return period_s_; }

 private:
  using Clock = std::chrono::steady_clock;

  double period_s_{0.0};
  Clock::duration period_{};
  Clock::time_point next_{};
  bool started_{false};
};

}  // namespace fc
