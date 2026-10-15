#include "fc/rt/fixed_rate_loop.hpp"

#include <thread>

namespace fc {

FixedRateLoop::FixedRateLoop(double frequency_hz) {
  const double hz = frequency_hz > 1.0 ? frequency_hz : 1.0;
  period_s_ = 1.0 / hz;
  period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period_s_));
  if (period_.count() <= 0) {
    period_ = std::chrono::microseconds(1);
  }
}

double FixedRateLoop::wait() {
  const auto now = Clock::now();
  if (!started_) {
    started_ = true;
    next_ = now + period_;
    return period_s_;
  }

  if (next_ > now) {
    std::this_thread::sleep_until(next_);
  }
  const auto woke = Clock::now();
  const double dt = std::chrono::duration<double>(woke - (next_ - period_)).count();
  next_ += period_;
  if (next_ < woke) {
    next_ = woke + period_;
  }
  return dt > 0.0 ? dt : period_s_;
}

}  // namespace fc
