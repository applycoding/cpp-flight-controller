#include "fc/control/pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace fc {

Pid::Pid(PidGains gains) : gains_(gains) {}

void Pid::setGains(PidGains gains) {
  gains_ = gains;
}

void Pid::reset() {
  integrator_ = 0.0;
  derivative_ = 0.0;
  previous_measurement_ = 0.0;
  last_unsaturated_ = 0.0;
  derivative_ready_ = false;
}

double Pid::update(double error, double measurement, double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return std::clamp(gains_.kp * error + integrator_, gains_.out_min, gains_.out_max);
  }

  double raw_rate = 0.0;
  if (!derivative_ready_) {
    previous_measurement_ = measurement;
    derivative_ready_ = true;
  } else {
    raw_rate = (measurement - previous_measurement_) / dt;
    previous_measurement_ = measurement;
  }

  if (gains_.derivative_filter_hz > 0.0) {
    const double tau = 1.0 / (2.0 * std::numbers::pi * gains_.derivative_filter_hz);
    const double alpha = dt / (tau + dt);
    derivative_ += alpha * (raw_rate - derivative_);
  } else {
    derivative_ = raw_rate;
  }

  const double unsaturated = gains_.kp * error + integrator_ - gains_.kd * derivative_;
  last_unsaturated_ = unsaturated;
  const double output = std::clamp(unsaturated, gains_.out_min, gains_.out_max);

  integrator_ += (gains_.ki * error + gains_.k_aw * (output - unsaturated)) * dt;
  constexpr double kIntegratorLimit = 1.0e6;
  integrator_ = std::clamp(integrator_, -kIntegratorLimit, kIntegratorLimit);
  return output;
}

}  // namespace fc
