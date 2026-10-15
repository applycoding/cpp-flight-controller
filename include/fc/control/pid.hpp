#pragma once

#include "fc/types.hpp"

namespace fc {

// PID with output clamping, back-calculation anti-windup, and a first-order
// filter on the derivative of the measurement (so a setpoint step does not kick).
class Pid {
 public:
  Pid() = default;
  explicit Pid(PidGains gains);

  void setGains(PidGains gains);
  void reset();

  // `error` is setpoint - measurement. The D term uses d(measurement)/dt.
  double update(double error, double measurement, double dt);

  double integrator() const { return integrator_; }
  double lastDerivative() const { return derivative_; }
  double lastUnsaturated() const { return last_unsaturated_; }

 private:
  PidGains gains_{};
  double integrator_{0.0};
  double derivative_{0.0};
  double previous_measurement_{0.0};
  double last_unsaturated_{0.0};
  bool derivative_ready_{false};
};

}  // namespace fc
