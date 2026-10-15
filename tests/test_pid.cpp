#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "fc/control/pid.hpp"

TEST(Pid, ProportionalSignAndLimit) {
  fc::PidGains gains;
  gains.kp = 4.0;
  gains.out_min = -1.0;
  gains.out_max = 1.0;
  fc::Pid pid(gains);
  EXPECT_NEAR(pid.update(0.1, 0.0, 0.01), 0.4, 1.0e-12);
  EXPECT_NEAR(pid.update(10.0, 0.0, 0.01), 1.0, 1.0e-12);
  EXPECT_NEAR(pid.update(-10.0, 0.0, 0.01), -1.0, 1.0e-12);
}

TEST(Pid, BackCalculationBoundsTheIntegrator) {
  fc::PidGains wound;
  wound.ki = 5.0;
  wound.out_min = -1.0;
  wound.out_max = 1.0;
  fc::PidGains bounded = wound;
  bounded.k_aw = 8.0;

  fc::Pid open(wound);
  fc::Pid closed(bounded);
  const double dt = 0.01;
  for (int i = 0; i < 200; ++i) {
    open.update(1.0, 0.0, dt);
    closed.update(1.0, 0.0, dt);
  }
  EXPECT_LT(std::abs(closed.integrator()), 3.0);
  EXPECT_GT(std::abs(open.integrator()), 8.0);

  bool recovered = false;
  for (int i = 0; i < 40; ++i) {
    if (closed.update(-1.0, 0.0, dt) < 0.0) {
      recovered = true;
      break;
    }
  }
  EXPECT_TRUE(recovered);
}

TEST(Pid, DerivativeFilterAttenuatesHighFrequency) {
  fc::PidGains raw_gains;
  raw_gains.kd = 1.0;
  fc::PidGains filtered_gains = raw_gains;
  filtered_gains.derivative_filter_hz = 2.0;
  fc::Pid raw(raw_gains);
  fc::Pid filtered(filtered_gains);

  const double dt = 0.001;
  const double omega = 2.0 * std::numbers::pi * 40.0;
  double raw_peak = 0.0;
  double filtered_peak = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double measurement = std::sin(omega * t);
    raw_peak = std::max(raw_peak, std::abs(raw.update(0.0, measurement, dt)));
    filtered_peak = std::max(filtered_peak, std::abs(filtered.update(0.0, measurement, dt)));
  }
  EXPECT_GT(raw_peak, 100.0);
  EXPECT_LT(filtered_peak, raw_peak / 8.0);
}
