#include "fc/estimation/altitude_kf.hpp"

namespace fc {

AltitudeKf::AltitudeKf(const EstimatorConfig& config) {
  setConfig(config);
}

void AltitudeKf::setConfig(const EstimatorConfig& config) {
  config_ = config;
}

void AltitudeKf::reset(double altitude_m, double vertical_speed_m_s) {
  altitude_ = altitude_m;
  vertical_speed_ = vertical_speed_m_s;
  p_ = Matrix<2, 2>::identity();
  p_(0, 0) = 1.0;
  p_(1, 1) = 1.0;
}

void AltitudeKf::predict(double accel_up_m_s2, double dt) {
  if (!(dt > 0.0)) {
    return;
  }
  altitude_ += vertical_speed_ * dt + 0.5 * accel_up_m_s2 * dt * dt;
  vertical_speed_ += accel_up_m_s2 * dt;

  Matrix<2, 2> f = Matrix<2, 2>::identity();
  f(0, 1) = dt;
  const double q = config_.alt_accel_noise * config_.alt_accel_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Matrix<2, 2> process = Matrix<2, 2>::zero();
  process(0, 0) = 0.25 * dt4 * q;
  process(0, 1) = 0.5 * dt3 * q;
  process(1, 0) = process(0, 1);
  process(1, 1) = dt2 * q;
  p_ = f.times(p_).times(f.transpose()) + process;
  p_.symmetrize();
}

void AltitudeKf::updateBaro(double altitude_m) {
  const double variance = config_.baro_noise_m * config_.baro_noise_m;
  const double s = p_(0, 0) + variance;
  if (!(s > 0.0)) {
    return;
  }
  const double k0 = p_(0, 0) / s;
  const double k1 = p_(1, 0) / s;
  const double innovation = altitude_m - altitude_;
  altitude_ += k0 * innovation;
  vertical_speed_ += k1 * innovation;

  Matrix<2, 2> next = p_;
  next(0, 0) = p_(0, 0) - k0 * p_(0, 0);
  next(0, 1) = p_(0, 1) - k0 * p_(0, 1);
  next(1, 0) = p_(1, 0) - k1 * p_(0, 0);
  next(1, 1) = p_(1, 1) - k1 * p_(0, 1);
  p_ = next;
  p_.symmetrize();
  // Keep the covariance usable if a long open-loop stretch drives a diagonal negative.
  if (p_(0, 0) < 1.0e-9) {
    p_(0, 0) = 1.0e-9;
  }
  if (p_(1, 1) < 1.0e-9) {
    p_(1, 1) = 1.0e-9;
  }
}

}  // namespace fc
