#include "fc/estimation/attitude_ekf.hpp"

#include <cmath>

#include "fc/math/vec3.hpp"

namespace fc {

AttitudeEkf::AttitudeEkf(const EstimatorConfig& config) {
  setConfig(config);
}

void AttitudeEkf::setConfig(const EstimatorConfig& config) {
  config_ = config;
  reset(q_, aligned_);
}

void AttitudeEkf::reset(Quaternion attitude, bool aligned) {
  q_ = attitude.normalized();
  bias_ = {};
  last_omega_ = {};
  p_ = Matrix<6, 6>::zero();
  // An in-flight handoff is already aligned, so the covariance stays tight.
  // Acquisition starts uncertain enough to pull in from a bad bench attitude.
  const double att_std = aligned ? 0.015 : config_.initial_attitude_std_rad;
  const double bias_std = aligned ? 0.025 : config_.initial_bias_std;
  for (int i = 0; i < 3; ++i) {
    p_(i, i) = att_std * att_std;
    p_(i + 3, i + 3) = bias_std * bias_std;
  }
  accel_accepted_ = false;
  aligned_ = aligned;
  consistent_updates_ = 0;
}

bool AttitudeEkf::accelCorrectionAllowed(double residual) const {
  if (!aligned_) {
    return true;
  }
  return residual <= config_.accel_gate_m_s2;
}

void AttitudeEkf::propagate(Vec3 gyro_rad_s, double dt) {
  if (!(dt > 0.0)) {
    return;
  }
  const Vec3 omega = gyro_rad_s - bias_;
  last_omega_ = omega;
  q_ = q_.integrate(omega, dt);

  Matrix<6, 6> f = Matrix<6, 6>::identity();
  const Matrix<3, 3> omega_skew = skew(omega);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      f(r, c) -= omega_skew(r, c) * dt;
    }
    f(r, r + 3) -= dt;
  }

  Matrix<6, 6> qn = Matrix<6, 6>::zero();
  const double q_gyro = config_.gyro_noise * config_.gyro_noise * dt;
  const double q_bias = config_.bias_random_walk * config_.bias_random_walk * dt;
  for (int i = 0; i < 3; ++i) {
    qn(i, i) = q_gyro;
    qn(i + 3, i + 3) = q_bias;
  }

  p_ = f.times(p_).times(f.transpose()) + qn;
  p_.symmetrize();
}

bool AttitudeEkf::updateAccel(Vec3 accel_m_s2) {
  accel_accepted_ = false;
  const double accel_norm = norm(accel_m_s2);
  // A free-flying quad's accelerometer reads thrust. Only treat it as gravity when
  // the magnitude is near 1g; anything else is linear acceleration.
  if (std::abs(accel_norm - config_.gravity_m_s2) > 0.55) {
    consistent_updates_ = 0;
    return false;
  }
  if (aligned_ && norm(last_omega_) > 0.35) {
    return false;
  }
  const Vec3 predicted = q_.rotateInverse(Vec3{0.0, 0.0, -config_.gravity_m_s2});
  const double residual = norm(accel_m_s2 - predicted);
  if (!accelCorrectionAllowed(residual)) {
    consistent_updates_ = 0;
    return false;
  }
  const double variance = config_.accel_noise * config_.accel_noise;
  accel_accepted_ = updateVector(accel_m_s2, predicted, variance);
  if (accel_accepted_ && residual <= config_.accel_gate_m_s2) {
    ++consistent_updates_;
    if (consistent_updates_ > 25) {
      aligned_ = true;
    }
  }
  return accel_accepted_;
}

bool AttitudeEkf::updateMag(Vec3 mag_body) {
  const Vec3 field = normalized(config_.mag_ned);
  const Vec3 measured = normalized(mag_body);
  if (norm(field) <= 0.0 || norm(measured) <= 0.0) {
    return false;
  }
  const Vec3 predicted = q_.rotateInverse(field);
  // A huge residual means the linearisation is not trustworthy (or the field is garbage).
  if (norm(measured - predicted) > 0.7) {
    return false;
  }
  const double variance = config_.mag_noise * config_.mag_noise;
  return updateVector(measured, predicted, variance);
}

bool AttitudeEkf::updateVector(Vec3 measured, Vec3 predicted, double noise_variance) {
  Matrix<3, 6> h = Matrix<3, 6>::zero();
  const Matrix<3, 3> dh = skew(predicted);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      h(r, c) = dh(r, c);
    }
  }

  const Matrix<6, 3> pht = p_.times(h.transpose());
  Matrix<3, 3> s = h.times(pht);
  for (int i = 0; i < 3; ++i) {
    s(i, i) += noise_variance;
  }
  Matrix<3, 3> s_inv = Matrix<3, 3>::zero();
  if (!invert3(s, s_inv)) {
    return false;
  }

  const Matrix<6, 3> k = pht.times(s_inv);
  Matrix<3, 1> innovation = Matrix<3, 1>::zero();
  const Vec3 residual = measured - predicted;
  innovation(0, 0) = residual.x;
  innovation(1, 0) = residual.y;
  innovation(2, 0) = residual.z;
  const Matrix<6, 1> dx = k.times(innovation);

  const Vec3 dtheta{dx(0, 0), dx(1, 0), dx(2, 0)};
  const Vec3 dbias{dx(3, 0), dx(4, 0), dx(5, 0)};
  q_ = (q_ * Quaternion::fromRotationVector(dtheta)).normalized();
  bias_ += dbias;

  const Matrix<6, 6> kh = k.times(h);
  const Matrix<6, 6> i_kh = Matrix<6, 6>::identity() - kh;
  Matrix<3, 3> r = Matrix<3, 3>::zero();
  for (int i = 0; i < 3; ++i) {
    r(i, i) = noise_variance;
  }
  p_ = i_kh.times(p_).times(i_kh.transpose()) + k.times(r).times(k.transpose());
  p_.symmetrize();
  return true;
}

}  // namespace fc
