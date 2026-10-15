#include "fc/estimation/complementary_filter.hpp"

#include <cmath>

#include "fc/math/vec3.hpp"

namespace fc {

ComplementaryFilter::ComplementaryFilter(const EstimatorConfig& config) {
  setConfig(config);
}

void ComplementaryFilter::setConfig(const EstimatorConfig& config) {
  config_ = config;
}

void ComplementaryFilter::reset(Quaternion attitude, bool aligned) {
  q_ = attitude.normalized();
  bias_ = {};
  accel_accepted_ = false;
  aligned_ = aligned;
  consistent_updates_ = 0;
}

bool ComplementaryFilter::accelCorrectionAllowed(double residual) const {
  if (!aligned_) {
    return true;
  }
  return residual <= config_.accel_gate_m_s2;
}

bool ComplementaryFilter::update(Vec3 gyro_rad_s, Vec3 accel_m_s2, double dt,
                                 const Vec3* mag_body) {
  accel_accepted_ = false;
  if (!(dt > 0.0)) {
    return false;
  }

  Vec3 correction{};
  const double accel_norm = norm(accel_m_s2);
  if (accel_norm > 1.0e-3 && config_.gravity_m_s2 > 0.0) {
    const Vec3 measured = accel_m_s2 / accel_norm;
    const Vec3 predicted = q_.rotateInverse(Vec3{0.0, 0.0, -1.0});
    const Vec3 error = measured.cross(predicted);
    const double residual = norm(accel_m_s2 - predicted * config_.gravity_m_s2);
    if (accelCorrectionAllowed(residual)) {
      correction += error * config_.comp_kp;
      // bias is subtracted from the gyro, so the integrator sign is opposite the
      // proportional correction (Mahony: ω = ω_gyro - b + kp e, ḃ = -ki e).
      bias_ -= error * (config_.comp_ki * dt);
      accel_accepted_ = true;
      if (residual <= config_.accel_gate_m_s2) {
        ++consistent_updates_;
        if (consistent_updates_ > 40) {
          aligned_ = true;
        }
      }
    } else {
      consistent_updates_ = 0;
    }

    if (mag_body != nullptr) {
      const Vec3 mag_n = normalized(*mag_body);
      const Vec3 mag_world = normalized(config_.mag_ned);
      if (norm(mag_n) > 0.0 && norm(mag_world) > 0.0) {
        const Vec3 mag_predicted = q_.rotateInverse(mag_world);
        const Vec3 mag_error = mag_n.cross(mag_predicted);
        // Keep the part along the predicted down axis so mag primarily corrects yaw
        // and does not fight a rejected accelerometer update on tilt.
        const Vec3 down = -predicted;
        correction += down * (mag_error.dot(down) * config_.comp_mag_kp);
      }
    }
  }

  const Vec3 omega = gyro_rad_s - bias_ + correction;
  q_ = q_.integrate(omega, dt);
  return accel_accepted_;
}

}  // namespace fc
