#pragma once

#include "fc/math/matrix.hpp"
#include "fc/math/quaternion.hpp"

namespace fc {

// Error-state extended Kalman filter for attitude and gyro bias.
// Nominal state: body-to-world quaternion and body gyro bias.
// Error state: small-angle attitude error δθ (right-multiplicative) and δb.
//
// Gyro is an input. Accelerometer and magnetometer are measurements.
// The accelerometer model is h = R^T * (0, 0, -g), i.e. it is only consistent
// when linear acceleration is negligible. Updates whose innovation exceeds the
// configured gate are rejected so a hard maneuver does not yank the estimate
// toward "level" just because thrust is along the body axis.
class AttitudeEkf {
 public:
  AttitudeEkf() = default;
  explicit AttitudeEkf(const EstimatorConfig& config);

  void setConfig(const EstimatorConfig& config);
  // See ComplementaryFilter::reset for the meaning of `aligned`.
  void reset(Quaternion attitude, bool aligned);

  void propagate(Vec3 gyro_rad_s, double dt);
  bool updateAccel(Vec3 accel_m_s2);
  bool updateMag(Vec3 mag_body);

  Quaternion attitude() const { return q_; }
  EulerAngles euler() const { return q_.toEuler(); }
  Vec3 gyroBias() const { return bias_; }
  Vec3 correctedRate(Vec3 gyro_rad_s) const { return gyro_rad_s - bias_; }
  bool lastAccelAccepted() const { return accel_accepted_; }

 private:
  bool updateVector(Vec3 measured, Vec3 predicted, double noise_variance);

  bool accelCorrectionAllowed(double residual) const;

  EstimatorConfig config_{};
  Quaternion q_{};
  Vec3 bias_{};
  Vec3 last_omega_{};
  Matrix<6, 6> p_{Matrix<6, 6>::identity()};
  bool accel_accepted_{false};
  bool aligned_{false};
  int consistent_updates_{0};
};

}  // namespace fc
