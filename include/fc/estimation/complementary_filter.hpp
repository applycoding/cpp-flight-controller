#pragma once

#include "fc/math/quaternion.hpp"
#include "fc/types.hpp"

namespace fc {

// Explicit complementary filter (Mahony) on the quaternion.
// Gyro integrates the attitude. Accelerometer and magnetometer corrections are
// applied as a feedback rate. Accel feedback is skipped when the specific-force
// residual is large, which happens whenever the vehicle is linearly accelerating:
// a flying quad's accelerometer measures thrust, not gravity, until acceleration
// returns to ~0.
class ComplementaryFilter {
 public:
  ComplementaryFilter() = default;
  explicit ComplementaryFilter(const EstimatorConfig& config);

  void setConfig(const EstimatorConfig& config);
  // `aligned` false: accept large accel residuals until the estimate locks (bench /
  // pre-flight alignment). `aligned` true: a large residual is treated as linear
  // acceleration and the accel correction is skipped.
  void reset(Quaternion attitude, bool aligned);

  // Returns false when the accelerometer correction was rejected.
  bool update(Vec3 gyro_rad_s, Vec3 accel_m_s2, double dt, const Vec3* mag_body);

  Quaternion attitude() const { return q_; }
  EulerAngles euler() const { return q_.toEuler(); }
  Vec3 gyroBias() const { return bias_; }
  bool lastAccelAccepted() const { return accel_accepted_; }

 private:
  bool accelCorrectionAllowed(double residual) const;

  EstimatorConfig config_{};
  Quaternion q_{};
  Vec3 bias_{};
  bool accel_accepted_{false};
  bool aligned_{false};
  int consistent_updates_{0};
};

}  // namespace fc
