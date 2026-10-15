#pragma once

#include "fc/math/matrix.hpp"

namespace fc {

// Vertical-channel Kalman filter. Altitude is positive up.
// Specific force is an input (INS mechanization); barometer is the measurement.
class AltitudeKf {
 public:
  AltitudeKf() = default;
  explicit AltitudeKf(const EstimatorConfig& config);

  void setConfig(const EstimatorConfig& config);
  void reset(double altitude_m, double vertical_speed_m_s);

  void predict(double accel_up_m_s2, double dt);
  void updateBaro(double altitude_m);

  double altitude() const { return altitude_; }
  double verticalSpeed() const { return vertical_speed_; }

 private:
  EstimatorConfig config_{};
  double altitude_{0.0};
  double vertical_speed_{0.0};
  Matrix<2, 2> p_{Matrix<2, 2>::identity()};
};

}  // namespace fc
