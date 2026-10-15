#pragma once

#include "fc/control/controllers.hpp"
#include "fc/control/mixer.hpp"
#include "fc/estimation/altitude_kf.hpp"
#include "fc/estimation/attitude_ekf.hpp"
#include "fc/estimation/complementary_filter.hpp"

namespace fc {

// Cascaded attitude/rate and altitude controllers fed by the EKF.
// The complementary filter runs in parallel so a log can compare the two.
class FlightController {
 public:
  FlightController(VehicleParams vehicle, AttitudeGains attitude, AltitudeGains altitude,
                   EstimatorConfig estimator);

  // `aligned` should be true when `attitude` is already a good estimate (in-flight
  // handoff). False runs the accel acquisition path.
  void initialize(Quaternion attitude, double altitude_m, bool aligned);

  void estimate(const ImuSample& sample, double dt);
  MotorThrusts control(const Setpoint& setpoint, double dt);

  Quaternion ekfAttitude() const { return ekf_.attitude(); }
  EulerAngles ekfEuler() const { return ekf_.euler(); }
  EulerAngles complementaryEuler() const { return complementary_.euler(); }
  Vec3 ekfGyroBias() const { return ekf_.gyroBias(); }
  Vec3 complementaryGyroBias() const { return complementary_.gyroBias(); }
  double altitudeEstimate() const { return altitude_kf_.altitude(); }
  double verticalSpeedEstimate() const { return altitude_kf_.verticalSpeed(); }
  Vec3 correctedGyro() const { return corrected_gyro_; }
  bool ekfAccelAccepted() const { return ekf_.lastAccelAccepted(); }
  const VehicleParams& vehicle() const { return vehicle_; }

 private:
  VehicleParams vehicle_{};
  EstimatorConfig estimator_config_{};
  AttitudeEkf ekf_{};
  ComplementaryFilter complementary_{};
  AltitudeKf altitude_kf_{};
  AttitudeController attitude_;
  AltitudeController altitude_;
  QuadXMixer mixer_{};
  Vec3 corrected_gyro_{};
};

}  // namespace fc
