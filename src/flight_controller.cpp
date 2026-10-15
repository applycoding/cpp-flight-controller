#include "fc/flight_controller.hpp"

namespace fc {

FlightController::FlightController(VehicleParams vehicle, AttitudeGains attitude,
                                   AltitudeGains altitude, EstimatorConfig estimator)
    : vehicle_(vehicle),
      estimator_config_(estimator),
      ekf_(estimator),
      complementary_(estimator),
      altitude_kf_(estimator),
      attitude_(attitude),
      altitude_(altitude),
      mixer_(vehicle.arm_m, vehicle.yaw_torque_ratio_m, vehicle.max_thrust_per_motor_n) {}

void FlightController::initialize(Quaternion attitude, double altitude_m, bool aligned) {
  ekf_.reset(attitude, aligned);
  complementary_.reset(attitude, aligned);
  altitude_kf_.reset(altitude_m, 0.0);
  attitude_.reset();
  altitude_.reset();
  corrected_gyro_ = {};
}

void FlightController::estimate(const ImuSample& sample, double dt) {
  const Vec3* mag = sample.mag_valid ? &sample.mag_body : nullptr;
  complementary_.update(sample.gyro_rad_s, sample.accel_m_s2, dt, mag);

  ekf_.propagate(sample.gyro_rad_s, dt);
  ekf_.updateAccel(sample.accel_m_s2);
  if (sample.mag_valid) {
    ekf_.updateMag(sample.mag_body);
  }
  corrected_gyro_ = ekf_.correctedRate(sample.gyro_rad_s);

  const Vec3 specific_world = ekf_.attitude().rotate(sample.accel_m_s2);
  const double accel_up = -specific_world.z - vehicle_.gravity_m_s2;
  altitude_kf_.predict(accel_up, dt);
  if (sample.baro_valid) {
    altitude_kf_.updateBaro(sample.baro_altitude_m);
  }
}

MotorThrusts FlightController::control(const Setpoint& setpoint, double dt) {
  const AttitudeOutput attitude = attitude_.update(setpoint, ekf_.attitude(), corrected_gyro_, dt);
  const double thrust = altitude_.update(setpoint.altitude_m, altitude_kf_.altitude(), dt);
  return mixer_.mix(thrust, attitude.torque_nm);
}

}  // namespace fc
