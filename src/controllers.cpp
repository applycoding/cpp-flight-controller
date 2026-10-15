#include "fc/control/controllers.hpp"

#include <algorithm>

namespace fc {
namespace {

double clampAbs(double value, double limit) {
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

}  // namespace

AttitudeController::AttitudeController(AttitudeGains gains) {
  setGains(gains);
}

void AttitudeController::setGains(AttitudeGains gains) {
  gains_ = gains;
  roll_.setGains(gains.roll_rate);
  pitch_.setGains(gains.pitch_rate);
  yaw_.setGains(gains.yaw_rate);
}

void AttitudeController::reset() {
  roll_.reset();
  pitch_.reset();
  yaw_.reset();
}

AttitudeOutput AttitudeController::update(const Setpoint& setpoint, Quaternion attitude,
                                          Vec3 gyro_rad_s, double dt) {
  const Quaternion q_set =
      Quaternion::fromEuler({setpoint.roll_rad, setpoint.pitch_rad, setpoint.yaw_rad});
  const Vec3 angle_error = attitude.errorRotationVectorTo(q_set);

  AttitudeOutput out;
  out.angle_error_rad = angle_error;
  out.rate_setpoint_rad_s = {
      clampAbs(gains_.roll_angle_p * angle_error.x, gains_.max_rate_rad_s),
      clampAbs(gains_.pitch_angle_p * angle_error.y, gains_.max_rate_rad_s),
      clampAbs(gains_.yaw_angle_p * angle_error.z, gains_.max_rate_rad_s),
  };

  out.torque_nm = {
      roll_.update(out.rate_setpoint_rad_s.x - gyro_rad_s.x, gyro_rad_s.x, dt),
      pitch_.update(out.rate_setpoint_rad_s.y - gyro_rad_s.y, gyro_rad_s.y, dt),
      yaw_.update(out.rate_setpoint_rad_s.z - gyro_rad_s.z, gyro_rad_s.z, dt),
  };
  return out;
}

AltitudeController::AltitudeController(AltitudeGains gains) {
  setGains(gains);
}

void AltitudeController::setGains(AltitudeGains gains) {
  gains_ = gains;
  vertical_.setGains(gains.vertical);
}

void AltitudeController::reset() {
  vertical_.reset();
}

double AltitudeController::update(double altitude_setpoint_m, double altitude_m, double dt) {
  const double error = altitude_setpoint_m - altitude_m;
  const double delta = vertical_.update(error, altitude_m, dt);
  return gains_.hover_thrust_n + delta;
}

}  // namespace fc
