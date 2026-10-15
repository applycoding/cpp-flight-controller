#pragma once

#include "fc/control/pid.hpp"
#include "fc/math/quaternion.hpp"

namespace fc {

struct AttitudeOutput {
  Vec3 torque_nm{};
  Vec3 rate_setpoint_rad_s{};
  Vec3 angle_error_rad{};
};

class AttitudeController {
 public:
  AttitudeController() = default;
  explicit AttitudeController(AttitudeGains gains);

  void setGains(AttitudeGains gains);
  void reset();

  AttitudeOutput update(const Setpoint& setpoint, Quaternion attitude, Vec3 gyro_rad_s, double dt);

 private:
  AttitudeGains gains_{};
  Pid roll_;
  Pid pitch_;
  Pid yaw_;
};

class AltitudeController {
 public:
  AltitudeController() = default;
  explicit AltitudeController(AltitudeGains gains);

  void setGains(AltitudeGains gains);
  void reset();

  // Returns total thrust in newtons (collective).
  double update(double altitude_setpoint_m, double altitude_m, double dt);

 private:
  AltitudeGains gains_{};
  Pid vertical_;
};

}  // namespace fc
