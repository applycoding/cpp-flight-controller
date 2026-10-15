#pragma once

#include "fc/model/quad_x.hpp"

namespace fc {

// Allocates a collective thrust and a body torque onto four Quad-X motors.
// If the exact allocation would saturate a motor, torque is scaled down and
// collective thrust is kept (attitude authority yields to staying airborne).
class QuadXMixer {
 public:
  QuadXMixer() = default;
  QuadXMixer(double arm_m, double yaw_torque_ratio_m, double max_thrust_per_motor_n);

  MotorThrusts mix(double total_thrust_n, Vec3 torque_nm) const;

  double arm() const { return arm_m_; }
  double yawRatio() const { return yaw_ratio_m_; }
  double maxThrustPerMotor() const { return max_thrust_n_; }

 private:
  MotorThrusts allocate(double total_thrust_n, Vec3 torque_nm) const;
  bool inRange(const MotorThrusts& thrusts) const;

  double arm_m_{0.17};
  double yaw_ratio_m_{0.018};
  double max_thrust_n_{6.5};
};

}  // namespace fc
