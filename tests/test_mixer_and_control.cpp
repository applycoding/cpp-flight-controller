#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "fc/control/controllers.hpp"
#include "fc/control/mixer.hpp"
#include "fc/model/quad_x.hpp"

TEST(Mixer, UnsaturatedAllocationMatchesRigidBodyWrench) {
  const fc::VehicleParams vehicle;
  fc::QuadXMixer mixer(vehicle.arm_m, vehicle.yaw_torque_ratio_m, vehicle.max_thrust_per_motor_n);
  const double thrust = 10.0;
  const fc::Vec3 torque{0.08, -0.05, 0.02};
  const fc::MotorThrusts motors = mixer.mix(thrust, torque);

  const auto layout = fc::quadXMotors(vehicle.arm_m);
  fc::Vec3 recovered;
  double sum = 0.0;
  for (int i = 0; i < fc::kMotorCount; ++i) {
    sum += motors.newton[i];
    recovered += fc::motorTorque(layout[static_cast<std::size_t>(i)], motors.newton[i],
                                 vehicle.yaw_torque_ratio_m);
  }
  EXPECT_NEAR(sum, thrust, 1.0e-6);
  EXPECT_NEAR(recovered.x, torque.x, 1.0e-6);
  EXPECT_NEAR(recovered.y, torque.y, 1.0e-6);
  EXPECT_NEAR(recovered.z, torque.z, 1.0e-6);
}

TEST(Mixer, PositiveRollLoadsTheLeftMotors) {
  const fc::VehicleParams vehicle;
  fc::QuadXMixer mixer(vehicle.arm_m, vehicle.yaw_torque_ratio_m, vehicle.max_thrust_per_motor_n);
  const fc::MotorThrusts motors = mixer.mix(10.0, {0.15, 0.0, 0.0});
  EXPECT_GT(motors.newton[2], motors.newton[0]);
  EXPECT_GT(motors.newton[3], motors.newton[1]);
}

TEST(Mixer, SaturatedCommandStaysInsideMotorLimits) {
  const fc::VehicleParams vehicle;
  fc::QuadXMixer mixer(vehicle.arm_m, vehicle.yaw_torque_ratio_m, vehicle.max_thrust_per_motor_n);
  const fc::MotorThrusts motors = mixer.mix(12.0, {5.0, -5.0, 2.0});
  double sum = 0.0;
  for (double thrust : motors.newton) {
    EXPECT_GE(thrust, -1.0e-9);
    EXPECT_LE(thrust, vehicle.max_thrust_per_motor_n + 1.0e-9);
    sum += thrust;
  }
  EXPECT_NEAR(sum, 12.0, 1.0e-3);
}

TEST(AttitudeController, RollErrorCommandsARestoringTorque) {
  fc::AttitudeController controller(fc::demoAttitudeGains());
  fc::Setpoint setpoint;
  const fc::Quaternion tilted = fc::Quaternion::fromEuler({0.2, -0.15, 0.3});
  const fc::AttitudeOutput out = controller.update(setpoint, tilted, {}, 0.01);
  EXPECT_LT(out.rate_setpoint_rad_s.x, 0.0);
  EXPECT_LT(out.torque_nm.x, 0.0);
  EXPECT_GT(out.rate_setpoint_rad_s.y, 0.0);
  EXPECT_GT(out.torque_nm.y, 0.0);
  EXPECT_LT(out.rate_setpoint_rad_s.z, 0.0);
  EXPECT_LT(out.torque_nm.z, 0.0);
  EXPECT_LT(std::abs(out.rate_setpoint_rad_s.x), fc::demoAttitudeGains().max_rate_rad_s + 1.0e-9);
}

TEST(AttitudeController, YawTakesTheShortPath) {
  fc::AttitudeController controller(fc::demoAttitudeGains());
  fc::Setpoint setpoint;
  setpoint.yaw_rad = 170.0 * std::numbers::pi / 180.0;
  const fc::Quaternion attitude =
      fc::Quaternion::fromEuler({0.0, 0.0, -170.0 * std::numbers::pi / 180.0});
  const fc::AttitudeOutput out = controller.update(setpoint, attitude, {}, 0.01);
  // Shortest rotation is about -20 deg, so the yaw-rate command stays modest and negative.
  EXPECT_LT(out.rate_setpoint_rad_s.z, -0.4);
  EXPECT_GT(out.rate_setpoint_rad_s.z, -1.6);
}
