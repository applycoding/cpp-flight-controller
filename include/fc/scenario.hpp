#pragma once

#include <cstdint>
#include <vector>

#include "fc/flight_controller.hpp"
#include "fc/sim/quad_sim.hpp"

namespace fc {

struct TelemetrySample {
  double time_s{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
  double roll_sp_deg{0.0};
  double pitch_sp_deg{0.0};
  double yaw_sp_deg{0.0};
  double roll_ekf_deg{0.0};
  double pitch_ekf_deg{0.0};
  double yaw_ekf_deg{0.0};
  double roll_comp_deg{0.0};
  double pitch_comp_deg{0.0};
  double yaw_comp_deg{0.0};
  double roll_gyro_deg{0.0};
  double pitch_gyro_deg{0.0};
  double yaw_gyro_deg{0.0};
  double alt_m{0.0};
  double alt_sp_m{0.0};
  double alt_est_m{0.0};
  double p_rad_s{0.0};
  double q_rad_s{0.0};
  double r_rad_s{0.0};
  double bias_x{0.0};
  double bias_y{0.0};
  double bias_z{0.0};
  double bias_ekf_x{0.0};
  double bias_ekf_y{0.0};
  double bias_ekf_z{0.0};
  double motor_n[4]{0.0, 0.0, 0.0, 0.0};
};

struct ScenarioConfig {
  VehicleParams vehicle{};
  double dt_s{0.001};
  double duration_s{16.0};
  EulerAngles initial_euler{0.174532925199, -0.104719755120, 0.209439510239};  // 10, -6, 12 deg
  Vec3 gyro_bias_rad_s{0.02, -0.012, 0.015};
  bool sensor_noise{false};
  std::uint32_t seed{7};
  double gyro_noise{0.004};
  double accel_noise{0.08};
  double mag_noise{0.01};
  double baro_noise{0.03};
};

// Attitude and altitude schedule used by the demo and the closed-loop test.
Setpoint demoSetpoint(double time_s);

std::vector<TelemetrySample> runClosedLoop(const ScenarioConfig& config);

struct ThreadedSimStats {
  std::uint64_t imu_pushed{0};
  std::uint64_t imu_dropped{0};
  std::uint64_t control_steps{0};
  double final_roll_rad{0.0};
  double final_pitch_rad{0.0};
  double final_yaw_rad{0.0};
  double final_alt_m{0.0};
};

// Sensor thread and control thread exchange IMU samples and motor commands
// through SPSC rings. The plant is paced with a fixed-rate sleep.
ThreadedSimStats runThreadedHover(double duration_s, double rate_hz);

}  // namespace fc
