#pragma once

#include <array>
#include <random>

#include "fc/math/quaternion.hpp"
#include "fc/model/quad_x.hpp"
#include "fc/types.hpp"

namespace fc {

struct SimState {
  Vec3 position_ned_m{};
  Vec3 velocity_ned_m_s{};
  Quaternion attitude{};
  Vec3 omega_body_rad_s{};
};

struct SimNoise {
  double gyro_std{0.0};
  double accel_std{0.0};
  double mag_std{0.0};
  double baro_std{0.0};
  Vec3 gyro_bias_rad_s{};
  Vec3 accel_bias_m_s2{};
};

// Rigid-body quadcopter. Motor thrust is applied instantly along -body z.
// The IMU reports true specific force (thrust and drag), not a synthetic gravity vector.
class QuadSimulator {
 public:
  explicit QuadSimulator(VehicleParams params = {});

  void setState(const SimState& state);
  void setMotorThrusts(const MotorThrusts& thrusts);
  void setNoise(SimNoise noise, std::uint32_t seed);

  const SimState& state() const { return state_; }
  const VehicleParams& params() const { return params_; }
  Vec3 specificForceBody() const;
  Vec3 kinematicAccelerationWorld() const;

  void step(double dt);
  ImuSample readImu(double time_s);

 private:
  struct Wrench {
    Vec3 force_world{};
    Vec3 torque_body{};
  };

  Wrench wrench() const;
  double gaussian();

  VehicleParams params_{};
  SimState state_{};
  MotorThrusts thrusts_{};
  SimNoise noise_{};
  std::mt19937 rng_{1};
  bool noise_enabled_{false};
};

}  // namespace fc
