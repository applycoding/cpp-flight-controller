#include "fc/sim/quad_sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "fc/math/vec3.hpp"

namespace fc {

QuadSimulator::QuadSimulator(VehicleParams params) : params_(params) {}

void QuadSimulator::setState(const SimState& state) {
  state_ = state;
}

void QuadSimulator::setMotorThrusts(const MotorThrusts& thrusts) {
  thrusts_ = thrusts;
  for (double& thrust : thrusts_.newton) {
    thrust = std::clamp(thrust, 0.0, params_.max_thrust_per_motor_n);
  }
}

void QuadSimulator::setNoise(SimNoise noise, std::uint32_t seed) {
  noise_ = noise;
  rng_.seed(seed);
  noise_enabled_ = noise.gyro_std > 0.0 || noise.accel_std > 0.0 || noise.mag_std > 0.0 ||
                   noise.baro_std > 0.0 || norm(noise.gyro_bias_rad_s) > 0.0 ||
                   norm(noise.accel_bias_m_s2) > 0.0;
}

QuadSimulator::Wrench QuadSimulator::wrench() const {
  const auto motors = quadXMotors(params_.arm_m);
  Vec3 thrust_body{};
  Vec3 torque{};
  for (int i = 0; i < kMotorCount; ++i) {
    const double thrust = thrusts_.newton[static_cast<std::size_t>(i)];
    thrust_body.z -= thrust;
    torque += motorTorque(motors[static_cast<std::size_t>(i)], thrust, params_.yaw_torque_ratio_m);
  }
  const Vec3 drag_world =
      state_.velocity_ned_m_s * (-params_.quadratic_drag * norm(state_.velocity_ned_m_s));
  Wrench out;
  out.force_world = state_.attitude.rotate(thrust_body) + drag_world;
  out.torque_body = torque;
  return out;
}

Vec3 QuadSimulator::specificForceBody() const {
  return state_.attitude.rotateInverse(wrench().force_world / params_.mass_kg);
}

Vec3 QuadSimulator::kinematicAccelerationWorld() const {
  const Vec3 gravity{0.0, 0.0, params_.gravity_m_s2};
  return wrench().force_world / params_.mass_kg + gravity;
}

double QuadSimulator::gaussian() {
  // Box-Muller. A fixed seed makes noisy scenarios repeatable.
  std::uniform_real_distribution<double> uniform(1.0e-12, 1.0);
  const double u1 = uniform(rng_);
  const double u2 = uniform(rng_);
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

void QuadSimulator::step(double dt) {
  if (!(dt > 0.0)) {
    return;
  }
  const Wrench w = wrench();
  const Vec3 gravity{0.0, 0.0, params_.gravity_m_s2};
  const Vec3 acceleration = w.force_world / params_.mass_kg + gravity;

  const Vec3 inertia_omega{params_.inertia_xx * state_.omega_body_rad_s.x,
                           params_.inertia_yy * state_.omega_body_rad_s.y,
                           params_.inertia_zz * state_.omega_body_rad_s.z};
  const Vec3 gyroscopic = state_.omega_body_rad_s.cross(inertia_omega);
  const Vec3 omega_dot{(w.torque_body.x - gyroscopic.x) / params_.inertia_xx,
                       (w.torque_body.y - gyroscopic.y) / params_.inertia_yy,
                       (w.torque_body.z - gyroscopic.z) / params_.inertia_zz};

  state_.omega_body_rad_s += omega_dot * dt;
  state_.attitude = state_.attitude.integrate(state_.omega_body_rad_s, dt);
  state_.velocity_ned_m_s += acceleration * dt;
  state_.position_ned_m += state_.velocity_ned_m_s * dt;
}

ImuSample QuadSimulator::readImu(double time_s) {
  ImuSample sample;
  sample.time_s = time_s;
  sample.gyro_rad_s = state_.omega_body_rad_s + noise_.gyro_bias_rad_s;
  sample.accel_m_s2 = specificForceBody() + noise_.accel_bias_m_s2;
  sample.mag_body = state_.attitude.rotateInverse(normalized(defaultMagNed()));
  sample.baro_altitude_m = -state_.position_ned_m.z;
  sample.mag_valid = true;
  sample.baro_valid = true;

  if (noise_enabled_) {
    sample.gyro_rad_s += Vec3{gaussian(), gaussian(), gaussian()} * noise_.gyro_std;
    sample.accel_m_s2 += Vec3{gaussian(), gaussian(), gaussian()} * noise_.accel_std;
    sample.mag_body += Vec3{gaussian(), gaussian(), gaussian()} * noise_.mag_std;
    sample.baro_altitude_m += gaussian() * noise_.baro_std;
  }
  return sample;
}

}  // namespace fc
