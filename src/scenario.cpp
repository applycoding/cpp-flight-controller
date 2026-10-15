#include "fc/scenario.hpp"

#include <atomic>
#include <cmath>
#include <numbers>
#include <thread>

#include "fc/math/quaternion.hpp"
#include "fc/rt/fixed_rate_loop.hpp"
#include "fc/rt/spsc_ring_buffer.hpp"

namespace fc {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

double toDeg(double rad) {
  return rad / kDeg;
}

TelemetrySample makeSample(double time_s, const SimState& state, const Setpoint& setpoint,
                           const FlightController& fc, Quaternion gyro_only,
                           const MotorThrusts& motors, Vec3 gyro_bias) {
  const EulerAngles truth = state.attitude.toEuler();
  const EulerAngles ekf = fc.ekfEuler();
  const EulerAngles comp = fc.complementaryEuler();
  const EulerAngles gyro = gyro_only.toEuler();
  const Vec3 bias_hat = fc.ekfGyroBias();
  TelemetrySample row;
  row.time_s = time_s;
  row.roll_deg = toDeg(truth.roll);
  row.pitch_deg = toDeg(truth.pitch);
  row.yaw_deg = toDeg(truth.yaw);
  row.roll_sp_deg = toDeg(setpoint.roll_rad);
  row.pitch_sp_deg = toDeg(setpoint.pitch_rad);
  row.yaw_sp_deg = toDeg(setpoint.yaw_rad);
  row.roll_ekf_deg = toDeg(ekf.roll);
  row.pitch_ekf_deg = toDeg(ekf.pitch);
  row.yaw_ekf_deg = toDeg(ekf.yaw);
  row.roll_comp_deg = toDeg(comp.roll);
  row.pitch_comp_deg = toDeg(comp.pitch);
  row.yaw_comp_deg = toDeg(comp.yaw);
  row.roll_gyro_deg = toDeg(gyro.roll);
  row.pitch_gyro_deg = toDeg(gyro.pitch);
  row.yaw_gyro_deg = toDeg(gyro.yaw);
  row.alt_m = -state.position_ned_m.z;
  row.alt_sp_m = setpoint.altitude_m;
  row.alt_est_m = fc.altitudeEstimate();
  row.p_rad_s = state.omega_body_rad_s.x;
  row.q_rad_s = state.omega_body_rad_s.y;
  row.r_rad_s = state.omega_body_rad_s.z;
  row.bias_x = gyro_bias.x;
  row.bias_y = gyro_bias.y;
  row.bias_z = gyro_bias.z;
  row.bias_ekf_x = bias_hat.x;
  row.bias_ekf_y = bias_hat.y;
  row.bias_ekf_z = bias_hat.z;
  for (int i = 0; i < kMotorCount; ++i) {
    row.motor_n[i] = motors.newton[i];
  }
  return row;
}

MotorThrusts hoverThrust(const VehicleParams& vehicle) {
  MotorThrusts motors;
  const double each = vehicle.mass_kg * vehicle.gravity_m_s2 / static_cast<double>(kMotorCount);
  for (double& motor : motors.newton) {
    motor = each;
  }
  return motors;
}

}  // namespace

Setpoint demoSetpoint(double time_s) {
  Setpoint setpoint;
  setpoint.altitude_m = 0.0;
  if (time_s >= 4.0) {
    setpoint.altitude_m = 0.5;
  }
  if (time_s >= 7.0 && time_s < 9.5) {
    setpoint.roll_rad = 12.0 * kDeg;
  }
  if (time_s >= 10.0 && time_s < 12.5) {
    setpoint.pitch_rad = 10.0 * kDeg;
  }
  if (time_s >= 13.0) {
    setpoint.yaw_rad = 20.0 * kDeg;
  }
  return setpoint;
}

std::vector<TelemetrySample> runClosedLoop(const ScenarioConfig& config) {
  QuadSimulator sim(config.vehicle);
  SimState state;
  state.attitude = Quaternion::fromEuler(config.initial_euler);
  sim.setState(state);

  SimNoise noise;
  noise.gyro_bias_rad_s = config.gyro_bias_rad_s;
  if (config.sensor_noise) {
    noise.gyro_std = config.gyro_noise;
    noise.accel_std = config.accel_noise;
    noise.mag_std = config.mag_noise;
    noise.baro_std = config.baro_noise;
  }
  sim.setNoise(noise, config.seed);

  FlightController fc(config.vehicle, demoAttitudeGains(), demoAltitudeGains(config.vehicle),
                      demoEstimatorConfig(config.vehicle));
  fc.initialize(state.attitude, 0.0, true);

  MotorThrusts motors = hoverThrust(config.vehicle);
  sim.setMotorThrusts(motors);
  Quaternion gyro_only = state.attitude;

  const int steps = static_cast<int>(std::llround(config.duration_s / config.dt_s));
  std::vector<TelemetrySample> rows;
  rows.reserve(static_cast<std::size_t>(steps));

  for (int i = 0; i < steps; ++i) {
    const double time_s = static_cast<double>(i) * config.dt_s;
    const Setpoint setpoint = demoSetpoint(time_s);
    const ImuSample imu = sim.readImu(time_s);
    fc.estimate(imu, config.dt_s);
    motors = fc.control(setpoint, config.dt_s);
    rows.push_back(
        makeSample(time_s, sim.state(), setpoint, fc, gyro_only, motors, config.gyro_bias_rad_s));
    sim.setMotorThrusts(motors);
    sim.step(config.dt_s);
    gyro_only = gyro_only.integrate(imu.gyro_rad_s, config.dt_s);
  }
  return rows;
}

ThreadedSimStats runThreadedHover(double duration_s, double rate_hz) {
  const VehicleParams vehicle;
  QuadSimulator sim(vehicle);
  FlightController fc(vehicle, demoAttitudeGains(), demoAltitudeGains(vehicle),
                      demoEstimatorConfig(vehicle));
  fc.initialize(Quaternion{}, 0.0, true);

  SpscRingBuffer<ImuSample, 1024> imu_ring;
  SpscRingBuffer<MotorThrusts, 256> motor_ring;

  const double hz = rate_hz > 1.0 ? rate_hz : 1.0;
  const double dt = 1.0 / hz;
  MotorThrusts motors = hoverThrust(vehicle);
  sim.setMotorThrusts(motors);

  std::atomic<bool> plant_done{false};
  std::atomic<std::uint64_t> pushed{0};
  std::atomic<std::uint64_t> dropped{0};
  std::atomic<std::uint64_t> control_steps{0};

  std::thread plant([&] {
    FixedRateLoop pace(hz);
    double time_s = 0.0;
    MotorThrusts applied = motors;
    const int max_steps = static_cast<int>(duration_s * hz) + 2;
    for (int i = 0; i < max_steps; ++i) {
      pace.wait();
      MotorThrusts command{};
      while (motor_ring.pop(command)) {
        applied = command;
      }
      sim.setMotorThrusts(applied);
      sim.step(dt);
      time_s += dt;
      ImuSample sample = sim.readImu(time_s);
      if (imu_ring.push(sample)) {
        pushed.fetch_add(1, std::memory_order_relaxed);
      } else {
        dropped.fetch_add(1, std::memory_order_relaxed);
      }
    }
    plant_done.store(true, std::memory_order_release);
  });

  std::thread control([&] {
    FixedRateLoop pace(hz);
    double last_time = 0.0;
    const int max_spins = static_cast<int>(duration_s * hz) + 50;
    for (int i = 0; i < max_spins; ++i) {
      pace.wait();
      bool got = false;
      ImuSample sample{};
      while (imu_ring.pop(sample)) {
        double sample_dt = sample.time_s - last_time;
        if (!(sample_dt > 0.0) || sample_dt > 0.05) {
          sample_dt = dt;
        }
        fc.estimate(sample, sample_dt);
        last_time = sample.time_s;
        got = true;
      }
      if (got) {
        const MotorThrusts command = fc.control(Setpoint{}, dt);
        if (motor_ring.push(command)) {
          control_steps.fetch_add(1, std::memory_order_relaxed);
        }
      }
      if (plant_done.load(std::memory_order_acquire) && !got) {
        break;
      }
    }
  });

  plant.join();
  control.join();

  ThreadedSimStats stats;
  stats.imu_pushed = pushed.load();
  stats.imu_dropped = dropped.load();
  stats.control_steps = control_steps.load();
  const EulerAngles euler = sim.state().attitude.toEuler();
  stats.final_roll_rad = euler.roll;
  stats.final_pitch_rad = euler.pitch;
  stats.final_yaw_rad = euler.yaw;
  stats.final_alt_m = -sim.state().position_ned_m.z;
  return stats;
}

}  // namespace fc
