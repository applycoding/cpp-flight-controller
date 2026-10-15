#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "fc/scenario.hpp"
#include "fc/telemetry/csv_logger.hpp"

namespace {

void printUsage() {
  std::cerr << "Usage: fc_sim_demo [--output telemetry.csv] [--no-noise] [--duration seconds]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string output = "telemetry.csv";
  fc::ScenarioConfig config;
  config.sensor_noise = true;
  config.duration_s = 16.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--no-noise") {
      config.sensor_noise = false;
    } else if (arg == "--output" && i + 1 < argc) {
      output = argv[++i];
    } else if (arg == "--duration" && i + 1 < argc) {
      config.duration_s = std::atof(argv[++i]);
    } else if (arg == "--help") {
      printUsage();
      return 0;
    } else {
      printUsage();
      return 2;
    }
  }

  if (!(config.duration_s > 0.0)) {
    std::cerr << "Duration must be positive.\n";
    return 2;
  }

  const std::vector<fc::TelemetrySample> rows = fc::runClosedLoop(config);
  fc::CsvLogger logger(output);
  if (!logger.ok()) {
    std::cerr << "Could not open " << output << " for writing.\n";
    return 1;
  }
  for (const fc::TelemetrySample& row : rows) {
    logger.write(row);
  }

  const fc::TelemetrySample& last = rows.back();
  std::cout << "Wrote " << rows.size() << " samples to " << output << '\n';
  std::cout << "Final truth roll/pitch/yaw deg: " << last.roll_deg << ", " << last.pitch_deg << ", "
            << last.yaw_deg << '\n';
  std::cout << "Final EKF  roll/pitch/yaw deg: " << last.roll_ekf_deg << ", " << last.pitch_ekf_deg
            << ", " << last.yaw_ekf_deg << '\n';
  std::cout << "Final altitude m (truth / estimate / setpoint): " << last.alt_m << " / "
            << last.alt_est_m << " / " << last.alt_sp_m << '\n';
  std::cout << "Gyro bias true: " << last.bias_x << ", " << last.bias_y << ", " << last.bias_z
            << '\n';
  std::cout << "Gyro bias EKF:  " << last.bias_ekf_x << ", " << last.bias_ekf_y << ", "
            << last.bias_ekf_z << '\n';
  return 0;
}
