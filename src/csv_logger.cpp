#include "fc/telemetry/csv_logger.hpp"

#include <iomanip>

namespace fc {

CsvLogger::CsvLogger(const std::string& path) : out_(path) {
  out_ << std::setprecision(10);
  out_ << "t_s,roll_deg,pitch_deg,yaw_deg,roll_sp_deg,pitch_sp_deg,yaw_sp_deg,"
          "roll_ekf_deg,pitch_ekf_deg,yaw_ekf_deg,roll_comp_deg,pitch_comp_deg,yaw_comp_deg,"
          "roll_gyro_deg,pitch_gyro_deg,yaw_gyro_deg,alt_m,alt_sp_m,alt_est_m,"
          "p_rad_s,q_rad_s,r_rad_s,bias_x,bias_y,bias_z,bias_ekf_x,bias_ekf_y,bias_ekf_z,"
          "m0_n,m1_n,m2_n,m3_n\n";
}

void CsvLogger::write(const TelemetrySample& s) {
  out_ << s.time_s << ',' << s.roll_deg << ',' << s.pitch_deg << ',' << s.yaw_deg << ','
       << s.roll_sp_deg << ',' << s.pitch_sp_deg << ',' << s.yaw_sp_deg << ',' << s.roll_ekf_deg
       << ',' << s.pitch_ekf_deg << ',' << s.yaw_ekf_deg << ',' << s.roll_comp_deg << ','
       << s.pitch_comp_deg << ',' << s.yaw_comp_deg << ',' << s.roll_gyro_deg << ','
       << s.pitch_gyro_deg << ',' << s.yaw_gyro_deg << ',' << s.alt_m << ',' << s.alt_sp_m << ','
       << s.alt_est_m << ',' << s.p_rad_s << ',' << s.q_rad_s << ',' << s.r_rad_s << ',' << s.bias_x
       << ',' << s.bias_y << ',' << s.bias_z << ',' << s.bias_ekf_x << ',' << s.bias_ekf_y << ','
       << s.bias_ekf_z << ',' << s.motor_n[0] << ',' << s.motor_n[1] << ',' << s.motor_n[2] << ','
       << s.motor_n[3] << '\n';
}

}  // namespace fc
