#include "Simulation.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>

namespace {
void writePose(std::ostream& file, const sil::Pose& pose) {
  file << ',' << pose.x << ',' << pose.y << ',' << pose.yaw;
}
} // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "用法：sil_sim 输出目录\n";
    return 2;
  }
  const struct { const char* name; sil::Scenario scenario; } scenarios[] = {
    {"straight", sil::Scenario::Straight}, {"turn90", sil::Scenario::Turn90},
    {"sensor_error", sil::Scenario::SensorError}, {"watchdog", sil::Scenario::Watchdog},
    {"estop", sil::Scenario::Estop},
  };
  for (const auto& item : scenarios) {
    const auto rows = sil::run(item.scenario);
    const std::string path = std::string(argv[1]) + "/" + item.name + ".csv";
    std::ofstream file(path);
    file.imbue(std::locale::classic());
    file << "t_ms,connected,accepted,latched,allowed,linear_mm_s,angular_mrad_s,"
            "true_x_m,true_y_m,true_yaw_rad,enc_x_m,enc_y_m,enc_yaw_rad,"
            "fused_x_m,fused_y_m,fused_yaw_rad,left_cmd_rad_s,right_cmd_rad_s,event\n";
    file << std::fixed << std::setprecision(9);
    for (const auto& row : rows) {
      file << row.t << ',' << row.connected << ',' << row.accepted << ','
           << row.latched << ',' << row.allowed << ',' << row.linear << ',' << row.angular;
      writePose(file, row.truth);
      writePose(file, row.encoder);
      writePose(file, row.fused);
      file << ',' << row.leftRadPerSec << ',' << row.rightRadPerSec << ','
           << row.event << '\n';
    }
    file.close();
    if (!file) {
      std::cerr << "无法写入 " << path << '\n';
      return 1;
    }
    std::cout << path << " (" << rows.size() << " 个采样)\n";
  }
}
