#pragma once

#include <fstream>
#include <string>

#include "fc/scenario.hpp"

namespace fc {

class CsvLogger {
 public:
  explicit CsvLogger(const std::string& path);

  bool ok() const { return static_cast<bool>(out_); }
  void write(const TelemetrySample& sample);

 private:
  std::ofstream out_;
};

}  // namespace fc
