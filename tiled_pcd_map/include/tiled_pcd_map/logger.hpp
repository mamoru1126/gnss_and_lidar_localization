// ログ出力のインターフェース。ROS のロガーには IF 層が橋渡しする。
#pragma once

#include <iostream>
#include <string>

namespace tiled_pcd_map {

class ILogger {
 public:
  virtual ~ILogger() = default;
  virtual void debug(const std::string& msg) = 0;
  virtual void info(const std::string& msg) = 0;
  virtual void warn(const std::string& msg) = 0;
  virtual void error(const std::string& msg) = 0;
};

class NullLogger : public ILogger {
 public:
  void debug(const std::string&) override {}
  void info(const std::string&) override {}
  void warn(const std::string&) override {}
  void error(const std::string&) override {}
};

class StderrLogger : public ILogger {
 public:
  void debug(const std::string&) override {}
  void info(const std::string& m) override { std::cerr << "[INFO] " << m << '\n'; }
  void warn(const std::string& m) override { std::cerr << "[WARN] " << m << '\n'; }
  void error(const std::string& m) override { std::cerr << "[ERROR] " << m << '\n'; }
};

}  // namespace tiled_pcd_map
