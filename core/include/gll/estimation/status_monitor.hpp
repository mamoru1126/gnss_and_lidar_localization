// どの観測が効いているかの状態（設計書 3.12 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state.hpp"

namespace gll {

class StatusMonitor {
 public:
  explicit StatusMonitor(const MonitorConfig& cfg = MonitorConfig()) : cfg_(cfg) {}

  void onAccepted(MeasurementKind kind, double t);

  /// cov_world は出力の共分散（世界座標系の x, y, yaw）。
  LocalizationStatus evaluate(double t, bool ready, const Mat3& cov_world, bool offset_exceeded,
                              RecoveryState recovery) const;

  double lastGnssTime() const { return last_gnss_; }
  double lastLidarTime() const { return last_lidar_; }

 private:
  MonitorConfig cfg_;
  double last_gnss_ = -1e18;
  double last_lidar_ = -1e18;
};

}  // namespace gll
