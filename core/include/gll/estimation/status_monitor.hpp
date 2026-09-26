// どの観測が効いているかの状態（設計書 3.12 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state.hpp"

namespace gll {

class StatusMonitor {
 public:
  explicit StatusMonitor(const MonitorConfig& cfg = MonitorConfig()) : cfg_(cfg) {}

  /// 観測を採用したときに呼ぶ。位置の観測（GNSS 位置・LiDAR 姿勢）ならデッドレコニング距離を 0 に戻す。
  void onAccepted(MeasurementKind kind, double t);

  /// 予測で進んだ距離を積算する（デッドレコニング距離）。
  void addTravel(double distance) { dr_distance_ += distance; }
  /// 初期化・外部の初期姿勢などで位置が確定したときに呼ぶ。
  void resetTravel() { dr_distance_ = 0.0; }
  double drDistance() const { return dr_distance_; }
  bool drDistanceExceeded() const { return dr_distance_ > cfg_.dr_error_distance; }
  double drErrorDistance() const { return cfg_.dr_error_distance; }

  /// cov_world は出力の共分散（世界座標系の x, y, yaw）。
  LocalizationStatus evaluate(double t, bool ready, const Mat3& cov_world, bool offset_exceeded,
                              RecoveryState recovery) const;

  double lastGnssTime() const { return last_gnss_; }
  double lastLidarTime() const { return last_lidar_; }

 private:
  MonitorConfig cfg_;
  double last_gnss_ = -1e18;
  double last_lidar_ = -1e18;
  double dr_distance_ = 0.0;
};

}  // namespace gll
