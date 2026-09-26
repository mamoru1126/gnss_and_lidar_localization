// IMU と ODOM から予測の入力を作る（設計書 3.4 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state.hpp"
#include "gll/measurement/attitude_estimator.hpp"

#include <deque>
#include <optional>

namespace gll {

class MotionInputBuilder {
 public:
  explicit MotionInputBuilder(const MotionConfig& cfg = MotionConfig()) : cfg_(cfg) {}

  void addOdom(const OdomSample& o);
  bool hasOdom() const { return !odom_.empty(); }

  /// 時刻 t の前進速度・横速度（ODOM の線形補間。外挿は odom_hold_max まで最新値を保持）。
  /// ODOM が古すぎる場合は stale = true。
  struct Velocity {
    double v = 0.0;
    double v_lat = 0.0;
    double v_dot = 0.0;
    bool stale = false;
  };
  std::optional<Velocity> velocityAt(double t) const;

  /// IMU サンプルから予測の入力を作る（傾斜補正込み）。
  std::optional<MotionInput> build(const ImuSample& imu, const AttitudeEstimator& att) const;

  /// IMU が来ないときの代替: ODOM のヨーレートを使う。
  std::optional<MotionInput> buildFromOdom(const OdomSample& o) const;

  const std::optional<OdomSample>& latestOdom() const { return latest_; }

 private:
  MotionConfig cfg_;
  std::deque<OdomSample> odom_;
  std::optional<OdomSample> latest_;
};

}  // namespace gll
