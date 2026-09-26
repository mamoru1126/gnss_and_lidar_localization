// 初期化（GNSS 区間）。設計書 3.11 節、アルゴリズム説明書 8.2 節。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state_estimator.hpp"

#include <optional>

namespace gll {

class Initializer {
 public:
  enum class Phase { WAIT_FIX, WAIT_MOTION, CONVERGING, READY };

  explicit Initializer(const InitConfig& cfg = InitConfig()) : cfg_(cfg) {}

  /// RTK-FIX の位置観測を与える。粗い yaw が決まったら初期状態を返す。
  /// forward_sign は走行方向（前進 +1、後退 -1）。gyro_bias は静止初期化で求めた値。
  std::optional<FilterState> onGnss(const GnssPositionMeasurement& m, double forward_sign,
                                    double gyro_bias, bool estimate_scale);

  /// 外部から与えた初期姿勢で初期状態を作る。
  FilterState fromExternalPose(double t, const Pose2D& pose, const Mat3& cov_world, double gyro_bias,
                               bool estimate_scale, const IStateEstimator& est);

  /// 共分散が十分小さくなったら READY にする。
  bool updateReady(const FilterState& st, const IStateEstimator& est);

  Phase phase() const { return phase_; }
  bool ready() const { return phase_ == Phase::READY; }
  void reset() { phase_ = Phase::WAIT_FIX; }

 private:
  InitConfig cfg_;
  Phase phase_ = Phase::WAIT_FIX;
  Vec2 anchor_ = Vec2::Zero();
};

}  // namespace gll
