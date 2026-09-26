// 位置合わせの結果から LiDAR の姿勢観測を作る（設計書 3.7 節・6.2 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state.hpp"
#include "gll/matching/scan_matcher.hpp"

#include <optional>

namespace gll {

enum class LidarRejectReason { NONE, NOT_CONVERGED, FEW_INLIERS, LOW_OVERLAP, JUMP };

inline const char* toString(LidarRejectReason r) {
  switch (r) {
    case LidarRejectReason::NONE: return "NONE";
    case LidarRejectReason::NOT_CONVERGED: return "NOT_CONVERGED";
    case LidarRejectReason::FEW_INLIERS: return "FEW_INLIERS";
    case LidarRejectReason::LOW_OVERLAP: return "LOW_OVERLAP";
    case LidarRejectReason::JUMP: return "JUMP";
  }
  return "UNKNOWN";
}

struct LidarMeasurementResult {
  std::optional<PoseMeasurement> pose;
  LidarRejectReason reason = LidarRejectReason::NONE;
};

class LidarMeasurementBuilder {
 public:
  explicit LidarMeasurementBuilder(const LidarConfig& cfg = LidarConfig()) : cfg_(cfg) {}

  /// 品質（収束・インライア率・overlap・初期値からの移動量）を確かめ、UTM の姿勢観測を作る。
  /// init は照合の初期値（地図座標系）。変換にはターゲットのグループのアンカーを使う（設計書 5.5 節）。
  LidarMeasurementResult build(double t, const RegistrationResult& r, const MatchTarget& target,
                               const Eigen::Isometry3d& init) const;

  /// 観測共分散（機体座標系の x, y, yaw）: cov_scale · N_inlier · H⁻¹ の該当成分 + Σ_floor。
  Mat3 covarianceBody(const RegistrationResult& r) const;

  /// 品質の確認だけを行う（初期値からの移動量は見ない）。
  LidarRejectReason checkQuality(const RegistrationResult& r) const;

 private:
  LidarConfig cfg_;
};

}  // namespace gll
