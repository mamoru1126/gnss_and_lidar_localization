// GNSS の採用判定と観測の生成（設計書 3.6 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/common/geodesy.hpp"
#include "gll/estimation/state.hpp"
#include "gll/measurement/attitude_estimator.hpp"

#include <optional>

namespace gll {

enum class GnssRejectReason { NONE, NOT_RTK_FIX, COVARIANCE_UNKNOWN, STDDEV_TOO_LARGE, SETTLING };

inline const char* toString(GnssRejectReason r) {
  switch (r) {
    case GnssRejectReason::NONE: return "NONE";
    case GnssRejectReason::NOT_RTK_FIX: return "NOT_RTK_FIX";
    case GnssRejectReason::COVARIANCE_UNKNOWN: return "COVARIANCE_UNKNOWN";
    case GnssRejectReason::STDDEV_TOO_LARGE: return "STDDEV_TOO_LARGE";
    case GnssRejectReason::SETTLING: return "SETTLING";
  }
  return "UNKNOWN";
}

struct GnssPositionResult {
  std::optional<GnssPositionMeasurement> position;
  GnssFixType fix = GnssFixType::NONE;
  GnssRejectReason reason = GnssRejectReason::NONE;
};

class GnssMeasurementBuilder {
 public:
  explicit GnssMeasurementBuilder(const GnssConfig& cfg = GnssConfig());

  GnssFixType classify(const GnssSample& s) const;

  /// 採用条件を満たせば位置観測を返す。att は観測時刻の roll / pitch を取るのに使う。
  GnssPositionResult buildPosition(const GnssSample& s, const AttitudeEstimator& att);

  /// 進行方位の観測（任意）。rtk_fix_active は直近に RTK-FIX を採用しているか。
  std::optional<HeadingMeasurement> buildHeading(const GnssVelocitySample& s, double yaw_rate,
                                                 bool rtk_fix_active) const;

  const UtmProjector& projector() const { return utm_; }

  /// 観測時刻の roll / pitch でレバーアームを水平面に射影する。
  Vec2 projectLeverArm(double roll, double pitch) const;

  /// アンテナの楕円体高から、base_link の楕円体高を求める（LiDAR の照合の z の初期値に使う。設計書 3.9 節）。
  double baseHeight(const GnssSample& s, const AttitudeEstimator& att) const;

 private:
  GnssConfig cfg_;
  UtmProjector utm_;
  double fix_since_ = -1.0;
  double last_sample_t_ = -1.0;
  double last_convergence_ = 0.0;
};

}  // namespace gll
