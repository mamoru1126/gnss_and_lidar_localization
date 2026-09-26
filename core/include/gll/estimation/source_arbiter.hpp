// GNSS / LiDAR の優先度と食い違いの判定（設計書 3.13.1〜3.13.2 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state.hpp"

#include <map>
#include <string>

namespace gll {

enum class LidarDecision { FUSE_PRIMARY, FUSE_INFLATED, REJECT_MISMATCH };

struct MismatchStats {
  std::size_t count = 0;
  Vec3 mean_world = Vec3::Zero();
  Vec3 m2_world = Vec3::Zero();  ///< 分散計算用（Welford）
  Vec3 stddev() const;
};

class SourceArbiter {
 public:
  explicit SourceArbiter(const ArbiterConfig& arb = ArbiterConfig(),
                         const MonitorConfig& mon = MonitorConfig())
      : cfg_(arb), aid_timeout_(mon.aid_timeout) {}

  void onGnssAccepted(double t);
  bool isGnssFixActive(double t) const { return t - last_gnss_fix_ <= aid_timeout_; }

  GatePolicy gatePolicy(const Measurement& m) const;

  /// GNSS FIX 中の LiDAR の扱いを決める。FUSE_INFLATED のときは m の共分散を膨らませる。
  /// st は観測時刻の推定値。GNSS FIX 中は、推定値との差を地図グループごとのアンカーずれとして記録する。
  LidarDecision classifyLidar(PoseMeasurement& m, const FilterState& st);

  /// LiDAR の観測に使うゲートの扱い。GNSS FIX 中に整合した LiDAR は固定閾値で判定済みなので SKIP、
  /// それ以外は Mahalanobis ゲートで判定し、落ちたら再アンカーの候補にする（設計書 3.13.3 節）。
  static GatePolicy lidarPolicy(LidarDecision d) {
    return d == LidarDecision::FUSE_INFLATED ? GatePolicy::SKIP : GatePolicy::DEFER_TO_RECOVERY;
  }

  const std::map<std::string, MismatchStats>& mismatchStats() const { return mismatch_; }

 private:
  ArbiterConfig cfg_;
  double aid_timeout_;
  double last_gnss_fix_ = -1e18;
  std::map<std::string, MismatchStats> mismatch_;
};

}  // namespace gll
