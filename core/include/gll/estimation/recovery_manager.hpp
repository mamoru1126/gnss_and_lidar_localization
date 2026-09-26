// 再アンカーと LOST の判定（設計書 3.13.3〜3.13.5 節）。
// Phase 1 は GNSS の再アンカーと LOST を扱う。LiDAR の再アンカーと再位置推定は Phase 2。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state_history.hpp"

#include <deque>
#include <optional>

namespace gll {

struct RecoveryAction {
  enum class Kind { NONE, REANCHOR };
  Kind kind = Kind::NONE;
  std::optional<Measurement> measurement;
  Mat3 inflation = Mat3::Zero();  ///< 誤差の座標系で P の左上 3×3 に加える量
};

class RecoveryManager {
 public:
  explicit RecoveryManager(const RecoveryConfig& cfg = RecoveryConfig()) : cfg_(cfg) {}

  void onAccepted(const Measurement& m);

  /// ゲートに落ちた観測を受け取り、互いに一致する候補がそろえば再アンカーを指示する。
  RecoveryAction onRejected(const Measurement& m, const UpdateResult& res, const StateHistory& hist,
                            const IStateEstimator& est);

  /// 再アンカーを実行した後に呼ぶ。
  void onReanchored();

  /// 位置の不確かさから LOST を判定する。
  void updateLost(double pos_stddev, double lost_stddev);

  RecoveryState state() const { return state_; }
  std::size_t reanchorCount() const { return reanchor_count_; }

 private:
  RecoveryConfig cfg_;
  RecoveryState state_ = RecoveryState::TRACKING;
  std::deque<GnssPositionMeasurement> gnss_candidates_;
  std::size_t reanchor_count_ = 0;
};

}  // namespace gll
