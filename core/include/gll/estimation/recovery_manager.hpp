// 再アンカー・再位置推定・LOST の判定（設計書 3.13.3〜3.13.5 節）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state_history.hpp"

#include <deque>
#include <limits>
#include <optional>

namespace gll {

/// 再位置推定の依頼（推定値の周りで、複数の初期値から位置合わせを試す）。
struct RelocalizeRequest {
  double t = 0.0;
  SE2 center;              ///< 探索の中心（UTM）
  double radius = 1.0;     ///< [m]
  double yaw_range = 0.2;  ///< ±[rad]
};

struct RecoveryAction {
  enum class Kind { NONE, REANCHOR, RELOCALIZE };
  Kind kind = Kind::NONE;
  std::optional<Measurement> measurement;
  Mat3 inflation = Mat3::Zero();  ///< 誤差の座標系で P の左上 3×3 に加える量
  std::optional<RelocalizeRequest> relocalize;
};

class RecoveryManager {
 public:
  explicit RecoveryManager(const RecoveryConfig& cfg = RecoveryConfig(),
                           const RelocalizeConfig& reloc = RelocalizeConfig())
      : cfg_(cfg), reloc_(reloc) {}

  void onAccepted(const Measurement& m);

  /// ゲートに落ちた観測を受け取る。互いに一致する候補がそろえば再アンカーを、LiDAR の棄却が続けば
  /// 再位置推定を指示する。gnss_fix_active は、直近に RTK-FIX の GNSS を採用しているか。
  RecoveryAction onRejected(const Measurement& m, const UpdateResult& res, const StateHistory& hist,
                            const IStateEstimator& est, bool gnss_fix_active, double dr_distance = 0.0);

  /// LiDAR の位置合わせが品質の条件を満たさなかった（観測を作れなかった）ときに呼ぶ。棄却と同じく数え、
  /// 続けば再位置推定を指示する。
  RecoveryAction onLidarFailure(double t, const StateHistory& hist, const IStateEstimator& est, bool gnss_fix_active,
                                double dr_distance = 0.0);

  /// 再アンカーを実行した後に呼ぶ。
  void onReanchored();

  /// 再位置推定の結果を知らせる。失敗が max_attempts 回続いたら LOST にする（relocalize.lost_on_failure のとき）。
  void onRelocalizeResult(double t, bool success);

  /// 外部から初期姿勢を与えたときに呼ぶ（LOST などをすべて解除する）。
  void onExternalPose();

  /// 位置の不確かさから LOST を判定する。
  void updateLost(double pos_stddev, double lost_stddev) { updateLost(pos_stddev >= lost_stddev); }
  /// lost: 位置の共分散が LOST の条件に当たるか（StatusMonitor::lostByCovariance）。
  void updateLost(bool lost);

  /// LOST の間、lost_retry_interval ごとに再位置推定を試す時刻になったか。
  bool relocalizeDue(double t) const;
  /// 再位置推定の探索範囲。半径は、推定値の共分散の 3σ と、位置の観測なしで走った距離 dr_distance の
  /// radius_per_dr_distance 倍の大きい方を、[min_radius, max_radius] に収めたもの（widest なら最大）。
  RelocalizeRequest makeRelocalizeRequest(double t, const FilterState& st, const IStateEstimator& est, bool widest,
                                          double dr_distance = 0.0) const;

  RecoveryState state() const { return state_; }
  std::size_t reanchorCount() const { return reanchor_count_; }
  int relocalizeAttempts() const { return relocalize_attempts_; }

 private:
  RecoveryAction lidarRejectCount(double t, const StateHistory& hist, const IStateEstimator& est, bool gnss_fix_active,
                                  double dr_distance);

  RecoveryConfig cfg_;
  RelocalizeConfig reloc_;
  RecoveryState state_ = RecoveryState::TRACKING;
  std::deque<GnssPositionMeasurement> gnss_candidates_;
  std::deque<PoseMeasurement> lidar_candidates_;
  int lidar_rejects_ = 0;
  int relocalize_attempts_ = 0;
  bool lost_by_relocalize_ = false;
  double last_relocalize_t_ = -std::numeric_limits<double>::infinity();
  std::size_t reanchor_count_ = 0;
};

}  // namespace gll
