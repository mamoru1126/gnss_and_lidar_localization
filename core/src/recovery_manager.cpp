#include "gll/estimation/recovery_manager.hpp"

#include <algorithm>
#include <vector>

namespace gll {

void RecoveryManager::onAccepted(const Measurement& m) {
  if (measurementKind(m) == MeasurementKind::GNSS_POSITION) gnss_candidates_.clear();
  if (state_ == RecoveryState::SUSPECT) state_ = RecoveryState::TRACKING;
}

RecoveryAction RecoveryManager::onRejected(const Measurement& m, const UpdateResult& res,
                                           const StateHistory& hist, const IStateEstimator& est) {
  RecoveryAction act;
  if (state_ == RecoveryState::TRACKING) state_ = RecoveryState::SUSPECT;
  const auto* g = std::get_if<GnssPositionMeasurement>(&m);
  if (!g) return act;  // LiDAR の再アンカーは Phase 2

  gnss_candidates_.push_back(*g);
  while (!gnss_candidates_.empty() && gnss_candidates_.front().t < g->t - cfg_.candidate_max_age)
    gnss_candidates_.pop_front();
  if (static_cast<int>(gnss_candidates_.size()) < cfg_.reanchor_confirm_gnss) return act;

  // 各候補について「観測したアンテナ位置 − その時刻の推定から予測したアンテナ位置」を世界座標系で求める。
  // 推定値がずれているなら、短い区間ではこのずれはほぼ一定になる（推定の相対運動は正確なため）。
  // 観測側が外れているなら、ずれはばらつく。
  std::vector<Vec2> aligned;
  const std::size_t n = gnss_candidates_.size();
  const std::size_t first = n - static_cast<std::size_t>(cfg_.reanchor_confirm_gnss);
  for (std::size_t i = first; i < n; ++i) {
    const auto& c = gnss_candidates_[i];
    const auto x_i = hist.stateAt(c.t, est);
    if (!x_i) return act;
    aligned.push_back(c.y - x_i->X.act(c.lever_h));
  }
  double spread = 0.0;
  for (std::size_t i = 0; i < aligned.size(); ++i)
    for (std::size_t j = i + 1; j < aligned.size(); ++j)
      spread = std::max(spread, (aligned[i] - aligned[j]).norm());
  if (spread > cfg_.reanchor_consistency_xy) return act;

  // 互いに一致 → 推定値の方がずれていると判断して再アンカー
  act.kind = RecoveryAction::Kind::REANCHOR;
  act.measurement = *g;
  Vec3 e = Vec3::Zero();
  if (res.residual.size() >= 2) e.head<2>() = res.residual.head<2>();
  act.inflation = e.cwiseProduct(e).asDiagonal();
  act.inflation(0, 0) += cfg_.reanchor_extra_stddev_xy * cfg_.reanchor_extra_stddev_xy;
  act.inflation(1, 1) += cfg_.reanchor_extra_stddev_xy * cfg_.reanchor_extra_stddev_xy;
  act.inflation(2, 2) += cfg_.reanchor_extra_stddev_yaw * cfg_.reanchor_extra_stddev_yaw;
  state_ = RecoveryState::REANCHOR;
  return act;
}

void RecoveryManager::onReanchored() {
  gnss_candidates_.clear();
  ++reanchor_count_;
  state_ = RecoveryState::TRACKING;
}

void RecoveryManager::updateLost(double pos_stddev, double lost_stddev) {
  if (pos_stddev >= lost_stddev) {
    state_ = RecoveryState::LOST;
  } else if (state_ == RecoveryState::LOST) {
    state_ = RecoveryState::TRACKING;
  }
}

}  // namespace gll
