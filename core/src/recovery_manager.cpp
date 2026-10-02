#include "gll/estimation/recovery_manager.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace gll {
namespace {

Mat3 reanchorInflation(const UpdateResult& res, const RecoveryConfig& cfg) {
  Vec3 e = Vec3::Zero();
  for (int i = 0; i < std::min<int>(3, static_cast<int>(res.residual.size())); ++i) e(i) = res.residual(i);
  Mat3 inf = e.cwiseProduct(e).asDiagonal();
  inf(0, 0) += cfg.reanchor_extra_stddev_xy * cfg.reanchor_extra_stddev_xy;
  inf(1, 1) += cfg.reanchor_extra_stddev_xy * cfg.reanchor_extra_stddev_xy;
  inf(2, 2) += cfg.reanchor_extra_stddev_yaw * cfg.reanchor_extra_stddev_yaw;
  return inf;
}

}  // namespace

void RecoveryManager::onAccepted(const Measurement& m) {
  const MeasurementKind k = measurementKind(m);
  if (k == MeasurementKind::GNSS_POSITION) gnss_candidates_.clear();
  if (k == MeasurementKind::POSE) {
    lidar_candidates_.clear();
    lidar_rejects_ = 0;
  }
  if (k == MeasurementKind::GNSS_POSITION || k == MeasurementKind::POSE) {
    // 位置の観測をゲートを通して採用できたなら、推定値は観測と整合している
    relocalize_attempts_ = 0;
    lost_by_relocalize_ = false;
    if (state_ == RecoveryState::LOST || state_ == RecoveryState::RELOCALIZE) state_ = RecoveryState::TRACKING;
  }
  if (state_ == RecoveryState::SUSPECT) state_ = RecoveryState::TRACKING;
}

RecoveryAction RecoveryManager::onRejected(const Measurement& m, const UpdateResult& res, const StateHistory& hist,
                                           const IStateEstimator& est, bool gnss_fix_active, double dr_distance) {
  RecoveryAction act;
  if (state_ == RecoveryState::TRACKING) state_ = RecoveryState::SUSPECT;

  if (const auto* g = std::get_if<GnssPositionMeasurement>(&m)) {
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
      for (std::size_t j = i + 1; j < aligned.size(); ++j) spread = std::max(spread, (aligned[i] - aligned[j]).norm());
    if (spread > cfg_.reanchor_consistency_xy) return act;

    act.kind = RecoveryAction::Kind::REANCHOR;
    act.measurement = *g;
    act.inflation = reanchorInflation(res, cfg_);
    state_ = RecoveryState::REANCHOR;
    return act;
  }

  if (const auto* z = std::get_if<PoseMeasurement>(&m)) {
    lidar_candidates_.push_back(*z);
    while (!lidar_candidates_.empty() && lidar_candidates_.front().t < z->t - cfg_.candidate_max_age)
      lidar_candidates_.pop_front();
    if (static_cast<int>(lidar_candidates_.size()) >= cfg_.reanchor_confirm_lidar) {
      // GNSS と同じく、各候補の「観測 − その時刻の推定値」（世界座標系の位置と yaw）が一致するかを見る
      std::vector<Vec3> d;
      const std::size_t n = lidar_candidates_.size();
      const std::size_t first = n - static_cast<std::size_t>(cfg_.reanchor_confirm_lidar);
      bool ok = true;
      for (std::size_t i = first; i < n && ok; ++i) {
        const auto& c = lidar_candidates_[i];
        const auto x_i = hist.stateAt(c.t, est);
        if (!x_i) {
          ok = false;
          break;
        }
        d.emplace_back(c.Z.t().x() - x_i->X.t().x(), c.Z.t().y() - x_i->X.t().y(), wrapAngle(c.Z.yaw() - x_i->X.yaw()));
      }
      if (ok) {
        double sxy = 0.0, syaw = 0.0;
        for (std::size_t i = 0; i < d.size(); ++i)
          for (std::size_t j = i + 1; j < d.size(); ++j) {
            sxy = std::max(sxy, (d[i].head<2>() - d[j].head<2>()).norm());
            syaw = std::max(syaw, std::abs(wrapAngle(d[i](2) - d[j](2))));
          }
        if (sxy <= cfg_.reanchor_consistency_xy && syaw <= cfg_.reanchor_consistency_yaw) {
          act.kind = RecoveryAction::Kind::REANCHOR;
          act.measurement = *z;
          act.inflation = reanchorInflation(res, cfg_);
          state_ = RecoveryState::REANCHOR;
          return act;
        }
      }
    }
    return lidarRejectCount(z->t, hist, est, gnss_fix_active, dr_distance);
  }
  return act;
}

RecoveryAction RecoveryManager::onLidarFailure(double t, const StateHistory& hist, const IStateEstimator& est,
                                               bool gnss_fix_active, double dr_distance) {
  if (state_ == RecoveryState::TRACKING) state_ = RecoveryState::SUSPECT;
  return lidarRejectCount(t, hist, est, gnss_fix_active, dr_distance);
}

RecoveryAction RecoveryManager::lidarRejectCount(double t, const StateHistory& hist, const IStateEstimator& est,
                                                 bool gnss_fix_active, double dr_distance) {
  RecoveryAction act;
  ++lidar_rejects_;
  // GNSS が FIX している間は GNSS を優先し、LiDAR の失敗では再位置推定しない（設計書 3.13.4 節）
  if (gnss_fix_active || lidar_rejects_ < reloc_.after_rejects || state_ == RecoveryState::RELOCALIZE ||
      state_ == RecoveryState::LOST || hist.empty())
    return act;
  act.kind = RecoveryAction::Kind::RELOCALIZE;
  act.relocalize = makeRelocalizeRequest(t, hist.latest(), est, false, dr_distance);
  state_ = RecoveryState::RELOCALIZE;
  lidar_rejects_ = 0;
  lidar_candidates_.clear();
  return act;
}

RelocalizeRequest RecoveryManager::makeRelocalizeRequest(double t, const FilterState& st, const IStateEstimator& est,
                                                         bool widest, double dr_distance) const {
  RelocalizeRequest r;
  r.t = t;
  r.center = st.X;
  const Mat3 cw = est.worldCovariance(st);
  const double sxy = std::sqrt(std::max(cw(0, 0), cw(1, 1)));
  const double syaw = std::sqrt(std::max(cw(2, 2), 0.0));
  // フィルタの共分散は、モデル化していない誤差（スリップなど）があると小さすぎることがあるので、
  // 位置の観測なしで走った距離からも下限を決める
  const double r_needed = std::max(3.0 * sxy, reloc_.radius_per_dr_distance * dr_distance);
  r.radius = widest ? reloc_.max_radius : std::clamp(r_needed, reloc_.min_radius, reloc_.max_radius);
  r.yaw_range = widest ? reloc_.max_yaw : std::clamp(3.0 * syaw, reloc_.init_min_yaw, reloc_.max_yaw);
  return r;
}

void RecoveryManager::onReanchored() {
  gnss_candidates_.clear();
  lidar_candidates_.clear();
  lidar_rejects_ = 0;
  relocalize_attempts_ = 0;
  lost_by_relocalize_ = false;
  ++reanchor_count_;
  state_ = RecoveryState::TRACKING;
}

void RecoveryManager::onRelocalizeResult(double t, bool success) {
  last_relocalize_t_ = t;
  if (success) {
    relocalize_attempts_ = 0;
    state_ = RecoveryState::REANCHOR;
    return;
  }
  ++relocalize_attempts_;
  if (!reloc_.lost_on_failure && state_ != RecoveryState::LOST) {
    // LOST は位置の不確かさだけで決める。照合がまた続けて捨てられたら、もう一度再位置推定する
    state_ = RecoveryState::SUSPECT;
    return;
  }
  if (relocalize_attempts_ >= reloc_.max_attempts || state_ == RecoveryState::LOST) {
    state_ = RecoveryState::LOST;
    lost_by_relocalize_ = true;
  } else {
    state_ = RecoveryState::SUSPECT;
  }
}

void RecoveryManager::onExternalPose() {
  gnss_candidates_.clear();
  lidar_candidates_.clear();
  lidar_rejects_ = 0;
  relocalize_attempts_ = 0;
  lost_by_relocalize_ = false;
  state_ = RecoveryState::TRACKING;
}

void RecoveryManager::updateLost(bool lost) {
  if (lost) {
    state_ = RecoveryState::LOST;
  } else if (state_ == RecoveryState::LOST && !lost_by_relocalize_) {
    state_ = RecoveryState::TRACKING;
  }
}

bool RecoveryManager::relocalizeDue(double t) const {
  return state_ == RecoveryState::LOST && t - last_relocalize_t_ >= reloc_.lost_retry_interval;
}

}  // namespace gll
