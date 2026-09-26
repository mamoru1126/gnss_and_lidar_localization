#include "gll/estimation/state_history.hpp"

namespace gll {

void StateHistory::reset(const FilterState& st) {
  buf_.clear();
  MotionInput u;
  u.t = st.t;
  buf_.push_back(Entry{st, u});
}

void StateHistory::push(const FilterState& st, const MotionInput& u) {
  buf_.push_back(Entry{st, u});
  prune();
}

void StateHistory::prune() {
  if (buf_.empty()) return;
  const double t_min = buf_.back().state.t - length_;
  while (buf_.size() > 1 && buf_[1].state.t <= t_min) buf_.pop_front();
}

int StateHistory::indexAtOrBefore(double t) const {
  for (int i = static_cast<int>(buf_.size()) - 1; i >= 0; --i) {
    if (buf_[i].state.t <= t) return i;
  }
  return -1;
}

std::optional<FilterState> StateHistory::stateAt(double t, const IStateEstimator& est) const {
  if (buf_.empty()) return std::nullopt;
  const int i = indexAtOrBefore(t);
  if (i < 0) return std::nullopt;
  const FilterState& s = buf_[i].state;
  if (t <= s.t) return s;
  const MotionInput& u = (i + 1 < static_cast<int>(buf_.size())) ? buf_[i + 1].input : buf_[i].input;
  return est.predict(s, u, t - s.t);
}

std::optional<SE2> StateHistory::relativeMotion(double t1, double t2, const IStateEstimator& est) const {
  const auto a = stateAt(t1, est);
  const auto b = stateAt(t2, est);
  if (!a || !b) return std::nullopt;
  return a->X.inverse() * b->X;
}

UpdateResult StateHistory::applyDelayed(const Measurement& m, const IStateEstimator& est,
                                        const MahalanobisGate& gate, GatePolicy policy,
                                        const std::optional<Mat3>& inflation) {
  UpdateResult res;
  if (buf_.empty()) return res;
  const double t = measurementTime(m);
  const int n = static_cast<int>(buf_.size());
  const int i = indexAtOrBefore(t);
  if (i < 0) {
    res.too_old = true;
    return res;
  }

  // 時刻 t の状態を作る（最新より新しい観測は最新の状態に適用する）
  const MotionInput interval_input = (i + 1 < n) ? buf_[i + 1].input : buf_[i].input;
  FilterState st = buf_[i].state;
  if (i + 1 < n && t > st.t) st = est.predict(st, interval_input, t - st.t);
  if (inflation) st.P.topLeftCorner<3, 3>() += *inflation;

  const Pose2D latest_before = buf_.back().state.X.toPose();
  res = correct(est, st, est.linearize(st, m), policy, gate);
  if (!res.accepted) return res;

  // 時刻 t 以降を作り直す
  std::deque<Entry> rebuilt(buf_.begin(), buf_.begin() + i + 1);
  if (i + 1 < n && st.t > buf_[i].state.t) {
    MotionInput u = interval_input;
    u.t = st.t;
    rebuilt.push_back(Entry{st, u});
  } else {
    rebuilt.back().state = st;
  }
  FilterState prev = st;
  for (int j = i + 1; j < n; ++j) {
    const MotionInput& u = buf_[j].input;
    const double dt = u.t - prev.t;
    if (dt > 0.0) prev = est.predict(prev, u, dt);
    prev.t = u.t;
    rebuilt.push_back(Entry{prev, u});
  }
  buf_.swap(rebuilt);
  prune();

  const Pose2D latest_after = buf_.back().state.X.toPose();
  res.world_delta = Vec3(latest_after.x - latest_before.x, latest_after.y - latest_before.y,
                         wrapAngle(latest_after.yaw - latest_before.yaw));
  return res;
}

}  // namespace gll
