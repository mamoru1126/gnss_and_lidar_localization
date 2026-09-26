// 遅延観測の再適用が、時系列順に適用した結果と一致することを確かめる（設計書 9 章）。
#include "gll/estimation/inv_ekf_se2.hpp"
#include "gll/estimation/state_history.hpp"

#include <gtest/gtest.h>

using namespace gll;

namespace {

MotionInput inputAt(double t) {
  MotionInput u;
  u.t = t;
  u.v = 1.5;
  u.omega = 0.2 * std::sin(t);
  return u;
}

FilterState initial() {
  FilterState st;
  st.t = 0.0;
  st.X = SE2::fromPose(100.0, 200.0, 0.3);
  st.P = Mat5::Identity() * 1e-2;
  st.P(4, 4) = 1e-4;
  return st;
}

}  // namespace

TEST(StateHistory, DelayedEqualsInOrder) {
  const InvEkfSe2 ekf;
  const MahalanobisGate gate(0.001);
  const double dt = 0.01;
  const double t_meas = 0.505;  // IMU の刻みの間の時刻

  GnssPositionMeasurement m;
  m.t = t_meas;
  m.y = Vec2(100.9, 200.3);
  m.cov_world = Mat2::Identity() * 4e-4;

  // A: 時系列順（t_meas まで伝播 → 更新 → 残りを伝播）
  FilterState a = initial();
  double t = 0.0;
  while (t + dt <= t_meas + 1e-12) {
    t += dt;
    a = ekf.predict(a, inputAt(t), dt);
    a.t = t;
  }
  a = ekf.predict(a, inputAt(t + dt), t_meas - a.t);  // 区間の入力は次の刻みの入力
  correct(ekf, a, ekf.linearize(a, m), GatePolicy::SKIP, gate);
  FilterState prev = a;
  double tt = t + dt;
  prev = ekf.predict(prev, inputAt(tt), tt - prev.t);
  prev.t = tt;
  while (tt + dt <= 1.0 + 1e-12) {
    tt += dt;
    prev = ekf.predict(prev, inputAt(tt), dt);
    prev.t = tt;
  }

  // B: 1 秒まで伝播してから遅延観測として適用
  StateHistory hist(2.0);
  FilterState b = initial();
  hist.reset(b);
  for (double s = dt; s <= 1.0 + 1e-12; s += dt) {
    const MotionInput u = inputAt(s);
    FilterState nb = ekf.predict(hist.latest(), u, s - hist.latest().t);
    nb.t = s;
    hist.push(nb, u);
  }
  const UpdateResult r = hist.applyDelayed(m, ekf, gate, GatePolicy::SKIP);
  ASSERT_TRUE(r.accepted);

  const FilterState& lb = hist.latest();
  EXPECT_NEAR(lb.t, prev.t, 1e-9);
  EXPECT_LT((lb.X.t() - prev.X.t()).norm(), 1e-9);
  EXPECT_NEAR(lb.X.yaw(), prev.X.yaw(), 1e-9);
  EXPECT_LT((lb.P - prev.P).norm(), 1e-9);
  EXPECT_GT(r.world_delta.head<2>().norm(), 0.0);
}

TEST(StateHistory, TooOldMeasurementIsRejected) {
  const InvEkfSe2 ekf;
  const MahalanobisGate gate(0.001);
  StateHistory hist(0.5);
  hist.reset(initial());
  for (double s = 0.01; s <= 2.0 + 1e-12; s += 0.01) {
    const MotionInput u = inputAt(s);
    FilterState nb = ekf.predict(hist.latest(), u, s - hist.latest().t);
    nb.t = s;
    hist.push(nb, u);
  }
  GnssPositionMeasurement m;
  m.t = 1.0;  // 履歴（0.5 秒）より古い
  const UpdateResult r = hist.applyDelayed(m, ekf, gate, GatePolicy::SKIP);
  EXPECT_TRUE(r.too_old);
  EXPECT_FALSE(r.accepted);
}
