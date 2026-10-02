// RecoveryManager の LiDAR 部分（再アンカー・再位置推定・LOST）の単体テスト（設計書 3.13.3〜3.13.5 節）。
#include "gll/estimation/inv_ekf_se2.hpp"
#include "gll/estimation/recovery_manager.hpp"
#include "gll/estimation/status_monitor.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace gll;

namespace {

/// 東へ 1 m/s で 3 秒走った履歴（100 Hz）。
StateHistory straightHistory(const IStateEstimator& est) {
  StateHistory h(5.0);
  FilterState st;
  st.t = 0.0;
  st.X = SE2::fromPose(1000.0, 2000.0, 0.0);
  st.P = Mat5::Identity() * 1e-4;
  h.reset(st);
  for (int k = 1; k <= 300; ++k) {
    MotionInput u;
    u.t = k * 0.01;
    u.v = 1.0;
    st = est.predict(st, u, 0.01);
    st.t = u.t;
    h.push(st, u);
  }
  return h;
}

/// 時刻 t の推定値から、世界座標系で (dx, dy, dyaw) だけずれた LiDAR の観測。
PoseMeasurement offsetPose(const StateHistory& h, const IStateEstimator& est, double t, double dx, double dy,
                           double dyaw) {
  const FilterState s = *h.stateAt(t, est);
  PoseMeasurement m;
  m.t = t;
  m.Z = SE2::fromPose(s.X.t().x() + dx, s.X.t().y() + dy, s.X.yaw() + dyaw);
  m.cov_body = Mat3::Identity() * 1e-3;
  m.map_group = "g";
  return m;
}

UpdateResult rejected() {
  UpdateResult r;
  r.residual = Eigen::VectorXd::Constant(3, 0.5);
  return r;
}

}  // namespace

TEST(RecoveryLidar, ReanchorsWhenRejectedPosesAgree) {
  const InvEkfSe2 est;
  const StateHistory h = straightHistory(est);
  RecoveryManager rm;
  RecoveryAction act;
  for (int i = 0; i < 5; ++i) {
    act = rm.onRejected(offsetPose(h, est, 2.0 + 0.1 * i, 0.8, -0.3, 0.02), rejected(), h, est, false);
    if (i < 4) EXPECT_EQ(act.kind, RecoveryAction::Kind::NONE);
  }
  EXPECT_EQ(act.kind, RecoveryAction::Kind::REANCHOR);
  EXPECT_EQ(rm.state(), RecoveryState::REANCHOR);
  ASSERT_TRUE(act.measurement.has_value());
  EXPECT_GT(act.inflation(0, 0), 0.25);
  rm.onReanchored();
  EXPECT_EQ(rm.state(), RecoveryState::TRACKING);
}

TEST(RecoveryLidar, InconsistentRejectionsRequestRelocalization) {
  const InvEkfSe2 est;
  const StateHistory h = straightHistory(est);
  RelocalizeConfig rc;
  RecoveryManager rm(RecoveryConfig(), rc);
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  RecoveryAction act;
  int n = 0;
  for (; n < 30 && act.kind == RecoveryAction::Kind::NONE; ++n)
    act = rm.onRejected(offsetPose(h, est, 1.0 + 0.05 * n, u(rng), u(rng), 0.1 * u(rng)), rejected(), h, est, false,
                        40.0);
  EXPECT_EQ(n, rc.after_rejects);
  ASSERT_EQ(act.kind, RecoveryAction::Kind::RELOCALIZE);
  ASSERT_TRUE(act.relocalize.has_value());
  // 位置の観測なしで 40 m 走っているので、半径は 0.05 × 40 = 2 m まで広げる
  EXPECT_NEAR(act.relocalize->radius, 2.0, 1e-9);
  EXPECT_EQ(rm.state(), RecoveryState::RELOCALIZE);

  // GNSS FIX 中は、LiDAR が落ち続けても再位置推定しない
  RecoveryManager rm2(RecoveryConfig(), rc);
  for (int i = 0; i < 40; ++i)
    EXPECT_EQ(rm2.onLidarFailure(1.0 + 0.05 * i, h, est, true).kind, RecoveryAction::Kind::NONE);
}

TEST(RecoveryLidar, RepeatedRelocalizationFailuresMeanLost) {
  const InvEkfSe2 est;
  const StateHistory h = straightHistory(est);
  RelocalizeConfig rc;
  RecoveryManager rm(RecoveryConfig(), rc);
  for (int k = 0; k < rc.max_attempts; ++k) {
    EXPECT_NE(rm.state(), RecoveryState::LOST);
    rm.onRelocalizeResult(10.0 + k, false);
  }
  EXPECT_EQ(rm.state(), RecoveryState::LOST);
  // 共分散が小さくても、再位置推定の失敗による LOST は解除しない
  rm.updateLost(0.05, 1.0);
  EXPECT_EQ(rm.state(), RecoveryState::LOST);
  // LOST の間は、lost_retry_interval ごとに広い範囲で再位置推定を試す
  EXPECT_FALSE(rm.relocalizeDue(12.0 + rc.lost_retry_interval - 0.1));
  EXPECT_TRUE(rm.relocalizeDue(12.0 + rc.lost_retry_interval));
  const RelocalizeRequest wide = rm.makeRelocalizeRequest(20.0, h.latest(), est, true);
  EXPECT_DOUBLE_EQ(wide.radius, rc.max_radius);
  EXPECT_DOUBLE_EQ(wide.yaw_range, rc.max_yaw);
  // 位置の観測を採用できたら解除する
  rm.onAccepted(offsetPose(h, est, 2.9, 0, 0, 0));
  EXPECT_EQ(rm.state(), RecoveryState::TRACKING);
}

TEST(RecoveryLidar, RelocalizationFailuresDoNotMeanLostWhenDisabled) {
  // relocalize.lost_on_failure: false なら、再位置推定が何回失敗しても LOST にしない。
  // 照合がまた after_rejects 回続けて捨てられたら、再位置推定をやり直す
  const InvEkfSe2 est;
  const StateHistory h = straightHistory(est);
  RelocalizeConfig rc;
  rc.lost_on_failure = false;
  RecoveryManager rm(RecoveryConfig(), rc);
  for (int k = 0; k < rc.max_attempts + 2; ++k) {
    for (int i = 0; i < rc.after_rejects - 1; ++i)
      EXPECT_EQ(rm.onLidarFailure(2.9, h, est, false, 10.0).kind, RecoveryAction::Kind::NONE);
    const RecoveryAction act = rm.onLidarFailure(2.9, h, est, false, 10.0);
    ASSERT_EQ(act.kind, RecoveryAction::Kind::RELOCALIZE) << "attempt " << k;
    EXPECT_EQ(rm.state(), RecoveryState::RELOCALIZE);
    rm.onRelocalizeResult(10.0 + k, false);
    EXPECT_NE(rm.state(), RecoveryState::LOST);
  }
  // 位置の不確かさが大きくなれば LOST（共分散が小さくなれば解除）
  rm.updateLost(true);
  EXPECT_EQ(rm.state(), RecoveryState::LOST);
  rm.updateLost(false);
  EXPECT_NE(rm.state(), RecoveryState::LOST);
}

TEST(StatusMonitor, LateralStddevMeansLost) {
  MonitorConfig mc;
  mc.lost_stddev = 1.0;
  mc.lost_stddev_lateral = 0.3;
  const StatusMonitor sm(mc);
  // 東向き（yaw 0）で、前後（x）に 0.8 m、左右（y）に 0.2 m
  Mat3 cov = Mat3::Zero();
  cov(0, 0) = 0.8 * 0.8;
  cov(1, 1) = 0.2 * 0.2;
  cov(2, 2) = 1e-4;
  EXPECT_FALSE(sm.lostByCovariance(cov, 0.0));
  EXPECT_NE(sm.evaluate(0.0, true, cov, false, RecoveryState::TRACKING, 0.0), LocalizationStatus::LOST);
  // 北向き（yaw 90°）なら、同じ共分散でも左右が 0.8 m → LOST
  EXPECT_TRUE(sm.lostByCovariance(cov, kPi / 2.0));
  EXPECT_EQ(sm.evaluate(0.0, true, cov, false, RecoveryState::TRACKING, kPi / 2.0), LocalizationStatus::LOST);
  // 横を見ない設定（既定）なら LOST ではない
  const StatusMonitor sm0{MonitorConfig()};
  EXPECT_FALSE(sm0.lostByCovariance(cov, kPi / 2.0));
}
