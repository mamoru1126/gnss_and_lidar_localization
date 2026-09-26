// 遷移行列・観測行列を数値微分と照合する（アルゴリズム説明書 14 章）。
#include "gll/estimation/es_ekf_2d.hpp"
#include "gll/estimation/inv_ekf_se2.hpp"

#include <gtest/gtest.h>

using namespace gll;

namespace {

FilterState makeState() {
  FilterState st;
  st.t = 10.0;
  // 数値微分の桁落ちを避けるため原点付近に置く（式は平行移動に対して不変）
  st.X = SE2::fromPose(12.3, -45.6, 0.8);
  st.b = 0.003;
  st.s = 1.02;
  st.P = Mat5::Identity() * 1e-3;
  return st;
}

// Invariant EKF: 真値 = 推定 ⊕ δx
FilterState perturbInv(const FilterState& st, const Vec5& dx) {
  FilterState t = st;
  t.X = st.X * SE2::Exp(Vec3(dx.head<3>()));
  t.b += dx(3);
  t.s += dx(4);
  return t;
}
Vec5 errorInv(const FilterState& est, const FilterState& truth) {
  Vec5 e;
  e.head<3>() = (est.X.inverse() * truth.X).Log();
  e(3) = truth.b - est.b;
  e(4) = truth.s - est.s;
  return e;
}

// ESEKF: 真値 = 推定 + δx（位置は世界座標系）
FilterState perturbEs(const FilterState& st, const Vec5& dx) {
  FilterState t = st;
  const Vec2 p = st.X.t() + dx.head<2>();
  t.X = SE2::fromPose(p.x(), p.y(), st.X.yaw() + dx(2));
  t.b += dx(3);
  t.s += dx(4);
  return t;
}
Vec5 errorEs(const FilterState& est, const FilterState& truth) {
  Vec5 e;
  e.head<2>() = truth.X.t() - est.X.t();
  e(2) = wrapAngle(truth.X.yaw() - est.X.yaw());
  e(3) = truth.b - est.b;
  e(4) = truth.s - est.s;
  return e;
}

}  // namespace

TEST(InvariantEKF, TransitionMatrixMatchesNumericJacobian) {
  const InvEkfSe2 ekf;
  const FilterState st = makeState();
  MotionInput u;
  u.v = 1.5;
  u.v_lat = 0.05;
  u.omega = 0.25;
  const double dt = 0.01;
  Mat5 F;
  Eigen::Matrix<double, 5, 3> G;
  ekf.transitionMatrices(st, u, dt, F, G);

  const FilterState nominal = ekf.predict(st, u, dt);
  const double h = 1e-5;
  for (int j = 0; j < 5; ++j) {
    Vec5 dx = Vec5::Zero();
    dx(j) = h;
    const Vec5 col = (errorInv(nominal, ekf.predict(perturbInv(st, dx), u, dt)) -
                      errorInv(nominal, ekf.predict(perturbInv(st, -dx), u, dt))) /
                     (2.0 * h);
    // バイアスとスケールの列は右ヤコビアンを I で近似しているので O(dt²) の差を許す
    const double tol = (j >= 3) ? 2e-4 : 1e-6;
    EXPECT_LT((col - F.col(j)).norm(), tol) << "column " << j << "\nnumeric " << col.transpose()
                                            << "\nanalytic " << F.col(j).transpose();
  }
}

TEST(InvariantEKF, TransitionMatrixDoesNotDependOnPose) {
  // Invariant EKF の要点: F は推定姿勢に依存しない
  const InvEkfSe2 ekf;
  FilterState a = makeState(), b = makeState();
  b.X = SE2::fromPose(-100.0, 20.0, -2.5);
  MotionInput u;
  u.v = 1.2;
  u.omega = -0.3;
  Mat5 Fa, Fb;
  Eigen::Matrix<double, 5, 3> Ga, Gb;
  ekf.transitionMatrices(a, u, 0.01, Fa, Ga);
  ekf.transitionMatrices(b, u, 0.01, Fb, Gb);
  EXPECT_LT((Fa - Fb).norm(), 1e-15);
}

TEST(InvariantEKF, GnssObservationMatrixMatchesNumeric) {
  const InvEkfSe2 ekf;
  const FilterState st = makeState();
  GnssPositionMeasurement m;
  m.lever_h = Vec2(0.3, 0.2);
  m.y = st.X.act(m.lever_h);  // 誤差 0 のときの観測
  const Linearization lin0 = ekf.linearize(st, m);
  EXPECT_LT(lin0.r.norm(), 1e-9);
  const double h = 1e-5;
  for (int j = 0; j < 5; ++j) {
    Vec5 dx = Vec5::Zero();
    dx(j) = h;
    GnssPositionMeasurement mp = m, mm = m;
    mp.y = perturbInv(st, dx).X.act(m.lever_h);
    mm.y = perturbInv(st, -dx).X.act(m.lever_h);
    const Eigen::VectorXd col = (ekf.linearize(st, mp).r - ekf.linearize(st, mm).r) / (2.0 * h);
    EXPECT_LT((col - lin0.H.col(j)).norm(), 1e-5) << "column " << j;
  }
}

TEST(InvariantEKF, GnssObservationMatrixIsConstant) {
  const InvEkfSe2 ekf;
  FilterState a = makeState(), b = makeState();
  b.X = SE2::fromPose(12.0, -7.0, 2.9);
  GnssPositionMeasurement m;
  m.lever_h = Vec2(0.3, 0.2);
  EXPECT_LT((ekf.linearize(a, m).H - ekf.linearize(b, m).H).norm(), 1e-15);
}

TEST(InvariantEKF, PoseObservationResidualIsExactError) {
  const InvEkfSe2 ekf;
  const FilterState st = makeState();
  PoseMeasurement z;
  const Vec3 xi(0.05, -0.02, 0.01);
  z.Z = st.X * SE2::Exp(xi);
  EXPECT_LT((ekf.linearize(st, z).r - xi).norm(), 1e-10);
}

TEST(ESEKF, TransitionMatrixMatchesNumericJacobian) {
  const EsEkf2D ekf;
  const FilterState st = makeState();
  MotionInput u;
  u.v = 1.5;
  u.v_lat = 0.05;
  u.omega = 0.25;
  const double dt = 0.01;
  const FilterState nominal = ekf.predict(st, u, dt);
  // F を数値的に求めて、predict の P 伝播と比べる（Q を除いた部分）
  FilterState probe = st;
  probe.P.setZero();
  Mat5 Fnum;
  const double h = 1e-5;
  for (int j = 0; j < 5; ++j) {
    Vec5 dx = Vec5::Zero();
    dx(j) = h;
    Fnum.col(j) = (errorEs(nominal, ekf.predict(perturbEs(st, dx), u, dt)) -
                   errorEs(nominal, ekf.predict(perturbEs(st, -dx), u, dt))) /
                  (2.0 * h);
  }
  // 各列を単位ベクトルの P で伝播して解析的な F を取り出す
  for (int j = 0; j < 5; ++j) {
    FilterState pj = st;
    pj.P.setZero();
    pj.P(j, j) = 1.0;
    EstimatorConfig c;
    c.sigma_v = c.sigma_v_lat = c.sigma_omega = c.sigma_bias_rw = c.sigma_scale_rw = 0.0;
    const EsEkf2D noiseless(c);
    const Mat5 Pj = noiseless.predict(pj, u, dt).P;  // = F e_j e_jᵀ Fᵀ
    const Vec5 Fj = Pj.col(j) / std::sqrt(std::max(Pj(j, j), 1e-30));
    EXPECT_LT((Fj - Fnum.col(j)).norm(), 1e-5) << "column " << j;
  }
}

TEST(ESEKF, GnssObservationMatrixMatchesNumeric) {
  const EsEkf2D ekf;
  const FilterState st = makeState();
  GnssPositionMeasurement m;
  m.lever_h = Vec2(0.3, 0.2);
  m.y = st.X.act(m.lever_h);
  const Linearization lin0 = ekf.linearize(st, m);
  const double h = 1e-5;
  for (int j = 0; j < 5; ++j) {
    Vec5 dx = Vec5::Zero();
    dx(j) = h;
    GnssPositionMeasurement mp = m, mm = m;
    mp.y = perturbEs(st, dx).X.act(m.lever_h);
    mm.y = perturbEs(st, -dx).X.act(m.lever_h);
    const Eigen::VectorXd col = (ekf.linearize(st, mp).r - ekf.linearize(st, mm).r) / (2.0 * h);
    EXPECT_LT((col - lin0.H.col(j)).norm(), 1e-5) << "column " << j;
  }
}

TEST(Correct, UpdateReducesCovarianceAndMovesTowardMeasurement) {
  const InvEkfSe2 ekf;
  FilterState st = makeState();
  st.P = Mat5::Identity() * 0.01;
  GnssPositionMeasurement m;
  m.lever_h = Vec2(0.0, 0.0);
  m.y = st.X.t() + Vec2(0.1, 0.0);
  m.cov_world = Mat2::Identity() * 1e-4;
  const MahalanobisGate gate(0.001);
  const double tr0 = st.P.trace();
  const UpdateResult r = correct(ekf, st, ekf.linearize(st, m), GatePolicy::REJECT_ON_FAIL, gate);
  EXPECT_TRUE(r.accepted);
  EXPECT_LT(st.P.trace(), tr0);
  EXPECT_GT(r.world_delta.x(), 0.09);
}

TEST(Correct, DeferredMeasurementDoesNotChangeState) {
  const InvEkfSe2 ekf;
  FilterState st = makeState();
  st.P = Mat5::Identity() * 1e-6;
  GnssPositionMeasurement m;
  m.y = st.X.t() + Vec2(5.0, 0.0);  // 大きく外れた観測
  m.cov_world = Mat2::Identity() * 1e-4;
  const MahalanobisGate gate(0.001);
  const FilterState before = st;
  const UpdateResult r = correct(ekf, st, ekf.linearize(st, m), GatePolicy::DEFER_TO_RECOVERY, gate);
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.gate_passed);
  EXPECT_EQ(st.X.t(), before.X.t());
}

TEST(InvariantEKF, WorksAtUtmScaleCoordinates) {
  // UTM の大きな座標（約 400 万 m）でも、残差は桁落ちせずに計算できる
  const InvEkfSe2 ekf;
  FilterState st = makeState();
  st.X = SE2::fromPose(450123.4, 3950456.7, 0.8);
  GnssPositionMeasurement m;
  m.lever_h = Vec2(0.3, 0.2);
  m.y = st.X.act(m.lever_h) + Vec2(0.01, -0.02);
  const Eigen::VectorXd r = ekf.linearize(st, m).r;
  EXPECT_NEAR((st.X.R() * r).x(), 0.01, 1e-8);
  EXPECT_NEAR((st.X.R() * r).y(), -0.02, 1e-8);
}
