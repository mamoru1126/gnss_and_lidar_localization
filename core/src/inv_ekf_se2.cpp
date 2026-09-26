#include "gll/estimation/inv_ekf_se2.hpp"

namespace gll {
namespace {

Mat3 blockT(double yaw) {
  Mat3 T = Mat3::Identity();
  T.topLeftCorner<2, 2>() = SE2::rot(yaw);
  return T;
}

}  // namespace

void InvEkfSe2::transitionMatrices(const FilterState& st, const MotionInput& u, double dt, Mat5& F,
                                   Eigen::Matrix<double, 5, 3>& G) const {
  const double dphi = (u.omega - st.b) * dt;
  const Vec2 drho(st.s * u.v * dt, u.v_lat * dt);
  const SE2 D = SE2::Exp(drho, dphi);
  const Mat2 RdT = D.R().transpose();

  F.setIdentity();
  F.block<2, 2>(0, 0) = RdT;
  F.block<2, 1>(0, 2) = RdT * SE2::J() * D.t();
  F(0, 4) = cfg_.estimate_odom_scale ? u.v * dt : 0.0;
  F(2, 3) = -dt;

  // 入力ノイズ（前進速度・横すべり・ヨーレート）。右ヤコビアンは I で近似する。
  G.setZero();
  G(0, 0) = st.s * dt;
  G(1, 1) = dt;
  G(2, 2) = dt;
}

FilterState InvEkfSe2::predict(const FilterState& st, const MotionInput& u, double dt) const {
  FilterState out = st;
  if (dt <= 0.0) return out;
  Mat5 F;
  Eigen::Matrix<double, 5, 3> G;
  transitionMatrices(st, u, dt, F, G);

  const double dphi = (u.omega - st.b) * dt;
  const Vec2 drho(st.s * u.v * dt, u.v_lat * dt);
  out.X = st.X * SE2::Exp(drho, dphi);
  out.X.normalize();
  out.t = st.t + dt;

  const Eigen::Vector3d qn(cfg_.sigma_v * cfg_.sigma_v, cfg_.sigma_v_lat * cfg_.sigma_v_lat,
                           cfg_.sigma_omega * cfg_.sigma_omega);
  Mat5 Q = G * qn.asDiagonal() * G.transpose();
  Q(3, 3) += cfg_.sigma_bias_rw * cfg_.sigma_bias_rw * dt;
  if (cfg_.estimate_odom_scale) Q(4, 4) += cfg_.sigma_scale_rw * cfg_.sigma_scale_rw * dt;

  out.P = F * st.P * F.transpose() + Q;
  out.P = 0.5 * (out.P + out.P.transpose());
  return out;
}

Linearization InvEkfSe2::linearize(const FilterState& st, const Measurement& m) const {
  Linearization lin;
  const Mat2 R = st.X.R();
  const Vec2 p = st.X.t();

  if (const auto* g = std::get_if<GnssPositionMeasurement>(&m)) {
    // r = R̂ᵀ (y - p̂ - R̂ l̃),  H = [I2, J l̃, 0, 0]
    lin.r = R.transpose() * (g->y - p - R * g->lever_h);
    lin.H = Eigen::MatrixXd::Zero(2, 5);
    lin.H.block<2, 2>(0, 0).setIdentity();
    lin.H.block<2, 1>(0, 2) = SE2::J() * g->lever_h;
    lin.R = R.transpose() * g->cov_world * R + g->lever_cov_body;
  } else if (const auto* h = std::get_if<HeadingMeasurement>(&m)) {
    lin.r = Eigen::VectorXd::Constant(1, wrapAngle(h->yaw - st.X.yaw()));
    lin.H = Eigen::MatrixXd::Zero(1, 5);
    lin.H(0, 2) = 1.0;
    lin.R = Eigen::MatrixXd::Constant(1, 1, h->var);
  } else if (const auto* z = std::get_if<PoseMeasurement>(&m)) {
    // r = Log(X̂⁻¹ Z),  H = [I3, 0]
    lin.r = (st.X.inverse() * z->Z).Log();
    lin.H = Eigen::MatrixXd::Zero(3, 5);
    lin.H.block<3, 3>(0, 0).setIdentity();
    const Mat3 T = blockT(st.X.yaw());
    lin.R = z->cov_body + T.transpose() * z->anchor_cov_world * T;
  } else if (const auto* w = std::get_if<ZeroRateMeasurement>(&m)) {
    lin.r = Eigen::VectorXd::Constant(1, w->omega_mean - st.b);
    lin.H = Eigen::MatrixXd::Zero(1, 5);
    lin.H(0, 3) = 1.0;
    lin.R = Eigen::MatrixXd::Constant(1, 1, w->var);
  }
  return lin;
}

void InvEkfSe2::inject(FilterState& st, const Vec5& dx) const {
  st.X = st.X * SE2::Exp(Vec3(dx.head<3>()));
  st.X.normalize();
  st.b += dx(3);
  if (cfg_.estimate_odom_scale) st.s += dx(4);
}

Mat3 InvEkfSe2::worldCovariance(const FilterState& st) const {
  const Mat3 T = blockT(st.X.yaw());
  return T * st.P.topLeftCorner<3, 3>() * T.transpose();
}

Mat3 InvEkfSe2::errorCovarianceFromWorld(const FilterState& st, const Mat3& cov_world) const {
  const Mat3 T = blockT(st.X.yaw());
  return T.transpose() * cov_world * T;
}

}  // namespace gll
