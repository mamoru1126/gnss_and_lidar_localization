#include "gll/estimation/es_ekf_2d.hpp"

#include <cmath>

namespace gll {

FilterState EsEkf2D::predict(const FilterState& st, const MotionInput& u, double dt) const {
  FilterState out = st;
  if (dt <= 0.0) return out;
  const double th = st.X.yaw();
  const double w = u.omega - st.b;
  const double tb = th + 0.5 * w * dt;
  const double c = std::cos(tb), sn = std::sin(tb);
  const double s = st.s, v = u.v, vl = u.v_lat;

  // 名目状態（中点法）
  const Vec2 p = st.X.t() + dt * (s * v * Vec2(c, sn) + vl * Vec2(-sn, c));
  out.X = SE2::fromPose(p.x(), p.y(), wrapAngle(th + w * dt));
  out.t = st.t + dt;

  Mat5 F = Mat5::Identity();
  F(0, 2) = dt * (-s * v * sn - vl * c);
  F(1, 2) = dt * (s * v * c - vl * sn);
  F(0, 3) = 0.5 * dt * dt * (s * v * sn + vl * c);
  F(1, 3) = 0.5 * dt * dt * (-s * v * c + vl * sn);
  if (cfg_.estimate_odom_scale) {
    F(0, 4) = v * dt * c;
    F(1, 4) = v * dt * sn;
  }
  F(2, 3) = -dt;

  Eigen::Matrix<double, 5, 3> G = Eigen::Matrix<double, 5, 3>::Zero();
  G(0, 0) = s * dt * c;
  G(1, 0) = s * dt * sn;
  G(0, 1) = -dt * sn;
  G(1, 1) = dt * c;
  G(0, 2) = -0.5 * s * v * dt * dt * sn;
  G(1, 2) = 0.5 * s * v * dt * dt * c;
  G(2, 2) = dt;
  const Eigen::Vector3d qn(cfg_.sigma_v * cfg_.sigma_v, cfg_.sigma_v_lat * cfg_.sigma_v_lat,
                           cfg_.sigma_omega * cfg_.sigma_omega);
  Mat5 Q = G * qn.asDiagonal() * G.transpose();
  Q(3, 3) += cfg_.sigma_bias_rw * cfg_.sigma_bias_rw * dt;
  if (cfg_.estimate_odom_scale) Q(4, 4) += cfg_.sigma_scale_rw * cfg_.sigma_scale_rw * dt;

  out.P = F * st.P * F.transpose() + Q;
  out.P = 0.5 * (out.P + out.P.transpose());
  return out;
}

Linearization EsEkf2D::linearize(const FilterState& st, const Measurement& m) const {
  Linearization lin;
  const Mat2 R = st.X.R();
  const Vec2 p = st.X.t();
  if (const auto* g = std::get_if<GnssPositionMeasurement>(&m)) {
    lin.r = g->y - p - R * g->lever_h;
    lin.H = Eigen::MatrixXd::Zero(2, 5);
    lin.H.block<2, 2>(0, 0).setIdentity();
    lin.H.block<2, 1>(0, 2) = SE2::J() * R * g->lever_h;
    lin.R = g->cov_world + R * g->lever_cov_body * R.transpose();
  } else if (const auto* h = std::get_if<HeadingMeasurement>(&m)) {
    lin.r = Eigen::VectorXd::Constant(1, wrapAngle(h->yaw - st.X.yaw()));
    lin.H = Eigen::MatrixXd::Zero(1, 5);
    lin.H(0, 2) = 1.0;
    lin.R = Eigen::MatrixXd::Constant(1, 1, h->var);
  } else if (const auto* z = std::get_if<PoseMeasurement>(&m)) {
    lin.r = Eigen::VectorXd(3);
    lin.r.head<2>() = z->Z.t() - p;
    lin.r(2) = wrapAngle(z->Z.yaw() - st.X.yaw());
    lin.H = Eigen::MatrixXd::Zero(3, 5);
    lin.H.block<3, 3>(0, 0).setIdentity();
    Mat3 T = Mat3::Identity();
    T.topLeftCorner<2, 2>() = R;
    lin.R = T * z->cov_body * T.transpose() + z->anchor_cov_world;
  } else if (const auto* w = std::get_if<ZeroRateMeasurement>(&m)) {
    lin.r = Eigen::VectorXd::Constant(1, w->omega_mean - st.b);
    lin.H = Eigen::MatrixXd::Zero(1, 5);
    lin.H(0, 3) = 1.0;
    lin.R = Eigen::MatrixXd::Constant(1, 1, w->var);
  }
  return lin;
}

void EsEkf2D::inject(FilterState& st, const Vec5& dx) const {
  const Vec2 p = st.X.t() + dx.head<2>();
  st.X = SE2::fromPose(p.x(), p.y(), wrapAngle(st.X.yaw() + dx(2)));
  st.b += dx(3);
  if (cfg_.estimate_odom_scale) st.s += dx(4);
}

}  // namespace gll
