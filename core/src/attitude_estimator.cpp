#include "gll/measurement/attitude_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace gll {
namespace {

Eigen::Quaterniond expQuat(const Vec3& w) {
  const double a = w.norm();
  if (a < 1e-12) return Eigen::Quaterniond(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, w / a));
}

}  // namespace

bool AttitudeEstimator::addStaticSample(const ImuSample& s) {
  if (initialized_) return true;
  if (static_t0_ < 0.0) static_t0_ = s.t;
  acc_sum_ += s.acc;
  gyro_sum_ += s.gyro;
  ++static_n_;
  if (s.t - static_t0_ < cfg_.static_init_time) return false;

  const Vec3 a = acc_sum_ / static_n_;
  gyro_bias_ = gyro_sum_ / static_n_;
  const double roll = std::atan2(a.y(), a.z());
  const double pitch = std::atan2(-a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));
  q_ = Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX());
  last_t_ = s.t;
  initialized_ = true;
  history_.push_back(Attitude{s.t, roll, pitch});
  return true;
}

void AttitudeEstimator::update(const ImuSample& s, double v, double v_dot) {
  if (!initialized_) return;
  const double dt = (last_t_ < 0.0) ? 0.0 : s.t - last_t_;
  last_t_ = s.t;
  if (dt <= 0.0 || dt > 1.0) return;

  const Vec3 omega = s.gyro - gyro_bias_;
  // 運動加速度（加減速と向心加速度）を差し引いて重力方向を求める（設計書 3.9 節）
  const Vec3 a_lin(v_dot, v * omega.z(), 0.0);
  const Vec3 g_meas = s.acc - a_lin;
  Vec3 corr = Vec3::Zero();
  const double gn = g_meas.norm();
  if (gn > 1e-6) {
    const double weight = std::clamp(1.0 - std::abs(gn - kGravity) / cfg_.accel_tolerance, 0.0, 1.0);
    const Vec3 v_hat = q_.conjugate() * Vec3::UnitZ();  // 推定した鉛直方向（機体座標系）
    const Vec3 e = (g_meas / gn).cross(v_hat);
    integral_ += cfg_.ki * weight * e * dt;
    corr = cfg_.kp * weight * e + integral_;
  }
  q_ = (q_ * expQuat((omega + corr) * dt)).normalized();

  // yaw 成分を捨てて roll / pitch だけを保つ
  const double r = roll(), p = pitch();
  q_ = Eigen::AngleAxisd(p, Vec3::UnitY()) * Eigen::AngleAxisd(r, Vec3::UnitX());

  history_.push_back(Attitude{s.t, r, p});
  while (history_.size() > 2 && history_.front().t < s.t - cfg_.history_length) history_.pop_front();
}

double AttitudeEstimator::roll() const {
  const Mat3 R = q_.toRotationMatrix();
  return std::atan2(R(2, 1), R(2, 2));
}

double AttitudeEstimator::pitch() const {
  const Mat3 R = q_.toRotationMatrix();
  return std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
}

Mat3 AttitudeEstimator::tiltRotation() const {
  return (Eigen::AngleAxisd(pitch(), Vec3::UnitY()) * Eigen::AngleAxisd(roll(), Vec3::UnitX()))
      .toRotationMatrix();
}

double AttitudeEstimator::verticalRate(const Vec3& gyro) const {
  const Vec3 w(gyro.x() - gyro_bias_.x(), gyro.y() - gyro_bias_.y(), gyro.z());
  return (tiltRotation() * w).z();
}

Attitude AttitudeEstimator::attitudeAt(double t) const {
  if (history_.empty()) return Attitude{t, roll(), pitch()};
  const Attitude* best = &history_.back();
  double best_dt = std::abs(best->t - t);
  for (const auto& a : history_) {
    const double d = std::abs(a.t - t);
    if (d < best_dt) {
      best = &a;
      best_dt = d;
    }
  }
  return *best;
}

}  // namespace gll
