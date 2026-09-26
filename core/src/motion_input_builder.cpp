#include "gll/measurement/motion_input_builder.hpp"

#include <cmath>

namespace gll {

void MotionInputBuilder::addOdom(const OdomSample& o) {
  if (!odom_.empty() && o.t <= odom_.back().t) return;  // 時刻が戻ったサンプルは捨てる
  odom_.push_back(o);
  while (odom_.size() > 3) odom_.pop_front();
  latest_ = o;
}

std::optional<MotionInputBuilder::Velocity> MotionInputBuilder::velocityAt(double t) const {
  if (odom_.empty()) return std::nullopt;
  Velocity vel;
  const OdomSample& b = odom_.back();
  if (odom_.size() >= 2) {
    const OdomSample& a = odom_[odom_.size() - 2];
    const double span = b.t - a.t;
    if (span > 1e-6) vel.v_dot = (b.v - a.v) / span;
    if (t <= b.t && t >= a.t && span > 1e-6) {
      const double w = (t - a.t) / span;
      vel.v = a.v + w * (b.v - a.v);
      vel.v_lat = a.v_lat + w * (b.v_lat - a.v_lat);
      return vel;
    }
  }
  vel.v = b.v;
  vel.v_lat = b.v_lat;
  vel.stale = (t - b.t) > cfg_.odom_hold_max;
  return vel;
}

std::optional<MotionInput> MotionInputBuilder::build(const ImuSample& imu,
                                                     const AttitudeEstimator& att) const {
  const auto vel = velocityAt(imu.t);
  if (!vel) return std::nullopt;
  MotionInput u;
  u.t = imu.t;
  u.v = vel->v * std::cos(att.pitch());  // 坂道では水平成分に射影する
  u.v_lat = vel->v_lat;
  u.omega = att.verticalRate(imu.gyro);
  return u;
}

std::optional<MotionInput> MotionInputBuilder::buildFromOdom(const OdomSample& o) const {
  if (!o.yaw_rate) return std::nullopt;
  MotionInput u;
  u.t = o.t;
  u.v = o.v;
  u.v_lat = o.v_lat;
  u.omega = *o.yaw_rate;
  return u;
}

}  // namespace gll
