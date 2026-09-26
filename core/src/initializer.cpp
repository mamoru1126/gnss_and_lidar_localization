#include "gll/estimation/initializer.hpp"

#include <cmath>

namespace gll {

std::optional<FilterState> Initializer::onGnss(const GnssPositionMeasurement& m, double forward_sign,
                                               double gyro_bias, bool estimate_scale) {
  if (phase_ == Phase::WAIT_FIX) {
    anchor_ = m.y;
    phase_ = Phase::WAIT_MOTION;
    return std::nullopt;
  }
  if (phase_ != Phase::WAIT_MOTION) return std::nullopt;
  const Vec2 d = m.y - anchor_;
  if (d.norm() < cfg_.heading_min_distance) return std::nullopt;

  double yaw = std::atan2(d.y(), d.x());
  if (forward_sign < 0.0) yaw = wrapAngle(yaw + kPi);

  FilterState st;
  st.t = m.t;
  const Vec2 p = m.y - SE2::rot(yaw) * m.lever_h;
  st.X = SE2::fromPose(p.x(), p.y(), yaw);
  st.b = gyro_bias;
  st.s = 1.0;
  st.P.setZero();
  const double sp2 = std::max(m.cov_world(0, 0), m.cov_world(1, 1));
  st.P(0, 0) = sp2;
  st.P(1, 1) = sp2;
  st.P(2, 2) = cfg_.init_yaw_stddev * cfg_.init_yaw_stddev;
  st.P(3, 3) = cfg_.init_bias_stddev * cfg_.init_bias_stddev;
  st.P(4, 4) = estimate_scale ? cfg_.init_scale_stddev * cfg_.init_scale_stddev : 0.0;
  phase_ = Phase::CONVERGING;
  return st;
}

FilterState Initializer::fromExternalPose(double t, const Pose2D& pose, const Mat3& cov_world,
                                          double gyro_bias, bool estimate_scale,
                                          const IStateEstimator& est) {
  FilterState st;
  st.t = t;
  st.X = SE2::fromPose(pose.x, pose.y, pose.yaw);
  st.b = gyro_bias;
  st.s = 1.0;
  st.P.setZero();
  st.P.topLeftCorner<3, 3>() = est.errorCovarianceFromWorld(st, cov_world);
  st.P(3, 3) = cfg_.init_bias_stddev * cfg_.init_bias_stddev;
  st.P(4, 4) = estimate_scale ? cfg_.init_scale_stddev * cfg_.init_scale_stddev : 0.0;
  phase_ = Phase::CONVERGING;
  return st;
}

bool Initializer::updateReady(const FilterState& st, const IStateEstimator& est) {
  if (phase_ == Phase::READY) return true;
  if (phase_ != Phase::CONVERGING) return false;
  const Mat3 cw = est.worldCovariance(st);
  const double sp = std::sqrt(std::max(cw(0, 0), cw(1, 1)));
  const double sy = std::sqrt(cw(2, 2));
  if (sp < cfg_.ready_pos_stddev && sy < cfg_.ready_yaw_stddev) phase_ = Phase::READY;
  return phase_ == Phase::READY;
}

}  // namespace gll
