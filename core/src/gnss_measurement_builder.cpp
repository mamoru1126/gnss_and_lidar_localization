#include "gll/measurement/gnss_measurement_builder.hpp"

#include <algorithm>
#include <cmath>

namespace gll {

GnssMeasurementBuilder::GnssMeasurementBuilder(const GnssConfig& cfg)
    : cfg_(cfg), utm_(cfg.utm_zone, cfg.utm_north) {}

GnssFixType GnssMeasurementBuilder::classify(const GnssSample& s) const {
  if (s.raw_status == cfg_.rtk_fix_status) return GnssFixType::RTK_FIX;
  switch (s.raw_status) {
    case 0: return GnssFixType::SINGLE;
    case 1: return GnssFixType::DGPS;
    case 2: return GnssFixType::RTK_FLOAT;  // rtk_fix_status を 2 以外にした場合
    default: return GnssFixType::NONE;
  }
}

Vec2 GnssMeasurementBuilder::projectLeverArm(double roll, double pitch) const {
  const Mat3 R = (Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX()))
                     .toRotationMatrix();
  return (R * cfg_.lever_arm).head<2>();
}

GnssPositionResult GnssMeasurementBuilder::buildPosition(const GnssSample& s,
                                                        const AttitudeEstimator& att) {
  GnssPositionResult res;
  res.fix = classify(s);
  // メッセージが途切れていた（受信できなかった）場合も、FIX が一度切れたものとして安定待ちをやり直す
  if (last_sample_t_ >= 0.0 && s.t - last_sample_t_ > cfg_.settle_reset_gap) fix_since_ = -1.0;
  last_sample_t_ = s.t;
  if (res.fix != GnssFixType::RTK_FIX) {
    fix_since_ = -1.0;
    res.reason = GnssRejectReason::NOT_RTK_FIX;
    return res;
  }
  if (fix_since_ < 0.0) fix_since_ = s.t;

  Mat2 cov;
  if (s.cov_known) {
    cov = s.cov_enu.topLeftCorner<2, 2>();
    const double sigma_h = std::sqrt(std::max(cov(0, 0), cov(1, 1)));
    if (sigma_h > cfg_.max_stddev) {
      res.reason = GnssRejectReason::STDDEV_TOO_LARGE;
      return res;
    }
  } else {
    if (!cfg_.accept_unknown_covariance) {
      res.reason = GnssRejectReason::COVARIANCE_UNKNOWN;
      return res;
    }
    cov = cfg_.default_stddev * cfg_.default_stddev * Mat2::Identity();
  }
  if (s.t - fix_since_ < cfg_.fix_settle_time) {
    res.reason = GnssRejectReason::SETTLING;
    return res;
  }

  const UtmPoint u = utm_.forward(s.lat, s.lon);
  last_convergence_ = u.convergence;
  const double min_var = cfg_.min_stddev * cfg_.min_stddev;
  cov(0, 0) = std::max(cov(0, 0), min_var);
  cov(1, 1) = std::max(cov(1, 1), min_var);

  const Attitude a = att.attitudeAt(s.t);
  GnssPositionMeasurement m;
  m.t = s.t;
  m.y = Vec2(u.easting, u.northing);
  m.cov_world = cov;
  m.lever_h = projectLeverArm(a.roll, a.pitch);
  const double lz = cfg_.lever_arm.z();
  const double sa2 = cfg_.attitude_stddev * cfg_.attitude_stddev;
  m.lever_cov_body = (lz * lz * sa2) * Mat2::Identity();  // diag(σ_pitch², σ_roll²) で同じ値
  res.position = m;
  return res;
}

std::optional<HeadingMeasurement> GnssMeasurementBuilder::buildHeading(const GnssVelocitySample& s,
                                                                      double yaw_rate,
                                                                      bool rtk_fix_active) const {
  if (!cfg_.use_velocity || !rtk_fix_active) return std::nullopt;
  const double speed = s.vel_en.norm();
  if (speed < cfg_.cog_min_speed || std::abs(yaw_rate) > cfg_.cog_max_yaw_rate) return std::nullopt;
  HeadingMeasurement h;
  h.t = s.t;
  // ENU の角度をグリッドの角度に変換する（γ = 子午線収差）
  h.yaw = wrapAngle(std::atan2(s.vel_en.y(), s.vel_en.x()) + last_convergence_);
  const double sigma_v = std::sqrt(std::max(s.cov(0, 0), s.cov(1, 1)));
  const double sigma_psi = std::max(sigma_v / speed, deg2rad(0.5));
  h.var = sigma_psi * sigma_psi;
  return h;
}

}  // namespace gll
