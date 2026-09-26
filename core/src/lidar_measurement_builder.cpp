#include "gll/measurement/lidar_measurement_builder.hpp"

#include "gll/map/map_anchor.hpp"

#include <algorithm>
#include <cmath>

namespace gll {

LidarRejectReason LidarMeasurementBuilder::checkQuality(const RegistrationResult& r) const {
  if (!r.converged) return LidarRejectReason::NOT_CONVERGED;
  if (r.inlier_ratio < cfg_.min_inlier_ratio) return LidarRejectReason::FEW_INLIERS;
  if (r.overlap < cfg_.min_overlap) return LidarRejectReason::LOW_OVERLAP;
  return LidarRejectReason::NONE;
}

Mat3 LidarMeasurementBuilder::covarianceBody(const RegistrationResult& r) const {
  // small_gicp の H は T ← T Exp(δ)、δ = (回転 3, 並進 3) に対するもの。点は相関しているので、
  // 点数に比例して情報が増えないよう N_inlier を掛け戻し、cov_scale で実際の精度に合わせる（設計書 6.2 節）。
  const Mat6 Hr = r.H + 1e-6 * Mat6::Identity();
  const Mat6 cov6 = cfg_.cov_scale * static_cast<double>(std::max<std::size_t>(r.num_inliers, 1)) * Hr.inverse();
  // SE(2) の誤差 (ρx, ρy, φ) = 並進 x, 並進 y, 回転 z（roll / pitch が小さいとして射影する）
  const int idx[3] = {3, 4, 2};
  Mat3 c;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) c(i, j) = cov6(idx[i], idx[j]);
  c = 0.5 * (c + c.transpose());
  c(0, 0) += cfg_.min_stddev_xy * cfg_.min_stddev_xy;
  c(1, 1) += cfg_.min_stddev_xy * cfg_.min_stddev_xy;
  c(2, 2) += cfg_.min_stddev_yaw * cfg_.min_stddev_yaw;
  return c;
}

LidarMeasurementResult LidarMeasurementBuilder::build(double t, const RegistrationResult& r, const MatchTarget& target,
                                                      const Eigen::Isometry3d& init) const {
  LidarMeasurementResult out;
  out.reason = checkQuality(r);
  if (out.reason != LidarRejectReason::NONE) return out;
  const Eigen::Isometry3d d = init.inverse() * r.T_map_base;
  if (d.translation().head<2>().norm() > cfg_.max_jump_xy || std::abs(yawOf(d.linear())) > cfg_.max_jump_yaw) {
    out.reason = LidarRejectReason::JUMP;
    return out;
  }
  const Eigen::Isometry3d T_utm = target.anchor.poseMapToUtm(r.T_map_base);
  PoseMeasurement m;
  m.t = t;
  m.Z = SE2::fromPose(T_utm.translation().x(), T_utm.translation().y(), yawOf(T_utm.linear()));
  m.cov_body = covarianceBody(r);
  m.anchor_cov_world = target.anchor.covarianceWorld();
  m.map_group = target.group;
  out.pose = m;
  return out;
}

}  // namespace gll
