#include "gll/estimation/source_arbiter.hpp"

#include <algorithm>
#include <cmath>

namespace gll {

Vec3 MismatchStats::stddev() const {
  if (count < 2) return Vec3::Zero();
  return (m2_world / static_cast<double>(count - 1)).cwiseSqrt();
}

void SourceArbiter::onGnssAccepted(double t) { last_gnss_fix_ = std::max(last_gnss_fix_, t); }

GatePolicy SourceArbiter::gatePolicy(const Measurement& m) const {
  switch (measurementKind(m)) {
    case MeasurementKind::GNSS_POSITION: return GatePolicy::DEFER_TO_RECOVERY;  // FIX は捨てない
    case MeasurementKind::HEADING: return GatePolicy::REJECT_ON_FAIL;
    case MeasurementKind::ZERO_RATE: return GatePolicy::REJECT_ON_FAIL;
    case MeasurementKind::POSE: return GatePolicy::DEFER_TO_RECOVERY;  // 落ちたら LiDAR の再アンカーの候補
  }
  return GatePolicy::REJECT_ON_FAIL;
}

LidarDecision SourceArbiter::classifyLidar(PoseMeasurement& m, const FilterState& st) {
  if (!isGnssFixActive(m.t)) return LidarDecision::FUSE_PRIMARY;
  const Vec3 e = (st.X.inverse() * m.Z).Log();  // 機体座標系の食い違い
  Mat3 T = Mat3::Identity();
  T.topLeftCorner<2, 2>() = st.X.R();
  const Vec3 ew = T * e;

  MismatchStats& ms = mismatch_[m.map_group];
  ++ms.count;
  const Vec3 d = ew - ms.mean_world;
  ms.mean_world += d / static_cast<double>(ms.count);
  ms.m2_world += d.cwiseProduct(ew - ms.mean_world);

  if (e.head<2>().norm() <= cfg_.consistency_xy && std::abs(e(2)) <= cfg_.consistency_yaw) {
    m.cov_body *= cfg_.lidar_cov_inflation_under_fix;
    return LidarDecision::FUSE_INFLATED;
  }
  return LidarDecision::REJECT_MISMATCH;
}

}  // namespace gll
