#include "gll/estimation/status_monitor.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace gll {

void StatusMonitor::onAccepted(MeasurementKind kind, double t) {
  if (kind == MeasurementKind::GNSS_POSITION) last_gnss_ = std::max(last_gnss_, t);
  if (kind == MeasurementKind::POSE) last_lidar_ = std::max(last_lidar_, t);
  // 位置を直す観測だけがデッドレコニングを終わらせる（進行方位や ZARU は数えない）
  if (kind == MeasurementKind::GNSS_POSITION || kind == MeasurementKind::POSE) dr_distance_ = 0.0;
}

namespace {

double maxStddev(const Mat3& cov_world) {
  const Eigen::SelfAdjointEigenSolver<Mat2> es(cov_world.topLeftCorner<2, 2>());
  return std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0));
}

}  // namespace

bool StatusMonitor::lostByCovariance(const Mat3& cov_world, double yaw) const {
  if (maxStddev(cov_world) >= cfg_.lost_stddev) return true;
  if (cfg_.lost_stddev_lateral > 0.0) {
    const Vec2 n(-std::sin(yaw), std::cos(yaw));  // 車の左向き（世界座標系）
    const double var_lat = n.dot(cov_world.topLeftCorner<2, 2>() * n);
    if (std::sqrt(std::max(var_lat, 0.0)) >= cfg_.lost_stddev_lateral) return true;
  }
  return false;
}

LocalizationStatus StatusMonitor::evaluate(double t, bool ready, const Mat3& cov_world,
                                           bool offset_exceeded, RecoveryState recovery, double yaw) const {
  if (!ready) return LocalizationStatus::INITIALIZING;
  if (recovery == RecoveryState::LOST) return LocalizationStatus::LOST;
  if (lostByCovariance(cov_world, yaw)) return LocalizationStatus::LOST;
  const double sigma = maxStddev(cov_world);
  if (sigma >= cfg_.dr_max_stddev || offset_exceeded) return LocalizationStatus::DEGRADED;
  const bool gnss = t - last_gnss_ <= cfg_.aid_timeout;
  const bool lidar = t - last_lidar_ <= cfg_.aid_timeout;
  if (gnss && lidar) return LocalizationStatus::GNSS_LIDAR_AIDED;
  if (gnss) return LocalizationStatus::GNSS_AIDED;
  if (lidar) return LocalizationStatus::LIDAR_AIDED;
  return LocalizationStatus::DEAD_RECKONING;
}

}  // namespace gll
