#include "gll/estimation/status_monitor.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace gll {

void StatusMonitor::onAccepted(MeasurementKind kind, double t) {
  if (kind == MeasurementKind::GNSS_POSITION) last_gnss_ = std::max(last_gnss_, t);
  if (kind == MeasurementKind::POSE) last_lidar_ = std::max(last_lidar_, t);
}

LocalizationStatus StatusMonitor::evaluate(double t, bool ready, const Mat3& cov_world,
                                           bool offset_exceeded, RecoveryState recovery) const {
  if (!ready) return LocalizationStatus::INITIALIZING;
  if (recovery == RecoveryState::LOST) return LocalizationStatus::LOST;
  const Eigen::SelfAdjointEigenSolver<Mat2> es(cov_world.topLeftCorner<2, 2>());
  const double sigma = std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0));
  if (sigma >= cfg_.lost_stddev) return LocalizationStatus::LOST;
  if (sigma >= cfg_.dr_max_stddev || offset_exceeded) return LocalizationStatus::DEGRADED;
  const bool gnss = t - last_gnss_ <= cfg_.aid_timeout;
  const bool lidar = t - last_lidar_ <= cfg_.aid_timeout;
  if (gnss && lidar) return LocalizationStatus::GNSS_LIDAR_AIDED;
  if (gnss) return LocalizationStatus::GNSS_AIDED;
  if (lidar) return LocalizationStatus::LIDAR_AIDED;
  return LocalizationStatus::DEAD_RECKONING;
}

}  // namespace gll
