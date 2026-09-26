#include "gll/estimation/output_smoother.hpp"

#include <algorithm>
#include <cmath>

namespace gll {

void OutputSmoother::onCorrection(const Vec3& world_delta) {
  offset_.head<2>() -= world_delta.head<2>();
  offset_(2) = wrapAngle(offset_(2) - world_delta(2));
}

SmoothedOutput OutputSmoother::apply(const Pose2D& raw, const Mat3& raw_cov, double dt) {
  dt = std::max(dt, 0.0);
  const double n = offset_.head<2>().norm();
  if (n > 0.0) {
    const double step = std::min(n, cfg_.max_rate_xy * dt);
    offset_.head<2>() *= (n - step) / n;
  }
  const double oy = offset_(2);
  const double step_yaw = std::min(std::abs(oy), cfg_.max_rate_yaw * dt);
  offset_(2) = oy - std::copysign(step_yaw, oy);

  SmoothedOutput out;
  out.pose.x = raw.x + offset_(0);
  out.pose.y = raw.y + offset_(1);
  out.pose.yaw = wrapAngle(raw.yaw + offset_(2));
  out.cov = raw_cov + offset_ * offset_.transpose();
  return out;
}

bool OutputSmoother::offsetExceeded() const {
  return offset_.head<2>().norm() > cfg_.offset_error_xy || std::abs(offset_(2)) > cfg_.offset_error_yaw;
}

}  // namespace gll
