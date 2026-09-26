#include "gll/measurement/stop_detector.hpp"

#include <cmath>

namespace gll {

std::optional<ZeroRateMeasurement> StopDetector::update(const MotionInput& u) {
  const bool still = std::abs(u.v) < cfg_.v_threshold && std::abs(u.v_lat) < cfg_.v_threshold &&
                     std::abs(u.omega) < cfg_.w_threshold;
  if (!still) {
    stop_since_ = -1.0;
    window_start_ = -1.0;
    sum_ = 0.0;
    n_ = 0;
    return std::nullopt;
  }
  if (stop_since_ < 0.0) stop_since_ = u.t;
  if (window_start_ < 0.0) window_start_ = u.t;
  sum_ += u.omega;
  ++n_;
  if (u.t - window_start_ < cfg_.min_duration || n_ < 2) return std::nullopt;

  ZeroRateMeasurement z;
  z.t = u.t;
  z.omega_mean = sum_ / n_;
  z.var = cfg_.zaru_stddev * cfg_.zaru_stddev;
  window_start_ = u.t;
  sum_ = 0.0;
  n_ = 0;
  return z;
}

}  // namespace gll
