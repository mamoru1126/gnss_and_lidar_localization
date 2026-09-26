#include "gll_ros2/diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace gll_ros2 {
namespace {

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

const char* phaseName(gll::Initializer::Phase p) {
  switch (p) {
    case gll::Initializer::Phase::WAIT_FIX: return "WAIT_FIX";
    case gll::Initializer::Phase::WAIT_MOTION: return "WAIT_MOTION";
    case gll::Initializer::Phase::CONVERGING: return "CONVERGING";
    case gll::Initializer::Phase::READY: return "READY";
  }
  return "UNKNOWN";
}

diagnostic_msgs::msg::KeyValue kv(const std::string& k, const std::string& v) {
  diagnostic_msgs::msg::KeyValue x;
  x.key = k;
  x.value = v;
  return x;
}

}  // namespace

std::string toFixed(double v, int digits) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return buf;
}

uint8_t statusLevel(gll::LocalizationStatus s) {
  using S = gll::LocalizationStatus;
  switch (s) {
    case S::GNSS_AIDED:
    case S::LIDAR_AIDED:
    case S::GNSS_LIDAR_AIDED:
    case S::DEAD_RECKONING: return DiagStatus::OK;
    case S::INITIALIZING:
    case S::DEGRADED: return DiagStatus::WARN;
    case S::LOST: return DiagStatus::ERROR;
  }
  return DiagStatus::STALE;
}

DiagStatus makeLocalizationStatus(const gll::LocalizationOutput* out, const gll::Diagnostics& d,
                                  double dr_error_distance) {
  DiagStatus st;
  st.name = "gll_localizer";
  st.hardware_id = "localization";
  if (!out) {
    st.level = DiagStatus::WARN;
    st.message = "INITIALIZING";
    st.values.push_back(kv("init_phase", phaseName(d.init_phase)));
    st.values.push_back(kv("attitude_initialized", d.attitude_initialized ? "true" : "false"));
    return st;
  }
  st.level = statusLevel(out->status);
  st.message = gll::toString(out->status);
  // デッドレコニングが一定距離続いたら ERROR（設計書 3.12 節）。状態の重大度より優先する
  if (out->dr_distance_exceeded) {
    st.level = DiagStatus::ERROR;
    st.message += ": dead reckoning for " + toFixed(out->dr_distance, 1) +
                  " m without GNSS / LiDAR position (limit " + toFixed(dr_error_distance, 1) + " m)";
  }
  st.values.push_back(kv("recovery_state", gll::toString(out->recovery)));
  st.values.push_back(kv("dr_distance_m", toFixed(out->dr_distance, 2)));
  st.values.push_back(kv("dr_error_distance_m", toFixed(dr_error_distance, 1)));
  st.values.push_back(kv("pos_stddev_m", toFixed(std::sqrt(std::max(out->cov(0, 0), out->cov(1, 1))), 3)));
  st.values.push_back(kv("yaw_stddev_deg", toFixed(gll::rad2deg(std::sqrt(out->cov(2, 2))), 2)));
  st.values.push_back(kv("output_offset_m", toFixed(out->offset.head<2>().norm(), 3)));
  if (!out->active_map_group.empty()) st.values.push_back(kv("active_map_group", out->active_map_group));
  return st;
}

DiagStatus makeCounterStatus(const gll::Diagnostics& d) {
  DiagStatus st;
  st.name = "gll_localizer: counters";
  st.hardware_id = "localization";
  st.level = DiagStatus::OK;
  st.message = phaseName(d.init_phase);
  st.values.push_back(kv("imu", std::to_string(d.imu_count)));
  st.values.push_back(kv("odom", std::to_string(d.odom_count)));
  st.values.push_back(kv("gnss", std::to_string(d.gnss_count)));
  st.values.push_back(kv("gnss_accepted", std::to_string(d.gnss_accepted)));
  st.values.push_back(kv("gnss_deferred", std::to_string(d.gnss_deferred)));
  st.values.push_back(kv("gnss_too_old", std::to_string(d.gnss_too_old)));
  st.values.push_back(kv("heading_accepted", std::to_string(d.heading_accepted)));
  st.values.push_back(kv("reanchor", std::to_string(d.reanchor_count)));
  st.values.push_back(kv("zaru", std::to_string(d.zaru_accepted)));
  st.values.push_back(kv("odom_stale", std::to_string(d.odom_stale_count)));
  st.values.push_back(kv("imu_fallback", std::to_string(d.imu_fallback_count)));
  for (const auto& [reason, n] : d.gnss_reject_reasons) st.values.push_back(kv("gnss_reject_" + reason, std::to_string(n)));
  return st;
}

}  // namespace gll_ros2
