#include "gll_ros2/diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace gll_ros2 {
namespace {

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

const char* phaseName(gll::Initializer::Phase p) {
  switch (p) {
    case gll::Initializer::Phase::WAIT_FIX: return "WAIT_FIX";
    case gll::Initializer::Phase::WAIT_MOTION: return "WAIT_MOTION";
    case gll::Initializer::Phase::WAIT_MAP_MATCH: return "WAIT_MAP_MATCH";
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
  st.values.push_back(kv("lidar", std::to_string(d.lidar_count)));
  st.values.push_back(kv("lidar_matched", std::to_string(d.lidar_matched)));
  st.values.push_back(kv("lidar_accepted", std::to_string(d.lidar_accepted)));
  st.values.push_back(kv("lidar_deferred", std::to_string(d.lidar_deferred)));
  st.values.push_back(kv("lidar_mismatch", std::to_string(d.lidar_mismatch)));
  st.values.push_back(kv("lidar_dropped", std::to_string(d.lidar_dropped)));
  st.values.push_back(kv("lidar_no_target", std::to_string(d.lidar_no_target)));
  st.values.push_back(kv("lidar_too_old", std::to_string(d.lidar_too_old)));
  st.values.push_back(kv("lidar_reanchor", std::to_string(d.lidar_reanchor_count)));
  for (const auto& [reason, n] : d.lidar_reject_reasons) st.values.push_back(kv("lidar_reject_" + reason, std::to_string(n)));
  st.values.push_back(kv("relocalize", std::to_string(d.relocalize_success) + "/" + std::to_string(d.relocalize_attempts)));
  st.values.push_back(kv("map_init_attempts", std::to_string(d.map_init_attempts)));
  return st;
}

DiagStatus makeMapStatus(const gll::Diagnostics& d, const gll::ArbiterConfig& arb) {
  DiagStatus st;
  st.name = "gll_localizer: map";
  st.hardware_id = "localization";
  st.level = DiagStatus::OK;
  st.message = d.map.active_group.empty() ? "no active map group" : "active map group: " + d.map.active_group;
  st.values.push_back(kv("active_map_group", d.map.active_group));
  st.values.push_back(kv("loaded_tiles", std::to_string(d.map.loaded_tiles)));
  st.values.push_back(kv("target_tiles", std::to_string(d.map.target_tiles)));
  st.values.push_back(kv("target_points", std::to_string(d.map.target_points)));
  st.values.push_back(kv("last_match_ms", toFixed(d.last_match_ms, 1)));
  st.values.push_back(kv("last_inlier_ratio", toFixed(d.last_inlier_ratio, 2)));
  st.values.push_back(kv("last_overlap", toFixed(d.last_overlap, 2)));
  std::vector<std::string> warn;
  if (d.map.tile_load_failures > 0) warn.push_back(std::to_string(d.map.tile_load_failures) + " tile load failures");
  for (const auto& [group, ms] : d.anchor_mismatch) {
    const double exy = ms.mean_world.head<2>().norm();
    const double eyaw = std::abs(ms.mean_world.z());
    st.values.push_back(kv("anchor_mismatch_" + group, toFixed(exy, 3) + " m, " + toFixed(gll::rad2deg(eyaw), 2) +
                                                         " deg (n=" + std::to_string(ms.count) + ")"));
    if (static_cast<int>(ms.count) >= arb.anchor_mismatch_min_count &&
        (exy > arb.anchor_mismatch_warn_xy || eyaw > arb.anchor_mismatch_warn_yaw))
      warn.push_back("anchor of map group " + group + " needs calibration (" + toFixed(exy, 2) + " m, " +
                     toFixed(gll::rad2deg(eyaw), 2) + " deg from GNSS)");
  }
  if (!warn.empty()) {
    st.level = DiagStatus::WARN;
    st.message.clear();
    for (std::size_t i = 0; i < warn.size(); ++i) st.message += (i ? "; " : "") + warn[i];
  }
  return st;
}

}  // namespace gll_ros2
