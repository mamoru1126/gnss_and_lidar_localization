// 共通の型定義（ROS 非依存）。
// 座標系: base_link は前・左・上（FLU）。map は UTM（Easting, Northing, 楕円体高）。
#pragma once

#include "gll/common/math.hpp"

#include <optional>
#include <string>
#include <vector>

namespace gll {

constexpr double kGravity = 9.80665;

/// IMU の 1 サンプル（base_link 座標系に回転済み）。
struct ImuSample {
  double t = 0.0;
  Vec3 gyro = Vec3::Zero();  ///< 角速度 [rad/s]
  Vec3 acc = Vec3::Zero();   ///< 比力 [m/s^2]（静止時に +g が上向き）
};

/// ODOM の 1 サンプル（nav_msgs/Odometry の twist）。
struct OdomSample {
  double t = 0.0;
  double v = 0.0;      ///< 前進速度 [m/s]（twist.linear.x）
  double v_lat = 0.0;  ///< 横速度 [m/s]（twist.linear.y）
  std::optional<double> yaw_rate;  ///< ヨーレート [rad/s]（twist.angular.z）
};

enum class GnssFixType { NONE, SINGLE, DGPS, RTK_FLOAT, RTK_FIX };

/// GNSS の測位結果（sensor_msgs/NavSatFix 相当）。
struct GnssSample {
  double t = 0.0;
  double lat = 0.0;  ///< [deg]
  double lon = 0.0;  ///< [deg]
  double h = 0.0;    ///< 楕円体高 [m]
  Mat3 cov_enu = Mat3::Zero();  ///< ENU の共分散 [m^2]
  bool cov_known = false;
  int raw_status = -1;  ///< NavSatStatus.status の値
};

/// GNSS の速度（任意入力）。
struct GnssVelocitySample {
  double t = 0.0;
  Vec2 vel_en = Vec2::Zero();  ///< 東・北の速度 [m/s]
  Mat2 cov = Mat2::Identity();
};

/// LiDAR の 1 スキャン（LiDAR 座標系）。
/// t はスキャンの代表時刻（この時刻の base_link にデスキューする）。times は点ごとの t からの相対時刻 [s]
/// （LiDAR ドライバが出さない場合は空。そのときはデスキューしない）。
struct LidarScan {
  double t = 0.0;
  std::vector<Vec3f> points;
  std::vector<float> times;
};

enum class LocalizationStatus {
  INITIALIZING,
  GNSS_AIDED,
  LIDAR_AIDED,
  GNSS_LIDAR_AIDED,
  DEAD_RECKONING,
  DEGRADED,
  LOST
};

enum class RecoveryState { TRACKING, SUSPECT, REANCHOR, RELOCALIZE, LOST };

inline const char* toString(LocalizationStatus s) {
  switch (s) {
    case LocalizationStatus::INITIALIZING: return "INITIALIZING";
    case LocalizationStatus::GNSS_AIDED: return "GNSS_AIDED";
    case LocalizationStatus::LIDAR_AIDED: return "LIDAR_AIDED";
    case LocalizationStatus::GNSS_LIDAR_AIDED: return "GNSS_LIDAR_AIDED";
    case LocalizationStatus::DEAD_RECKONING: return "DEAD_RECKONING";
    case LocalizationStatus::DEGRADED: return "DEGRADED";
    case LocalizationStatus::LOST: return "LOST";
  }
  return "UNKNOWN";
}

inline const char* toString(RecoveryState s) {
  switch (s) {
    case RecoveryState::TRACKING: return "TRACKING";
    case RecoveryState::SUSPECT: return "SUSPECT";
    case RecoveryState::REANCHOR: return "REANCHOR";
    case RecoveryState::RELOCALIZE: return "RELOCALIZE";
    case RecoveryState::LOST: return "LOST";
  }
  return "UNKNOWN";
}

/// Localizer の出力。
struct LocalizationOutput {
  double t = 0.0;
  Pose2D pose;       ///< 出力整形後（map = UTM）
  Pose2D raw_pose;   ///< フィルタの推定値
  Mat3 cov = Mat3::Identity();      ///< 出力の共分散（Σ_w + o oᵀ、世界座標系の x, y, yaw）
  Mat3 raw_cov = Mat3::Identity();  ///< フィルタの共分散 Σ_w
  Vec3 offset = Vec3::Zero();       ///< 出力整形のオフセット
  double v = 0.0;
  double v_lat = 0.0;
  double yaw_rate = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double gyro_bias = 0.0;
  double odom_scale = 1.0;
  LocalizationStatus status = LocalizationStatus::INITIALIZING;
  RecoveryState recovery = RecoveryState::TRACKING;
  std::string active_map_group;
  double dr_distance = 0.0;           ///< 最後に位置の観測を採用してから走った距離 [m]
  bool dr_distance_exceeded = false;  ///< dr_distance が dr_error_distance を超えた（診断でエラー）
};

}  // namespace gll
