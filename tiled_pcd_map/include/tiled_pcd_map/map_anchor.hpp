// 地図グループの座標系（pcd_<group>）⇔ map（UTM）の変換（設計書 4.2 節）。
// 水平は相似変換（回転 φ、縮尺 k）、鉛直は高さのずれだけを扱う（地図の z 軸は鉛直にそろっている前提）。
#pragma once

#include "tiled_pcd_map/geodesy.hpp"
#include "tiled_pcd_map/se2.hpp"
#include "tiled_pcd_map/math.hpp"

#include <Eigen/Geometry>

#include <string>

namespace tiled_pcd_map {

/// アンカーの設定。緯度経度で与える（use_utm = false）か、UTM で直接与える（use_utm = true）。
struct AnchorConfig {
  Vec3 map_point = Vec3::Zero();  ///< アンカー点の地図座標 [m]
  bool use_utm = false;
  // 緯度経度で与える場合
  double latitude = 0.0;          ///< [deg]
  double longitude = 0.0;         ///< [deg]
  double heading = 0.0;           ///< 地図の x 軸の方位（真北から時計回り）[rad]
  // UTM で与える場合
  double easting = 0.0;           ///< [m]
  double northing = 0.0;          ///< [m]
  double grid_heading = 0.0;      ///< 地図の x 軸の方位（グリッド北から時計回り）[rad]
  // 共通
  double ellipsoid_height = 0.0;  ///< アンカー点の楕円体高 [m]
  bool use_scale_factor = true;   ///< UTM の縮尺係数 k を使う（地図は実距離なので、通常は true）
  double stddev_xy = 0.05;        ///< アンカーの不確かさ [m]
  double stddev_yaw = deg2rad(0.2);  ///< [rad]
};

class MapAnchor {
 public:
  MapAnchor() = default;

  /// 設定からアンカーを作る。緯度経度の場合は、その点の UTM 座標・子午線収差・縮尺係数を求める。
  static MapAnchor fromConfig(const AnchorConfig& cfg, const UtmProjector& utm);

  /// 恒等変換（地図座標をそのまま map として扱う。テスト用）。
  static MapAnchor identity();

  /// 地図座標の点 → UTM（x = Easting、y = Northing、z = 楕円体高）。
  Vec3 mapToUtm(const Vec3& p_map) const;
  Vec3 utmToMap(const Vec3& p_utm) const;

  /// 地図座標系の姿勢 → UTM の姿勢（位置は相似変換、姿勢は鉛直軸まわりに φ 回す）。
  Eigen::Isometry3d poseMapToUtm(const Eigen::Isometry3d& T_map) const;
  Eigen::Isometry3d poseUtmToMap(const Eigen::Isometry3d& T_utm) const;

  /// 水平の姿勢（x, y, yaw）の変換。
  SE2 toUtm(const SE2& X_map) const;
  SE2 toMap(const SE2& X_utm) const;

  /// 地図の x 軸の、UTM の東からの反時計回りの角度 φ [rad]。
  double rotation() const { return phi_; }
  double scale() const { return k_; }
  const Vec3& mapPoint() const { return a_map_; }
  const Vec3& utmPoint() const { return a_utm_; }

  /// アンカーの不確かさ（世界座標系の x, y, yaw）。
  Mat3 covarianceWorld() const;

 private:
  Vec3 a_map_ = Vec3::Zero();
  Vec3 a_utm_ = Vec3::Zero();
  double phi_ = 0.0;
  double k_ = 1.0;
  double stddev_xy_ = 0.0;
  double stddev_yaw_ = 0.0;
};

/// 3 次元の回転から yaw（ZYX オイラー角の z）を取り出す。
double yawOf(const Eigen::Matrix3d& R);

/// x, y, z と roll, pitch, yaw から姿勢を作る（R = Rz(yaw) Ry(pitch) Rx(roll)）。
Eigen::Isometry3d makePose(const Vec3& t, double roll, double pitch, double yaw);

}  // namespace tiled_pcd_map
