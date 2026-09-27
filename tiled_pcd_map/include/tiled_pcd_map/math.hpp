// 基本の型と角度の関数（ROS 非依存）。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace tiled_pcd_map {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec5 = Eigen::Matrix<double, 5, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat5 = Eigen::Matrix<double, 5, 5>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

/// 角度を [-pi, pi) に正規化する。
inline double wrapAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

/// 水平の姿勢（x, y [m]、yaw [rad]）。
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

}  // namespace tiled_pcd_map
