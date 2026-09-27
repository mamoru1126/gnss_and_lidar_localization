// リー群 SE(2) の最小限の実装（アルゴリズム説明書 2 章）。
// 接空間のベクトルは xi = (rho_x, rho_y, phi) の順（並進が先、回転が後）。
#pragma once

#include "tiled_pcd_map/math.hpp"

#include <cmath>

namespace tiled_pcd_map {

class SE2 {
 public:
  SE2() : R_(Mat2::Identity()), t_(Vec2::Zero()) {}
  SE2(const Mat2& R, const Vec2& t) : R_(R), t_(t) {}

  static Mat2 rot(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat2 R;
    R << c, -s, s, c;
    return R;
  }

  /// 90 度回転 J = [[0, -1], [1, 0]]。
  static Mat2 J() {
    Mat2 m;
    m << 0.0, -1.0, 1.0, 0.0;
    return m;
  }

  static SE2 fromPose(double x, double y, double yaw) { return SE2(rot(yaw), Vec2(x, y)); }

  /// V(phi) = sin(phi)/phi I + (1 - cos(phi))/phi J。小角度ではテイラー展開する。
  static Mat2 V(double phi) {
    double a, b;
    if (std::abs(phi) < 1e-6) {
      const double p2 = phi * phi;
      a = 1.0 - p2 / 6.0;
      b = phi / 2.0 - phi * p2 / 24.0;
    } else {
      a = std::sin(phi) / phi;
      b = (1.0 - std::cos(phi)) / phi;
    }
    return a * Mat2::Identity() + b * J();
  }

  static SE2 Exp(const Vec3& xi) { return SE2(rot(xi(2)), V(xi(2)) * xi.head<2>()); }
  static SE2 Exp(const Vec2& rho, double phi) { return SE2(rot(phi), V(phi) * rho); }

  Vec3 Log() const {
    const double phi = yaw();
    Vec3 xi;
    xi.head<2>() = V(phi).inverse() * t_;
    xi(2) = phi;
    return xi;
  }

  /// 随伴行列 Ad = [[R, -J t], [0, 1]]。
  Mat3 Ad() const {
    Mat3 A = Mat3::Zero();
    A.topLeftCorner<2, 2>() = R_;
    A.topRightCorner<2, 1>() = -J() * t_;
    A(2, 2) = 1.0;
    return A;
  }

  SE2 inverse() const {
    const Mat2 Rt = R_.transpose();
    return SE2(Rt, -Rt * t_);
  }

  SE2 operator*(const SE2& o) const { return SE2(R_ * o.R_, R_ * o.t_ + t_); }

  /// 機体座標系の点を世界座標系に移す。
  Vec2 act(const Vec2& q) const { return R_ * q + t_; }

  double yaw() const { return std::atan2(R_(1, 0), R_(0, 0)); }
  Pose2D toPose() const { return Pose2D{t_.x(), t_.y(), yaw()}; }

  const Mat2& R() const { return R_; }
  const Vec2& t() const { return t_; }

  /// 数値誤差で直交性が崩れないように回転を作り直す。
  void normalize() { R_ = rot(yaw()); }

 private:
  Mat2 R_;
  Vec2 t_;
};

}  // namespace tiled_pcd_map
