// テスト用の合成環境（地面・箱・円柱）と、レイキャストによる LiDAR の模擬。
// 地図の点群（表面を格子で打った点）と、回転式 LiDAR のスキャン（点ごとの時刻付き）を作る。
#pragma once

#include "gll/common/types.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <vector>

namespace gll::sim {

/// 鉛直軸まわりに yaw だけ回した箱。
struct Box {
  Vec3 center;
  Vec3 size;
  double yaw = 0.0;
};

/// 鉛直な円柱。
struct Cylinder {
  Vec2 center;
  double radius = 0.3;
  double z_min = 0.0;
  double z_max = 3.0;
};

struct World {
  double ground_z = 0.0;
  Vec2 ground_min = Vec2(-100, -100);
  Vec2 ground_max = Vec2(100, 100);
  std::vector<Box> boxes;
  std::vector<Cylinder> cylinders;

  /// 表面を spacing 間隔の格子で打った点群（地図）。noise は各座標に加える正規雑音の標準偏差。
  std::vector<Vec3f> samplePoints(double spacing, double noise, unsigned seed = 1) const {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n(0.0, 1.0);
    std::vector<Vec3f> pts;
    const auto add = [&](const Vec3& p) {
      pts.emplace_back((p + noise * Vec3(n(rng), n(rng), n(rng))).cast<float>());
    };
    // 地面（箱の中は除く）
    for (double x = ground_min.x(); x <= ground_max.x(); x += spacing)
      for (double y = ground_min.y(); y <= ground_max.y(); y += spacing)
        if (!insideAnyBox(Vec3(x, y, ground_z + 0.01))) add(Vec3(x, y, ground_z));
    // 箱の側面と上面
    for (const auto& b : boxes) {
      const Mat3 R = Eigen::AngleAxisd(b.yaw, Vec3::UnitZ()).toRotationMatrix();
      const Vec3 h = 0.5 * b.size;
      const auto face = [&](int axis, double sign) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        for (double a = -h[u]; a <= h[u] + 1e-9; a += spacing)
          for (double c = -h[v]; c <= h[v] + 1e-9; c += spacing) {
            Vec3 q;
            q[axis] = sign * h[axis];
            q[u] = a;
            q[v] = c;
            const Vec3 w = b.center + R * q;
            if (w.z() >= ground_z - 1e-6) add(w);
          }
      };
      face(0, 1);
      face(0, -1);
      face(1, 1);
      face(1, -1);
      face(2, 1);
    }
    // 円柱の側面
    for (const auto& c : cylinders) {
      const int na = std::max(8, static_cast<int>(2 * M_PI * c.radius / spacing));
      for (int i = 0; i < na; ++i) {
        const double a = 2 * M_PI * i / na;
        for (double z = c.z_min; z <= c.z_max + 1e-9; z += spacing)
          add(Vec3(c.center.x() + c.radius * std::cos(a), c.center.y() + c.radius * std::sin(a), z));
      }
    }
    return pts;
  }

  bool insideAnyBox(const Vec3& p) const {
    for (const auto& b : boxes) {
      const Vec3 q = Eigen::AngleAxisd(-b.yaw, Vec3::UnitZ()) * (p - b.center);
      if ((q.cwiseAbs().array() <= 0.5 * b.size.array()).all()) return true;
    }
    return false;
  }

  /// 光線 o + t d（|d| = 1）が最初に当たる距離。当たらなければ +inf。
  double raycast(const Vec3& o, const Vec3& d, double max_range) const {
    double best = std::numeric_limits<double>::infinity();
    // 地面
    if (d.z() < -1e-9) {
      const double t = (ground_z - o.z()) / d.z();
      const Vec3 p = o + t * d;
      if (t > 0 && p.x() >= ground_min.x() && p.x() <= ground_max.x() && p.y() >= ground_min.y() &&
          p.y() <= ground_max.y())
        best = std::min(best, t);
    }
    // 箱（箱の座標系でのスラブ法）
    for (const auto& b : boxes) {
      const Eigen::AngleAxisd Ri(-b.yaw, Vec3::UnitZ());
      const Vec3 lo = Ri * (o - b.center), ld = Ri * d;
      const Vec3 h = 0.5 * b.size;
      double t0 = 0.0, t1 = best;
      bool hit = true;
      for (int k = 0; k < 3 && hit; ++k) {
        if (std::abs(ld[k]) < 1e-12) {
          if (std::abs(lo[k]) > h[k]) hit = false;
        } else {
          double ta = (-h[k] - lo[k]) / ld[k], tb = (h[k] - lo[k]) / ld[k];
          if (ta > tb) std::swap(ta, tb);
          t0 = std::max(t0, ta);
          t1 = std::min(t1, tb);
          if (t0 > t1) hit = false;
        }
      }
      if (hit && t0 > 1e-6) best = std::min(best, t0);
    }
    // 円柱（側面だけ）
    for (const auto& c : cylinders) {
      const Vec2 oc = o.head<2>() - c.center;
      const Vec2 dd = d.head<2>();
      const double A = dd.squaredNorm();
      if (A < 1e-12) continue;
      const double B = 2 * oc.dot(dd), C = oc.squaredNorm() - c.radius * c.radius;
      const double disc = B * B - 4 * A * C;
      if (disc < 0) continue;
      const double t = (-B - std::sqrt(disc)) / (2 * A);
      if (t > 1e-6 && t < best) {
        const double z = o.z() + t * d.z();
        if (z >= c.z_min && z <= c.z_max) best = t;
      }
    }
    return best <= max_range ? best : std::numeric_limits<double>::infinity();
  }
};

/// 回転式 LiDAR のモデル。
struct LidarModel {
  int beams = 16;
  double vfov_min = -15.0 * M_PI / 180.0;
  double vfov_max = 15.0 * M_PI / 180.0;
  int columns = 900;          ///< 1 回転の水平方向の数
  double period = 0.1;        ///< 1 回転の時間 [s]
  double max_range = 60.0;
  double range_noise = 0.02;  ///< [m]
};

/// 1 スキャンを作る。sensor_pose_at(τ) は、スキャン開始から τ 秒後の LiDAR の姿勢（世界座標系）。
/// 点は、その点を測った時刻の LiDAR 座標系で表す（実機の回転式 LiDAR と同じ）。
/// 返すスキャンの t は scan_end（スキャン終了時刻）、times は t からの相対時刻（負の値）。
inline LidarScan simulateScan(const World& w, const LidarModel& m,
                              const std::function<Eigen::Isometry3d(double)>& sensor_pose_at, double scan_end,
                              std::mt19937& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  LidarScan s;
  s.t = scan_end;
  for (int c = 0; c < m.columns; ++c) {
    const double tau = m.period * c / m.columns;
    const Eigen::Isometry3d T = sensor_pose_at(tau);
    const double az = 2 * M_PI * c / m.columns;
    for (int b = 0; b < m.beams; ++b) {
      const double el = m.beams == 1 ? 0.0 : m.vfov_min + (m.vfov_max - m.vfov_min) * b / (m.beams - 1);
      const Vec3 dl(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      const Vec3 dw = T.linear() * dl;
      const double r = w.raycast(T.translation(), dw, m.max_range);
      if (!std::isfinite(r)) continue;
      const double rn = r + m.range_noise * n(rng);
      s.points.emplace_back((dl * rn).cast<float>());
      s.times.push_back(static_cast<float>(tau - m.period));
    }
  }
  return s;
}

/// テストで共通に使う屋外の環境（非対称に配置した建物・柱・車）。範囲はおよそ ±70 m。
inline World campusWorld() {
  World w;
  w.ground_min = Vec2(-90, -90);
  w.ground_max = Vec2(90, 90);
  // 建物
  w.boxes.push_back({Vec3(-30, 25, 4), Vec3(20, 10, 8), 0.1});
  w.boxes.push_back({Vec3(15, 30, 3), Vec3(12, 14, 6), -0.2});
  w.boxes.push_back({Vec3(40, 5, 5), Vec3(8, 25, 10), 0.35});
  w.boxes.push_back({Vec3(-40, -20, 3.5), Vec3(15, 18, 7), 0.0});
  w.boxes.push_back({Vec3(10, -35, 2.5), Vec3(30, 8, 5), 0.05});
  w.boxes.push_back({Vec3(-5, 55, 6), Vec3(25, 6, 12), 0.0});
  w.boxes.push_back({Vec3(60, -40, 4), Vec3(10, 10, 8), 0.6});
  // 車・小さな箱
  w.boxes.push_back({Vec3(-12, 8, 0.8), Vec3(4.5, 1.8, 1.6), 0.3});
  w.boxes.push_back({Vec3(22, -12, 0.75), Vec3(4.2, 1.8, 1.5), -1.2});
  w.boxes.push_back({Vec3(5, 14, 0.5), Vec3(1.0, 1.0, 1.0), 0.0});
  w.boxes.push_back({Vec3(-22, -3, 1.0), Vec3(2.0, 6.0, 2.0), 0.7});
  // 柱・街灯
  for (int i = 0; i < 8; ++i) w.cylinders.push_back({Vec2(-25 + 9 * i, -8 - 0.7 * i), 0.15, 0.0, 5.0});
  w.cylinders.push_back({Vec2(30, 20), 0.5, 0.0, 4.0});
  w.cylinders.push_back({Vec2(-15, 35), 0.4, 0.0, 3.0});
  return w;
}

}  // namespace gll::sim
