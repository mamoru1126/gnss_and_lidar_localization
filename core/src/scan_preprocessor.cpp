#include "gll/matching/scan_preprocessor.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <map>

namespace gll {

std::vector<Vec3f> ScanPreprocessor::process(const LidarScan& scan, const ScanMotion& motion) const {
  std::vector<Vec3f> out;
  out.reserve(scan.points.size());
  const Eigen::Matrix3f R_bl = cfg_.T_base_lidar.linear().cast<float>();
  const Vec3f t_bl = cfg_.T_base_lidar.translation().cast<float>();
  const bool deskew = cfg_.deskew && scan.times.size() == scan.points.size() &&
                      (motion.angular_velocity.squaredNorm() > 0.0 || motion.velocity.squaredNorm() > 0.0);
  const float rmin2 = static_cast<float>(cfg_.min_range * cfg_.min_range);
  const float rmax2 = static_cast<float>(cfg_.max_range * cfg_.max_range);
  const Vec3f bmin = cfg_.crop_box_min.cast<float>();
  const Vec3f bmax = cfg_.crop_box_max.cast<float>();

  // 時刻を 0.1 ms 刻みにまとめて、補正の変換を使い回す（点ごとに回転を作ると遅いため）
  std::map<int, std::pair<Eigen::Matrix3f, Vec3f>> cache;
  const auto motionAt = [&](float tau) -> const std::pair<Eigen::Matrix3f, Vec3f>& {
    const int key = static_cast<int>(std::lround(tau * 1e4f));
    auto it = cache.find(key);
    if (it == cache.end()) {
      const double dt = key * 1e-4;
      // 時刻 scan.t + dt の base_link の、時刻 scan.t の base_link から見た姿勢。
      // 速度と角速度が一定（機体座標系）なら、SE(3) の指数写像 Exp(dt [ω, v]) になる
      const Vec3 w = motion.angular_velocity * dt;
      const Vec3 u = motion.velocity * dt;
      const double th = w.norm();
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
      Vec3 t = u;
      if (th > 1e-9) {
        const Vec3 a = w / th;
        R = Eigen::AngleAxisd(th, a).toRotationMatrix();
        Eigen::Matrix3d K;
        K << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
        const Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + (1.0 - std::cos(th)) / th * K +
                                  (th - std::sin(th)) / th * K * K;
        t = V * u;
      }
      it = cache.emplace(key, std::make_pair(R.cast<float>(), t.cast<float>())).first;
    }
    return it->second;
  };

  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const Vec3f& pl = scan.points[i];
    if (!pl.allFinite()) continue;
    const float r2 = pl.squaredNorm();
    if (r2 < rmin2 || r2 > rmax2) continue;
    Vec3f pb = R_bl * pl + t_bl;
    if (deskew) {
      const auto& m = motionAt(scan.times[i]);
      pb = m.first * pb + m.second;
    }
    if (cfg_.crop_box_enabled && (pb.array() >= bmin.array()).all() && (pb.array() <= bmax.array()).all()) continue;
    out.push_back(pb);
  }
  return out;
}

}  // namespace gll
