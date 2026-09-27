#include "tiled_pcd_map/map_anchor.hpp"

#include <cmath>

namespace tiled_pcd_map {

MapAnchor MapAnchor::fromConfig(const AnchorConfig& cfg, const UtmProjector& utm) {
  MapAnchor a;
  a.a_map_ = cfg.map_point;
  UtmPoint u;
  double alpha = 0.0;  // 地図の x 軸のグリッド方位（グリッド北から時計回り）
  if (cfg.use_utm) {
    // UTM で与えられた場合も、子午線収差と縮尺係数は、その点の緯度経度から求める
    const LatLon ll = utm.inverse(cfg.easting, cfg.northing);
    u = utm.forward(ll.lat, ll.lon);
    u.easting = cfg.easting;
    u.northing = cfg.northing;
    alpha = cfg.grid_heading;
  } else {
    u = utm.forward(cfg.latitude, cfg.longitude);
    // 真北からの方位 ψ をグリッド方位にする（γ はグリッド北の、真北からの時計回り角）
    alpha = cfg.heading - u.convergence;
  }
  a.a_utm_ = Vec3(u.easting, u.northing, cfg.ellipsoid_height);
  a.phi_ = wrapAngle(kPi / 2.0 - alpha);  // 東から反時計回り
  a.k_ = cfg.use_scale_factor ? u.scale : 1.0;
  a.stddev_xy_ = cfg.stddev_xy;
  a.stddev_yaw_ = cfg.stddev_yaw;
  return a;
}

MapAnchor MapAnchor::identity() { return MapAnchor(); }

Vec3 MapAnchor::mapToUtm(const Vec3& p) const {
  Vec3 q;
  q.head<2>() = a_utm_.head<2>() + k_ * (SE2::rot(phi_) * (p.head<2>() - a_map_.head<2>()));
  q.z() = a_utm_.z() + (p.z() - a_map_.z());
  return q;
}

Vec3 MapAnchor::utmToMap(const Vec3& q) const {
  Vec3 p;
  p.head<2>() = a_map_.head<2>() + SE2::rot(phi_).transpose() * (q.head<2>() - a_utm_.head<2>()) / k_;
  p.z() = a_map_.z() + (q.z() - a_utm_.z());
  return p;
}

Eigen::Isometry3d MapAnchor::poseMapToUtm(const Eigen::Isometry3d& T) const {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = Eigen::AngleAxisd(phi_, Vec3::UnitZ()).toRotationMatrix() * T.linear();
  out.translation() = mapToUtm(T.translation());
  return out;
}

Eigen::Isometry3d MapAnchor::poseUtmToMap(const Eigen::Isometry3d& T) const {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = Eigen::AngleAxisd(-phi_, Vec3::UnitZ()).toRotationMatrix() * T.linear();
  out.translation() = utmToMap(T.translation());
  return out;
}

SE2 MapAnchor::toUtm(const SE2& X) const {
  const Vec3 q = mapToUtm(Vec3(X.t().x(), X.t().y(), a_map_.z()));
  return SE2::fromPose(q.x(), q.y(), wrapAngle(X.yaw() + phi_));
}

SE2 MapAnchor::toMap(const SE2& X) const {
  const Vec3 p = utmToMap(Vec3(X.t().x(), X.t().y(), a_utm_.z()));
  return SE2::fromPose(p.x(), p.y(), wrapAngle(X.yaw() - phi_));
}

Mat3 MapAnchor::covarianceWorld() const {
  return Vec3(stddev_xy_ * stddev_xy_, stddev_xy_ * stddev_xy_, stddev_yaw_ * stddev_yaw_).asDiagonal();
}

double yawOf(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }

Eigen::Isometry3d makePose(const Vec3& t, double roll, double pitch, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                Eigen::AngleAxisd(roll, Vec3::UnitX()))
                   .toRotationMatrix();
  T.translation() = t;
  return T;
}

}  // namespace tiled_pcd_map
