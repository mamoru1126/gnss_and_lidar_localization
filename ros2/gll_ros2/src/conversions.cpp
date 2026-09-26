#include "gll_ros2/conversions.hpp"

#include <cmath>

namespace gll_ros2 {

double toSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time toStamp(double sec) {
  builtin_interfaces::msg::Time t;
  double s = std::floor(sec);
  double ns = std::round((sec - s) * 1e9);
  if (ns >= 1e9) {
    s += 1.0;
    ns -= 1e9;
  }
  t.sec = static_cast<int32_t>(s);
  t.nanosec = static_cast<uint32_t>(ns);
  return t;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

gll::ImuSample toCore(const sensor_msgs::msg::Imu& m, const Eigen::Matrix3d& R_base_imu) {
  gll::ImuSample s;
  s.t = toSec(m.header.stamp);
  const Eigen::Vector3d w(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
  const Eigen::Vector3d a(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
  s.gyro = R_base_imu * w;
  s.acc = R_base_imu * a;
  return s;
}

gll::OdomSample toCore(const nav_msgs::msg::Odometry& m) {
  gll::OdomSample s;
  s.t = toSec(m.header.stamp);
  s.v = m.twist.twist.linear.x;
  s.v_lat = m.twist.twist.linear.y;
  s.yaw_rate = m.twist.twist.angular.z;
  return s;
}

gll::GnssSample toCore(const sensor_msgs::msg::NavSatFix& m, double stamp_offset) {
  gll::GnssSample s;
  s.t = toSec(m.header.stamp) + stamp_offset;
  s.lat = m.latitude;
  s.lon = m.longitude;
  s.h = m.altitude;
  s.raw_status = m.status.status;
  s.cov_known = m.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) s.cov_enu(r, c) = m.position_covariance[static_cast<std::size_t>(r * 3 + c)];
  return s;
}

gll::GnssVelocitySample toCore(const geometry_msgs::msg::TwistWithCovarianceStamped& m, double stamp_offset) {
  gll::GnssVelocitySample s;
  s.t = toSec(m.header.stamp) + stamp_offset;
  s.vel_en = gll::Vec2(m.twist.twist.linear.x, m.twist.twist.linear.y);
  const auto& c = m.twist.covariance;
  s.cov << c[0], c[1], c[6], c[7];
  return s;
}

std::array<double, 36> toCovariance6(const gll::Mat3& cov, double other_var) {
  std::array<double, 36> c{};
  const int idx[3] = {0, 1, 5};  // x, y, yaw
  for (int r = 0; r < 3; ++r)
    for (int k = 0; k < 3; ++k) c[static_cast<std::size_t>(idx[r] * 6 + idx[k])] = cov(r, k);
  c[2 * 6 + 2] = other_var;
  c[3 * 6 + 3] = other_var;
  c[4 * 6 + 4] = other_var;
  return c;
}

gll::Mat3 fromCovariance6(const std::array<double, 36>& c) {
  gll::Mat3 m;
  const int idx[3] = {0, 1, 5};
  for (int r = 0; r < 3; ++r)
    for (int k = 0; k < 3; ++k) m(r, k) = c[static_cast<std::size_t>(idx[r] * 6 + idx[k])];
  return m;
}

}  // namespace gll_ros2
