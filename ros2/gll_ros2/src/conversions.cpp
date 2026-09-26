#include "gll_ros2/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

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

namespace {

using sensor_msgs::msg::PointField;

const PointField* findField(const sensor_msgs::msg::PointCloud2& m, const std::string& name) {
  for (const auto& f : m.fields)
    if (f.name == name) return &f;
  return nullptr;
}

double readField(const uint8_t* p, uint8_t datatype) {
  switch (datatype) {
    case PointField::INT8: { int8_t v; std::memcpy(&v, p, 1); return v; }
    case PointField::UINT8: { uint8_t v; std::memcpy(&v, p, 1); return v; }
    case PointField::INT16: { int16_t v; std::memcpy(&v, p, 2); return v; }
    case PointField::UINT16: { uint16_t v; std::memcpy(&v, p, 2); return v; }
    case PointField::INT32: { int32_t v; std::memcpy(&v, p, 4); return v; }
    case PointField::UINT32: { uint32_t v; std::memcpy(&v, p, 4); return v; }
    case PointField::FLOAT32: { float v; std::memcpy(&v, p, 4); return v; }
    case PointField::FLOAT64: { double v; std::memcpy(&v, p, 8); return v; }
    default: return std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace

gll::LidarScan toCore(const sensor_msgs::msg::PointCloud2& m, const PointCloudOptions& opt,
                      std::string* time_field_used) {
  gll::LidarScan s;
  const double stamp = toSec(m.header.stamp);
  s.t = stamp + opt.stamp_offset;
  if (time_field_used) time_field_used->clear();
  const PointField* fx = findField(m, "x");
  const PointField* fy = findField(m, "y");
  const PointField* fz = findField(m, "z");
  if (!fx || !fy || !fz || m.is_bigendian) return s;
  const PointField* ft = nullptr;
  if (opt.time_field == "auto") {
    for (const char* name : {"time", "t", "timestamp", "time_stamp", "offset_time"})
      if ((ft = findField(m, name))) break;
  } else if (!opt.time_field.empty()) {
    ft = findField(m, opt.time_field);
  }
  const std::size_t n = static_cast<std::size_t>(m.width) * m.height;
  if (m.data.size() < n * m.point_step) return s;
  s.points.reserve(n);
  std::vector<double> abs_times;
  if (ft) abs_times.reserve(n);

  // 時刻の単位（設計書 6.1 節。ドライバごとに異なる）
  enum class TimeKind { REL_NS, REL_SEC, ABS_NS, ABS_SEC };
  TimeKind kind = TimeKind::REL_SEC;
  if (ft) {
    if (ft->datatype == PointField::FLOAT32) {
      kind = TimeKind::REL_SEC;
    } else if (ft->datatype == PointField::FLOAT64) {
      double mag = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        const double v = readField(&m.data[i * m.point_step + ft->offset], ft->datatype);
        if (std::isfinite(v) && v != 0.0) {
          mag = std::abs(v);
          break;
        }
      }
      kind = mag > 1e15 ? TimeKind::ABS_NS : mag > 1e6 ? TimeKind::ABS_SEC : TimeKind::REL_SEC;
    } else {
      kind = TimeKind::REL_NS;
    }
  }
  double t_max = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const uint8_t* p = &m.data[i * m.point_step];
    const double x = readField(p + fx->offset, fx->datatype);
    const double y = readField(p + fy->offset, fy->datatype);
    const double z = readField(p + fz->offset, fz->datatype);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    s.points.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    if (ft) {
      const double v = readField(p + ft->offset, ft->datatype);
      double ta = stamp;
      switch (kind) {
        case TimeKind::REL_NS: ta = stamp + 1e-9 * v; break;
        case TimeKind::REL_SEC: ta = stamp + v; break;
        case TimeKind::ABS_NS: ta = 1e-9 * v; break;
        case TimeKind::ABS_SEC: ta = v; break;
      }
      abs_times.push_back(ta);
      t_max = std::max(t_max, ta);
    }
  }
  if (ft && !abs_times.empty()) {
    s.t = t_max + opt.stamp_offset;
    s.times.resize(abs_times.size());
    for (std::size_t i = 0; i < abs_times.size(); ++i) s.times[i] = static_cast<float>(abs_times[i] - t_max);
    if (time_field_used) *time_field_used = ft->name;
  }
  return s;
}

sensor_msgs::msg::PointCloud2 toPointCloud2(const std::vector<gll::Vec3f>& points, const std::string& frame_id,
                                            const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.height = 1;
  m.width = static_cast<uint32_t>(points.size());
  const char* names[3] = {"x", "y", "z"};
  for (uint32_t k = 0; k < 3; ++k) {
    PointField f;
    f.name = names[k];
    f.offset = 4 * k;
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    m.fields.push_back(f);
  }
  m.is_bigendian = false;
  m.point_step = 12;
  m.row_step = m.point_step * m.width;
  m.is_dense = true;
  m.data.resize(m.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) std::memcpy(&m.data[i * 12], points[i].data(), 12);
  return m;
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
