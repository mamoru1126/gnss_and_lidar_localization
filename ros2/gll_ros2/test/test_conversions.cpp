#include "gll_ros2/conversions.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <tuple>
#include <vector>

using namespace gll_ros2;

TEST(Conversions, StampRoundTrip) {
  const double t = 1727400000.123456789;
  EXPECT_NEAR(toSec(toStamp(t)), t, 1e-6);
}

TEST(Conversions, YawQuaternionRoundTrip) {
  for (double yaw : {-3.0, -1.0, 0.0, 0.5, 2.9}) EXPECT_NEAR(quaternionToYaw(yawToQuaternion(yaw)), yaw, 1e-12);
}

TEST(Conversions, ImuFrdToFlu) {
  // 前・右・下（FRD）の IMU を roll = 180° で base_link（FLU）に回す
  sensor_msgs::msg::Imu m;
  m.angular_velocity.x = 0.1;
  m.angular_velocity.y = 0.2;
  m.angular_velocity.z = 0.3;
  m.linear_acceleration.z = -9.8;  // FRD では重力の反力が -z
  const auto s = toCore(m, rpyToMatrix(gll::kPi, 0.0, 0.0));
  EXPECT_NEAR(s.gyro.x(), 0.1, 1e-12);
  EXPECT_NEAR(s.gyro.y(), -0.2, 1e-12);
  EXPECT_NEAR(s.gyro.z(), -0.3, 1e-12);
  EXPECT_NEAR(s.acc.z(), 9.8, 1e-12);
}

TEST(Conversions, NavSatFix) {
  sensor_msgs::msg::NavSatFix m;
  m.header.stamp = toStamp(100.0);
  m.latitude = 35.68;
  m.longitude = 139.77;
  m.altitude = 40.0;
  m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  m.position_covariance = {1e-4, 0, 0, 0, 4e-4, 0, 0, 0, 9e-4};
  const auto s = toCore(m, -0.05);
  EXPECT_NEAR(s.t, 99.95, 1e-9);
  EXPECT_EQ(s.raw_status, 2);
  EXPECT_TRUE(s.cov_known);
  EXPECT_NEAR(s.cov_enu(1, 1), 4e-4, 1e-15);

  m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  EXPECT_FALSE(toCore(m, 0.0).cov_known);
}

TEST(Conversions, OdometryUsesTwistOnly) {
  nav_msgs::msg::Odometry m;
  m.pose.pose.position.x = 123.0;  // 使わない
  m.twist.twist.linear.x = 1.2;
  m.twist.twist.linear.y = 0.1;
  m.twist.twist.angular.z = -0.2;
  const auto s = toCore(m);
  EXPECT_NEAR(s.v, 1.2, 1e-12);
  EXPECT_NEAR(s.v_lat, 0.1, 1e-12);
  ASSERT_TRUE(s.yaw_rate.has_value());
  EXPECT_NEAR(*s.yaw_rate, -0.2, 1e-12);
}

TEST(Conversions, Covariance6RoundTrip) {
  gll::Mat3 c;
  c << 1, 0.1, 0.2, 0.1, 2, 0.3, 0.2, 0.3, 3;
  const auto c6 = toCovariance6(c, 1e4);
  EXPECT_DOUBLE_EQ(c6[35], 3.0);   // yaw-yaw
  EXPECT_DOUBLE_EQ(c6[5], 0.2);    // x-yaw
  EXPECT_DOUBLE_EQ(c6[14], 1e4);   // z-z
  EXPECT_LT((fromCovariance6(c6) - c).norm(), 1e-15);
}

namespace {

/// x, y, z（FLOAT32）と、指定した時刻のフィールドを持つ PointCloud2 を作る。
sensor_msgs::msg::PointCloud2 makeCloud(const std::string& time_name, uint8_t time_type,
                                        const std::vector<std::array<double, 4>>& pts, double stamp) {
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp = toStamp(stamp);
  m.height = 1;
  m.width = static_cast<uint32_t>(pts.size());
  uint32_t off = 0;
  for (const char* n : {"x", "y", "z"}) {
    PointField f;
    f.name = n;
    f.offset = off;
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    m.fields.push_back(f);
    off += 4;
  }
  uint32_t tsize = 0;
  if (!time_name.empty()) {
    PointField f;
    f.name = time_name;
    f.offset = off;
    f.datatype = time_type;
    f.count = 1;
    m.fields.push_back(f);
    tsize = time_type == PointField::FLOAT64 ? 8 : 4;
  }
  m.point_step = off + tsize;
  m.row_step = m.point_step * m.width;
  m.data.resize(m.row_step);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    uint8_t* p = &m.data[i * m.point_step];
    for (int k = 0; k < 3; ++k) {
      const float v = static_cast<float>(pts[i][k]);
      std::memcpy(p + 4 * k, &v, 4);
    }
    if (time_type == PointField::FLOAT32) {
      const float v = static_cast<float>(pts[i][3]);
      std::memcpy(p + off, &v, 4);
    } else if (time_type == PointField::FLOAT64) {
      std::memcpy(p + off, &pts[i][3], 8);
    } else if (time_type == PointField::UINT32) {
      const uint32_t v = static_cast<uint32_t>(pts[i][3]);
      std::memcpy(p + off, &v, 4);
    }
  }
  return m;
}

}  // namespace

TEST(Conversions, PointCloudRelativeSecondsFloat32) {
  // Velodyne 型: ヘッダの時刻からの秒（FLOAT32）。スキャンの最後の点の時刻が t になる
  const auto m = makeCloud("time", sensor_msgs::msg::PointField::FLOAT32,
                           {{1, 2, 3, 0.0}, {4, 5, 6, 0.05}, {7, 8, 9, 0.1}}, 100.0);
  std::string field;
  const auto s = toCore(m, PointCloudOptions(), &field);
  EXPECT_EQ(field, "time");
  ASSERT_EQ(s.points.size(), 3u);
  EXPECT_NEAR(s.t, 100.1, 1e-6);
  ASSERT_EQ(s.times.size(), 3u);
  EXPECT_NEAR(s.times[0], -0.1, 1e-6);
  EXPECT_NEAR(s.times[2], 0.0, 1e-6);
  EXPECT_FLOAT_EQ(s.points[1].y(), 5.0f);
}

TEST(Conversions, PointCloudNanosecondsUint32AndNan) {
  // Ouster 型: ヘッダの時刻からの ns（UINT32）。NaN の点は捨てる
  const auto m = makeCloud("t", sensor_msgs::msg::PointField::UINT32,
                           {{1, 0, 0, 0}, {std::nan(""), 0, 0, 5e7}, {2, 0, 0, 1e8}}, 50.0);
  const auto s = toCore(m, PointCloudOptions());
  ASSERT_EQ(s.points.size(), 2u);
  EXPECT_NEAR(s.t, 50.1, 1e-6);
  EXPECT_NEAR(s.times[0], -0.1, 1e-6);
}

TEST(Conversions, PointCloudAbsoluteSecondsFloat64) {
  // Hesai 型: 絶対時刻の秒（FLOAT64）
  const auto m = makeCloud("timestamp", sensor_msgs::msg::PointField::FLOAT64,
                           {{1, 0, 0, 1727400000.00}, {2, 0, 0, 1727400000.08}}, 1727400000.0);
  const auto s = toCore(m, PointCloudOptions());
  EXPECT_NEAR(s.t, 1727400000.08, 1e-6);
  EXPECT_NEAR(s.times[0], -0.08, 1e-5);
}

TEST(Conversions, PointCloudLivoxMid360) {
  // livox_ros_driver2（xfer_format: 0）の PointCloud2: x, y, z, intensity（FLOAT32）、tag, line（UINT8）、
  // timestamp（FLOAT64、絶対時刻の ns）。詰めて並ぶので timestamp は 8 バイト境界にない（offset 18、point_step 26）
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 m;
  const double t0 = 1727400000.0;
  m.header.stamp = toStamp(t0);
  m.height = 1;
  m.width = 3;
  const std::vector<std::tuple<std::string, uint32_t, uint8_t>> fields = {
      {"x", 0, PointField::FLOAT32},     {"y", 4, PointField::FLOAT32},   {"z", 8, PointField::FLOAT32},
      {"intensity", 12, PointField::FLOAT32}, {"tag", 16, PointField::UINT8}, {"line", 17, PointField::UINT8},
      {"timestamp", 18, PointField::FLOAT64}};
  for (const auto& [name, off, type] : fields) {
    PointField f;
    f.name = name;
    f.offset = off;
    f.datatype = type;
    f.count = 1;
    m.fields.push_back(f);
  }
  m.point_step = 26;
  m.row_step = m.point_step * m.width;
  m.data.assign(m.row_step, 0);
  const double ns0 = t0 * 1e9;
  const double dts[3] = {0.0, 0.04, 0.1};
  for (uint32_t i = 0; i < m.width; ++i) {
    uint8_t* p = &m.data[i * m.point_step];
    const float xyz[3] = {1.0f + i, 2.0f, 3.0f};
    std::memcpy(p, xyz, 12);
    const double ts = ns0 + dts[i] * 1e9;
    std::memcpy(p + 18, &ts, 8);
  }
  std::string field;
  const auto s = toCore(m, PointCloudOptions(), &field);
  EXPECT_EQ(field, "timestamp");
  ASSERT_EQ(s.points.size(), 3u);
  EXPECT_NEAR(s.t, t0 + 0.1, 1e-6);
  ASSERT_EQ(s.times.size(), 3u);
  EXPECT_NEAR(s.times[0], -0.1, 1e-5);
  EXPECT_NEAR(s.times[1], -0.06, 1e-5);
  EXPECT_FLOAT_EQ(s.points[2].x(), 3.0f);
}

TEST(Conversions, PointCloudWithoutTimeField) {
  const auto m = makeCloud("", 0, {{1, 0, 0, 0}, {2, 0, 0, 0}}, 10.0);
  PointCloudOptions opt;
  opt.stamp_offset = 0.1;  // ヘッダの時刻がスキャン開始なら、スキャン終了に合わせるなど
  std::string field = "x";
  const auto s = toCore(m, opt, &field);
  EXPECT_TRUE(field.empty());
  EXPECT_TRUE(s.times.empty());
  EXPECT_NEAR(s.t, 10.1, 1e-9);
  ASSERT_EQ(s.points.size(), 2u);
}

TEST(Conversions, PointCloudRoundTrip) {
  const std::vector<gll::Vec3f> pts = {gll::Vec3f(1, 2, 3), gll::Vec3f(-4, 5.5f, 6)};
  const auto m = toPointCloud2(pts, "map_local", toStamp(3.0));
  EXPECT_EQ(m.header.frame_id, "map_local");
  const auto s = toCore(m, PointCloudOptions());
  ASSERT_EQ(s.points.size(), 2u);
  EXPECT_FLOAT_EQ(s.points[1].y(), 5.5f);
}
