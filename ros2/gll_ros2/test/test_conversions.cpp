#include "gll_ros2/conversions.hpp"

#include <gtest/gtest.h>

using namespace gll_ros2;

TEST(Conversions, StampRoundTrip) {
  const double t = 1727400000.123456789;
  EXPECT_NEAR(toSec(toStamp(t)), t, 1e-6);
}

TEST(Conversions, YawQuaternionRoundTrip) {
  for (double yaw : {-3.0, -1.0, 0.0, 0.5, 2.9}) EXPECT_NEAR(quaternionToYaw(yawToQuaternion(yaw)), yaw, 1e-12);
}

TEST(Conversions, ImuFrdToFlu) {
  // 前・右・下（FRD）の IMU を roll = 180° で base_link（FLU）に回す（i2Nav-Robot の ADIS16465）
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
