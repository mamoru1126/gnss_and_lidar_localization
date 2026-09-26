// ROS メッセージ ⇔ コアの型の変換（IF 層。ソフトウェア構成 3.7 節）。
#pragma once

#include <gll/common/types.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <array>
#include <string>

namespace gll_ros2 {

double toSec(const builtin_interfaces::msg::Time& t);
builtin_interfaces::msg::Time toStamp(double sec);

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);

/// roll / pitch / yaw [rad]（Z-Y-X）から回転行列を作る（IMU の取り付け回転用）。
Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw);

/// IMU を base_link に回転してコアの型にする。R_base_imu は IMU 座標系 → base_link の回転。
gll::ImuSample toCore(const sensor_msgs::msg::Imu& m, const Eigen::Matrix3d& R_base_imu);

/// nav_msgs/Odometry の twist だけを使う（設計書 7.5 節）。
gll::OdomSample toCore(const nav_msgs::msg::Odometry& m);

/// NavSatFix。stamp_offset を観測時刻に加える（受信時刻が入っている場合の遅延補正）。
gll::GnssSample toCore(const sensor_msgs::msg::NavSatFix& m, double stamp_offset);

/// GNSS の速度（twist.linear.x = 東、y = 北）。
gll::GnssVelocitySample toCore(const geometry_msgs::msg::TwistWithCovarianceStamped& m, double stamp_offset);

/// 6×6 の共分散（x, y, z, roll, pitch, yaw）に (x, y, yaw) の 3×3 を入れる。z / roll / pitch は other_var。
std::array<double, 36> toCovariance6(const gll::Mat3& cov_xy_yaw, double other_var);

/// 6×6 の共分散から (x, y, yaw) の 3×3 を取り出す。
gll::Mat3 fromCovariance6(const std::array<double, 36>& c);

}  // namespace gll_ros2
