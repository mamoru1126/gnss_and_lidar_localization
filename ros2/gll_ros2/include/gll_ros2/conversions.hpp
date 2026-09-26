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
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <array>
#include <string>
#include <vector>

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

/// PointCloud2 の読み方（設計書 6.1 節）。
struct PointCloudOptions {
  /// 点ごとの時刻のフィールド名。"auto" なら time / t / timestamp / time_stamp / offset_time の順に探す。"" なら使わない。
  std::string time_field = "auto";
  /// 点ごとの時刻が無いときにスキャンの時刻にする header.stamp からのずれ [s]（ドライバの仕様に合わせる）。
  double stamp_offset = 0.0;
};

/// PointCloud2 → LidarScan（LiDAR 座標系）。x / y / z は FLOAT32 か FLOAT64。
/// 点ごとの時刻があれば、スキャンの最後の点の時刻を t にし、各点の t からの相対時刻を times に入れる。
/// 時刻の単位は型から決める: 整数型はヘッダの時刻からの ns、FLOAT32 はヘッダの時刻からの秒、
/// FLOAT64 は値の大きさで、絶対時刻の ns（> 1e15）・絶対時刻の秒（> 1e6）・ヘッダの時刻からの秒を見分ける。
/// time_field_used に使ったフィールド名を返す（無ければ空）。
gll::LidarScan toCore(const sensor_msgs::msg::PointCloud2& m, const PointCloudOptions& opt,
                      std::string* time_field_used = nullptr);

/// 点群を PointCloud2（x, y, z の FLOAT32）にする（可視化用）。
sensor_msgs::msg::PointCloud2 toPointCloud2(const std::vector<gll::Vec3f>& points, const std::string& frame_id,
                                            const builtin_interfaces::msg::Time& stamp);

/// 6×6 の共分散（x, y, z, roll, pitch, yaw）に (x, y, yaw) の 3×3 を入れる。z / roll / pitch は other_var。
std::array<double, 36> toCovariance6(const gll::Mat3& cov_xy_yaw, double other_var);

/// 6×6 の共分散から (x, y, yaw) の 3×3 を取り出す。
gll::Mat3 fromCovariance6(const std::array<double, 36>& c);

}  // namespace gll_ros2
