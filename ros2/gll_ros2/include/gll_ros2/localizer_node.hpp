// LocalizerNode: ROS 2 のインターフェース層（設計書 7.5 節）。
#pragma once

#include <gll/localizer.hpp>

#include "gll_ros2/conversions.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fstream>
#include <memory>
#include <optional>
#include <string>

namespace gll_ros2 {

class RosLogger : public gll::ILogger {
 public:
  explicit RosLogger(rclcpp::Logger logger) : logger_(logger) {}
  void debug(const std::string& m) override { RCLCPP_DEBUG(logger_, "%s", m.c_str()); }
  void info(const std::string& m) override { RCLCPP_INFO(logger_, "%s", m.c_str()); }
  void warn(const std::string& m) override { RCLCPP_WARN(logger_, "%s", m.c_str()); }
  void error(const std::string& m) override { RCLCPP_ERROR(logger_, "%s", m.c_str()); }

 private:
  rclcpp::Logger logger_;
};

class LocalizerNode : public rclcpp::Node {
 public:
  explicit LocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LocalizerNode() override;

 private:
  gll::LocalizerConfig loadConfig();
  /// maps.yaml を読んで地図とスキャンマッチャを Localizer に渡す（map.config_path が空なら何もしない）。
  void setupMap(const gll::LocalizerConfig& cfg);
  void onOutputTimer();
  void onDiagnosticsTimer();
  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void publishStatus(const gll::LocalizationOutput* out);
  void publishLidarDebug();
  void publishMapPoints();
  void savePose(const gll::LocalizationOutput& out);
  void writeCsv(const gll::LocalizationOutput& out);

  std::unique_ptr<gll::Localizer> localizer_;
  gll::LocalizerConfig cfg_;
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  double imu_acc_scale_ = 1.0;
  double gnss_stamp_offset_ = 0.0;
  PointCloudOptions points_opt_;
  std::string map_frame_;
  std::string base_frame_;
  std::string local_frame_;
  bool publish_tf_ = true;
  bool publish_height_ = true;  ///< 出力の z に推定した base_link の楕円体高を入れる（無ければ 0）
  double other_var_ = 1e4;
  std::optional<gll::Vec2> local_origin_;  ///< map → map_local のオフセット（UTM）

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr raw_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  std::ofstream csv_;
  gll::LocalizationStatus last_status_ = gll::LocalizationStatus::INITIALIZING;
  std::optional<gll::LocalizationOutput> last_out_;
  double dr_error_distance_ = 30.0;
  bool dr_error_active_ = false;
  bool lidar_enabled_ = false;
  bool gnss_enabled_ = true;  ///< anchor: local の地図では GNSS を使わない
  bool points_time_logged_ = false;
  double last_lidar_debug_t_ = -1.0;
  const gll::MatchTarget* last_published_target_ = nullptr;
  std::size_t map_points_max_ = 200000;
  double scan_points_voxel_ = 0.2;
  std::string registration_ = "gicp";
  // 前回位置の保存（設計書 3.11 節）
  std::string saved_pose_path_;
  double save_interval_ = 1.0;
  double last_save_t_ = -1e18;
};

}  // namespace gll_ros2
