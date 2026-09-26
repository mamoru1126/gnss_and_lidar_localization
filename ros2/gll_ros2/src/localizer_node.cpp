#include "gll_ros2/localizer_node.hpp"

#include "gll_ros2/conversions.hpp"
#include "gll_ros2/diagnostics.hpp"

#include <gll/estimation/es_ekf_2d.hpp>
#include <gll/estimation/inv_ekf_se2.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <vector>

namespace gll_ros2 {

gll::LocalizerConfig LocalizerNode::loadConfig() {
  gll::LocalizerConfig c;
  auto d = [this](const std::string& name, double def) { return declare_parameter<double>(name, def); };
  auto i = [this](const std::string& name, int def) {
    return static_cast<int>(declare_parameter<int64_t>(name, def));
  };
  auto b = [this](const std::string& name, bool def) { return declare_parameter<bool>(name, def); };
  auto deg = [&d](const std::string& name, double def_rad) { return gll::deg2rad(d(name, gll::rad2deg(def_rad))); };

  // estimator
  c.estimator.sigma_v = d("estimator.sigma_v", c.estimator.sigma_v);
  c.estimator.sigma_v_lat = d("estimator.sigma_v_lat", c.estimator.sigma_v_lat);
  c.estimator.sigma_omega = d("estimator.sigma_omega", c.estimator.sigma_omega);
  c.estimator.sigma_bias_rw = d("estimator.sigma_bias_rw", c.estimator.sigma_bias_rw);
  c.estimator.sigma_scale_rw = d("estimator.sigma_scale_rw", c.estimator.sigma_scale_rw);
  c.estimator.estimate_odom_scale = b("estimator.estimate_odom_scale", c.estimator.estimate_odom_scale);
  c.estimator.history_length = d("estimator.history_length", c.estimator.history_length);
  c.estimator.gate_alpha = d("estimator.gate_alpha", c.estimator.gate_alpha);
  // gnss
  c.gnss.utm_zone = i("gnss.utm_zone", c.gnss.utm_zone);
  c.gnss.utm_north = b("gnss.utm_north", c.gnss.utm_north);
  c.gnss.rtk_fix_status = i("gnss.rtk_fix_status", c.gnss.rtk_fix_status);
  c.gnss.max_stddev = d("gnss.max_stddev", c.gnss.max_stddev);
  c.gnss.min_stddev = d("gnss.min_stddev", c.gnss.min_stddev);
  c.gnss.fix_settle_time = d("gnss.fix_settle_time", c.gnss.fix_settle_time);
  c.gnss.settle_reset_gap = d("gnss.settle_reset_gap", c.gnss.settle_reset_gap);
  c.gnss.accept_unknown_covariance = b("gnss.accept_unknown_covariance", c.gnss.accept_unknown_covariance);
  c.gnss.default_stddev = d("gnss.default_stddev", c.gnss.default_stddev);
  const auto lever = declare_parameter<std::vector<double>>("gnss.lever_arm", {0.0, 0.0, 0.0});
  if (lever.size() == 3) c.gnss.lever_arm = gll::Vec3(lever[0], lever[1], lever[2]);
  c.gnss.attitude_stddev = deg("gnss.attitude_stddev_deg", c.gnss.attitude_stddev);
  c.gnss.use_velocity = b("gnss.use_velocity", c.gnss.use_velocity);
  c.gnss.cog_min_speed = d("gnss.cog_min_speed", c.gnss.cog_min_speed);
  c.gnss.cog_max_yaw_rate = deg("gnss.cog_max_yaw_rate_deg", c.gnss.cog_max_yaw_rate);
  gnss_stamp_offset_ = d("gnss.stamp_offset", 0.0);
  // attitude
  c.attitude.kp = d("attitude.kp", c.attitude.kp);
  c.attitude.ki = d("attitude.ki", c.attitude.ki);
  c.attitude.static_init_time = d("attitude.static_init_time", c.attitude.static_init_time);
  c.attitude.accel_tolerance = d("attitude.accel_tolerance", c.attitude.accel_tolerance);
  // motion / stop
  c.motion.odom_hold_max = d("motion.odom_hold_max", c.motion.odom_hold_max);
  c.motion.imu_timeout = d("motion.imu_timeout", c.motion.imu_timeout);
  c.stop.v_threshold = d("stop.v_threshold", c.stop.v_threshold);
  c.stop.w_threshold = d("stop.w_threshold", c.stop.w_threshold);
  c.stop.min_duration = d("stop.min_duration", c.stop.min_duration);
  c.stop.zaru_stddev = d("stop.zaru_stddev", c.stop.zaru_stddev);
  // init
  c.init.heading_min_distance = d("init.heading_min_distance", c.init.heading_min_distance);
  c.init.init_yaw_stddev = deg("init.init_yaw_stddev_deg", c.init.init_yaw_stddev);
  c.init.ready_yaw_stddev = deg("init.ready_yaw_stddev_deg", c.init.ready_yaw_stddev);
  c.init.ready_pos_stddev = d("init.ready_pos_stddev", c.init.ready_pos_stddev);
  // output
  c.output.max_rate_xy = d("output.max_rate_xy", c.output.max_rate_xy);
  c.output.max_rate_yaw = deg("output.max_rate_yaw_deg", c.output.max_rate_yaw);
  c.output.offset_error_xy = d("output.offset_error_xy", c.output.offset_error_xy);
  c.output.offset_error_yaw = deg("output.offset_error_yaw_deg", c.output.offset_error_yaw);
  // monitor
  c.monitor.aid_timeout = d("monitor.aid_timeout", c.monitor.aid_timeout);
  c.monitor.dr_max_stddev = d("monitor.dr_max_stddev", c.monitor.dr_max_stddev);
  c.monitor.lost_stddev = d("monitor.lost_stddev", c.monitor.lost_stddev);
  c.monitor.dr_error_distance = d("monitor.dr_error_distance", c.monitor.dr_error_distance);
  dr_error_distance_ = c.monitor.dr_error_distance;
  // recovery
  c.recovery.reanchor_confirm_gnss = i("recovery.reanchor_confirm_gnss", c.recovery.reanchor_confirm_gnss);
  c.recovery.reanchor_consistency_xy = d("recovery.reanchor_consistency_xy", c.recovery.reanchor_consistency_xy);
  c.recovery.candidate_max_age = d("recovery.candidate_max_age", c.recovery.candidate_max_age);
  return c;
}

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("gll_localizer", options) {
  const gll::LocalizerConfig cfg = loadConfig();
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  const double rate = declare_parameter<double>("output_rate", 50.0);
  const std::string estimator = declare_parameter<std::string>("estimator.type", "invariant_ekf");
  const auto rpy = declare_parameter<std::vector<double>>("imu.rotation_rpy_deg", {0.0, 0.0, 0.0});
  if (rpy.size() == 3)
    R_base_imu_ = rpyToMatrix(gll::deg2rad(rpy[0]), gll::deg2rad(rpy[1]), gll::deg2rad(rpy[2]));
  const auto origin = declare_parameter<std::vector<double>>("map_local_origin", std::vector<double>{});
  const std::string csv_path = declare_parameter<std::string>("debug_csv_path", "");

  std::unique_ptr<gll::IStateEstimator> est;
  if (estimator == "esekf") {
    est = std::make_unique<gll::EsEkf2D>(cfg.estimator);
  } else {
    est = std::make_unique<gll::InvEkfSe2>(cfg.estimator);
  }
  localizer_ = std::make_unique<gll::Localizer>(cfg, std::move(est), std::make_shared<RosLogger>(get_logger()));

  const auto sensor_qos = rclcpp::SensorDataQoS();
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "~/input/imu", sensor_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr m) { localizer_->addImu(toCore(*m, R_base_imu_)); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr m) { localizer_->addOdom(toCore(*m)); });
  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "~/input/gnss/fix", sensor_qos, [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        localizer_->addGnss(toCore(*m, gnss_stamp_offset_));
      });
  if (cfg.gnss.use_velocity) {
    gnss_vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/input/gnss/velocity", sensor_qos,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {
          localizer_->addGnssVelocity(toCore(*m, gnss_stamp_offset_));
        });
  }
  init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/input/initial_pose", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        gll::Pose2D p{m->pose.pose.position.x, m->pose.pose.position.y, quaternionToYaw(m->pose.pose.orientation)};
        localizer_->setInitialPose(toSec(m->header.stamp), p, fromCovariance6(m->pose.covariance));
        RCLCPP_INFO(get_logger(), "initial pose received: (%.3f, %.3f, %.2f deg)", p.x, p.y, gll::rad2deg(p.yaw));
      });

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/pose", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/output/odometry", 10);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/status", 10);
  raw_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/debug/raw_pose", 10);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  if (publish_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 可視化用の map → map_local（UTM の大きな座標を避けるための固定オフセット）
  if (origin.size() >= 2) {
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = map_frame_ + "_local";
    t.transform.translation.x = origin[0];
    t.transform.translation.y = origin[1];
    t.transform.rotation.w = 1.0;
    static_tf_->sendTransform(t);
  }

  if (!csv_path.empty()) {
    csv_.open(csv_path);
    if (csv_) {
      csv_ << "t,x,y,yaw,raw_x,raw_y,raw_yaw,var_x,var_y,var_yaw,raw_var_x,raw_var_y,raw_var_yaw,"
              "offset_x,offset_y,offset_yaw,status,recovery_state,active_map_group,roll,pitch,gyro_bias,odom_scale,"
              "dr_distance\n";
      csv_.precision(10);
    } else {
      RCLCPP_WARN(get_logger(), "cannot open debug_csv_path: %s", csv_path.c_str());
    }
  }

  output_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { onOutputTimer(); });
  diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { onDiagnosticsTimer(); });
  RCLCPP_INFO(get_logger(), "gll_localizer started (estimator: %s, UTM zone %d%s)", estimator.c_str(),
              cfg.gnss.utm_zone, cfg.gnss.utm_north ? "N" : "S");
}

LocalizerNode::~LocalizerNode() {
  if (csv_.is_open()) csv_.close();
}

void LocalizerNode::onOutputTimer() {
  const auto out = localizer_->getOutput();
  if (!out) {
    publishStatus(nullptr);
    return;
  }
  if (out->status != last_status_) {
    RCLCPP_INFO(get_logger(), "status: %s -> %s", gll::toString(last_status_), gll::toString(out->status));
    last_status_ = out->status;
  }
  // デッドレコニングが一定距離続いたらエラーを通知する（設計書 3.12 節）
  if (out->dr_distance_exceeded && !dr_error_active_) {
    RCLCPP_ERROR(get_logger(), "dead reckoning for %.1f m without GNSS / LiDAR position (limit %.1f m)",
                 out->dr_distance, dr_error_distance_);
    dr_error_active_ = true;
  } else if (!out->dr_distance_exceeded && dr_error_active_) {
    RCLCPP_INFO(get_logger(), "dead reckoning ended (position observation accepted)");
    dr_error_active_ = false;
  }
  last_out_ = *out;
  publishStatus(&*out);
  if (csv_.is_open()) writeCsv(*out);
  if (out->status == gll::LocalizationStatus::INITIALIZING) return;

  const auto stamp = toStamp(out->t);
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.pose.position.x = out->pose.x;
  pose.pose.pose.position.y = out->pose.y;
  pose.pose.pose.orientation = yawToQuaternion(out->pose.yaw);
  pose.pose.covariance = toCovariance6(out->cov, other_var_);
  pose_pub_->publish(pose);

  geometry_msgs::msg::PoseWithCovarianceStamped raw = pose;
  raw.pose.pose.position.x = out->raw_pose.x;
  raw.pose.pose.position.y = out->raw_pose.y;
  raw.pose.pose.orientation = yawToQuaternion(out->raw_pose.yaw);
  raw.pose.covariance = toCovariance6(out->raw_cov, other_var_);
  raw_pose_pub_->publish(raw);

  nav_msgs::msg::Odometry odom;
  odom.header = pose.header;
  odom.child_frame_id = base_frame_;
  odom.pose = pose.pose;
  odom.twist.twist.linear.x = out->v;
  odom.twist.twist.linear.y = out->v_lat;
  odom.twist.twist.angular.z = out->yaw_rate;
  odom_pub_->publish(odom);

  if (tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header = pose.header;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = out->pose.x;
    t.transform.translation.y = out->pose.y;
    t.transform.rotation = pose.pose.pose.orientation;
    tf_->sendTransform(t);
  }
}

void LocalizerNode::publishStatus(const gll::LocalizationOutput* out) {
  status_pub_->publish(makeLocalizationStatus(out, localizer_->diagnostics(), dr_error_distance_));
}

void LocalizerNode::onDiagnosticsTimer() {
  const auto d = localizer_->diagnostics();
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  // 自己位置推定の状態（デッドレコニング距離の超過は ERROR。設計書 3.12 節）と入力・棄却のカウンタ
  arr.status.push_back(makeLocalizationStatus(last_out_ ? &*last_out_ : nullptr, d, dr_error_distance_));
  arr.status.push_back(makeCounterStatus(d));
  diag_pub_->publish(arr);
}

void LocalizerNode::writeCsv(const gll::LocalizationOutput& o) {
  csv_ << o.t << ',' << o.pose.x << ',' << o.pose.y << ',' << o.pose.yaw << ',' << o.raw_pose.x << ','
       << o.raw_pose.y << ',' << o.raw_pose.yaw << ',' << o.cov(0, 0) << ',' << o.cov(1, 1) << ',' << o.cov(2, 2)
       << ',' << o.raw_cov(0, 0) << ',' << o.raw_cov(1, 1) << ',' << o.raw_cov(2, 2) << ',' << o.offset(0) << ','
       << o.offset(1) << ',' << o.offset(2) << ',' << gll::toString(o.status) << ',' << gll::toString(o.recovery)
       << ',' << o.active_map_group << ',' << o.roll << ',' << o.pitch << ',' << o.gyro_bias << ','
       << o.odom_scale << ',' << o.dr_distance << '\n';
}

}  // namespace gll_ros2
