#include "gll_ros2/localizer_node.hpp"

#include "gll_ros2/diagnostics.hpp"

#include <gll/common/pose_store.hpp>
#include <gll/estimation/es_ekf_2d.hpp>
#include <gll/estimation/inv_ekf_se2.hpp>
#include <gll/map/map_config.hpp>
#include <gll/map/map_tile_manager.hpp>
#include <gll/matching/gicp_matcher.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
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
  auto vec3 = [this](const std::string& name, const gll::Vec3& def) {
    const auto v = declare_parameter<std::vector<double>>(name, {def.x(), def.y(), def.z()});
    if (v.size() != 3) throw std::invalid_argument("parameter " + name + " must have 3 elements");
    return gll::Vec3(v[0], v[1], v[2]);
  };

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
  c.gnss.lever_arm = vec3("gnss.lever_arm", c.gnss.lever_arm);
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
  // arbiter（GNSS FIX 中の LiDAR。設計書 3.13.2 節）
  c.arbiter.lidar_cov_inflation_under_fix =
      d("arbiter.lidar_cov_inflation_under_fix", c.arbiter.lidar_cov_inflation_under_fix);
  c.arbiter.consistency_xy = d("arbiter.consistency_xy", c.arbiter.consistency_xy);
  c.arbiter.consistency_yaw = deg("arbiter.consistency_yaw_deg", c.arbiter.consistency_yaw);
  c.arbiter.anchor_mismatch_warn_xy = d("arbiter.anchor_mismatch_warn_xy", c.arbiter.anchor_mismatch_warn_xy);
  c.arbiter.anchor_mismatch_warn_yaw = deg("arbiter.anchor_mismatch_warn_yaw_deg", c.arbiter.anchor_mismatch_warn_yaw);
  // recovery
  c.recovery.reanchor_confirm_gnss = i("recovery.reanchor_confirm_gnss", c.recovery.reanchor_confirm_gnss);
  c.recovery.reanchor_confirm_lidar = i("recovery.reanchor_confirm_lidar", c.recovery.reanchor_confirm_lidar);
  c.recovery.reanchor_consistency_xy = d("recovery.reanchor_consistency_xy", c.recovery.reanchor_consistency_xy);
  c.recovery.reanchor_consistency_yaw =
      deg("recovery.reanchor_consistency_yaw_deg", c.recovery.reanchor_consistency_yaw);
  c.recovery.candidate_max_age = d("recovery.candidate_max_age", c.recovery.candidate_max_age);
  // lidar（設計書 6 章）
  const gll::Vec3 lxyz = vec3("lidar.extrinsic_xyz", gll::Vec3::Zero());
  const gll::Vec3 lrpy = vec3("lidar.extrinsic_rpy_deg", gll::Vec3::Zero());
  c.lidar.T_base_lidar = Eigen::Isometry3d::Identity();
  c.lidar.T_base_lidar.linear() = rpyToMatrix(gll::deg2rad(lrpy.x()), gll::deg2rad(lrpy.y()), gll::deg2rad(lrpy.z()));
  c.lidar.T_base_lidar.translation() = lxyz;
  points_opt_.time_field = declare_parameter<std::string>("lidar.time_field", "auto");
  points_opt_.stamp_offset = d("lidar.stamp_offset", 0.0);
  c.lidar.min_range = d("lidar.min_range", c.lidar.min_range);
  c.lidar.max_range = d("lidar.max_range", c.lidar.max_range);
  c.lidar.crop_box_enabled = b("lidar.crop_box_enabled", c.lidar.crop_box_enabled);
  c.lidar.crop_box_min = vec3("lidar.crop_box_min", c.lidar.crop_box_min);
  c.lidar.crop_box_max = vec3("lidar.crop_box_max", c.lidar.crop_box_max);
  c.lidar.deskew = b("lidar.deskew", c.lidar.deskew);
  c.lidar.source_voxel_size = d("lidar.source_voxel_size", c.lidar.source_voxel_size);
  c.lidar.source_num_neighbors = i("lidar.source_num_neighbors", c.lidar.source_num_neighbors);
  c.lidar.min_source_points = i("lidar.min_source_points", c.lidar.min_source_points);
  c.lidar.max_correspondence_distance = d("lidar.max_correspondence_distance", c.lidar.max_correspondence_distance);
  c.lidar.max_iterations = i("lidar.max_iterations", c.lidar.max_iterations);
  c.lidar.num_threads = i("lidar.num_threads", 4);
  c.lidar.min_inlier_ratio = d("lidar.min_inlier_ratio", c.lidar.min_inlier_ratio);
  c.lidar.overlap_distance = d("lidar.overlap_distance", c.lidar.overlap_distance);
  c.lidar.min_overlap = d("lidar.min_overlap", c.lidar.min_overlap);
  c.lidar.max_jump_xy = d("lidar.max_jump_xy", c.lidar.max_jump_xy);
  c.lidar.max_jump_yaw = deg("lidar.max_jump_yaw_deg", c.lidar.max_jump_yaw);
  c.lidar.cov_scale = d("lidar.cov_scale", c.lidar.cov_scale);
  c.lidar.min_stddev_xy = d("lidar.min_stddev_xy", c.lidar.min_stddev_xy);
  c.lidar.min_stddev_yaw = deg("lidar.min_stddev_yaw_deg", c.lidar.min_stddev_yaw);
  c.lidar.min_interval = d("lidar.min_interval", c.lidar.min_interval);
  c.lidar.base_link_height = d("lidar.base_link_height", c.lidar.base_link_height);
  c.lidar.async = true;
  // map（設計書 5 章）
  c.map.load_radius = d("map.load_radius", c.map.load_radius);
  c.map.unload_radius = d("map.unload_radius", c.map.unload_radius);
  c.map.lookahead_time = d("map.lookahead_time", c.map.lookahead_time);
  c.map.update_distance = d("map.update_distance", c.map.update_distance);
  c.map.update_interval = d("map.update_interval", c.map.update_interval);
  c.map.group_switch_margin = d("map.group_switch_margin", c.map.group_switch_margin);
  c.map.min_target_points = i("map.min_target_points", c.map.min_target_points);
  c.map.async = true;
  // relocalize（地図上での初期化と再位置推定。設計書 3.11 節・3.13.4 節）
  auto& r = c.relocalize;
  r.init_min_radius = d("relocalize.init_min_radius", r.init_min_radius);
  r.init_max_radius = d("relocalize.init_max_radius", r.init_max_radius);
  r.init_min_yaw = deg("relocalize.init_min_yaw_deg", r.init_min_yaw);
  r.init_max_yaw = deg("relocalize.init_max_yaw_deg", r.init_max_yaw);
  r.position_step = d("relocalize.position_step", r.position_step);
  r.yaw_step = deg("relocalize.yaw_step_deg", r.yaw_step);
  r.coarse_voxel_size = d("relocalize.coarse_voxel_size", r.coarse_voxel_size);
  r.coarse_source_voxel = d("relocalize.coarse_source_voxel", r.coarse_source_voxel);
  r.coarse_max_iterations = i("relocalize.coarse_max_iterations", r.coarse_max_iterations);
  r.refine_candidates = i("relocalize.refine_candidates", r.refine_candidates);
  r.distinct_xy = d("relocalize.distinct_xy", r.distinct_xy);
  r.distinct_yaw = deg("relocalize.distinct_yaw_deg", r.distinct_yaw);
  r.uniqueness_ratio = d("relocalize.uniqueness_ratio", r.uniqueness_ratio);
  r.min_overlap = d("relocalize.min_overlap", r.min_overlap);
  r.init_max_attempts = i("relocalize.init_max_attempts", r.init_max_attempts);
  r.init_map_distance = d("relocalize.init_map_distance", r.init_map_distance);
  r.init_timeout = d("relocalize.init_timeout", r.init_timeout);
  r.after_rejects = i("relocalize.after_rejects", r.after_rejects);
  r.min_radius = d("relocalize.min_radius", r.min_radius);
  r.radius_per_dr_distance = d("relocalize.radius_per_dr_distance", r.radius_per_dr_distance);
  r.max_radius = d("relocalize.max_radius", r.max_radius);
  r.max_yaw = deg("relocalize.max_yaw_deg", r.max_yaw);
  r.max_attempts = i("relocalize.max_attempts", r.max_attempts);
  r.lost_retry_interval = d("relocalize.lost_retry_interval", r.lost_retry_interval);
  return c;
}

void LocalizerNode::setupMap(const gll::LocalizerConfig& cfg) {
  const std::string path = declare_parameter<std::string>("map.config_path", "");
  map_points_max_ = static_cast<std::size_t>(declare_parameter<int64_t>("map.publish_points_max", 200000));
  if (path.empty()) {
    RCLCPP_INFO(get_logger(), "map.config_path is empty: LiDAR localization is disabled (GNSS + dead reckoning)");
    return;
  }
  const gll::MapSetConfig ms = gll::loadMapSetConfig(path);
  if ((ms.utm_zone && *ms.utm_zone != cfg.gnss.utm_zone) || (ms.utm_north && *ms.utm_north != cfg.gnss.utm_north))
    throw std::runtime_error("UTM zone in " + path + " does not match gnss.utm_zone / gnss.utm_north");
  auto logger = std::make_shared<RosLogger>(get_logger());
  const gll::UtmProjector utm(cfg.gnss.utm_zone, cfg.gnss.utm_north);
  std::vector<gll::MapGroup> groups = gll::loadMapGroups(ms, utm);
  std::size_t tiles = 0;
  for (const auto& g : groups) {
    tiles += g.index.tiles.size();
    const gll::Vec3 a = g.anchor.utmPoint();
    RCLCPP_INFO(get_logger(), "map group %s: %zu tiles, anchor (%.2f, %.2f), rotation %.3f deg, scale %.7f",
                g.id.c_str(), g.index.tiles.size(), a.x(), a.y(), gll::rad2deg(g.anchor.rotation()), g.anchor.scale());
  }
  if (!local_origin_ && !groups.empty()) {
    // 可視化用の map_local の原点: 最初のグループのアンカーを 100 m 単位に丸めた点
    const gll::Vec3 a = groups.front().anchor.utmPoint();
    local_origin_ = gll::Vec2(std::round(a.x() / 100.0) * 100.0, std::round(a.y() / 100.0) * 100.0);
  }
  auto matcher = std::make_shared<gll::GicpMatcher>(cfg.lidar, cfg.relocalize);
  auto maps = std::make_shared<gll::MapTileManager>(cfg.map, std::move(groups),
                                                    std::make_shared<gll::BinaryTileLoader>(), matcher, logger);
  localizer_->setMap(maps, matcher);
  lidar_enabled_ = true;
  RCLCPP_INFO(get_logger(), "LiDAR localization enabled: %zu map groups, %zu tiles (%s)", ms.groups.size(), tiles,
              path.c_str());
}

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("gll_localizer", options) {
  cfg_ = loadConfig();
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  local_frame_ = map_frame_ + "_local";
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  const double rate = declare_parameter<double>("output_rate", 50.0);
  const std::string estimator = declare_parameter<std::string>("estimator.type", "invariant_ekf");
  const auto rpy = declare_parameter<std::vector<double>>("imu.rotation_rpy_deg", {0.0, 0.0, 0.0});
  if (rpy.size() == 3)
    R_base_imu_ = rpyToMatrix(gll::deg2rad(rpy[0]), gll::deg2rad(rpy[1]), gll::deg2rad(rpy[2]));
  const auto origin = declare_parameter<std::vector<double>>("map_local_origin", std::vector<double>{});
  if (origin.size() >= 2) local_origin_ = gll::Vec2(origin[0], origin[1]);
  const std::string csv_path = declare_parameter<std::string>("debug_csv_path", "");
  saved_pose_path_ = declare_parameter<std::string>("init.saved_pose_path", "");
  const bool use_saved_pose = declare_parameter<bool>("init.use_saved_pose", false);
  const double saved_sxy = declare_parameter<double>("init.saved_pose_stddev_xy", 0.5);
  const double saved_syaw = gll::deg2rad(declare_parameter<double>("init.saved_pose_stddev_yaw_deg", 10.0));
  save_interval_ = declare_parameter<double>("init.save_interval", 1.0);

  std::unique_ptr<gll::IStateEstimator> est;
  if (estimator == "esekf") {
    est = std::make_unique<gll::EsEkf2D>(cfg_.estimator);
  } else {
    est = std::make_unique<gll::InvEkfSe2>(cfg_.estimator);
  }
  localizer_ = std::make_unique<gll::Localizer>(cfg_, std::move(est), std::make_shared<RosLogger>(get_logger()));
  setupMap(cfg_);

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
  if (cfg_.gnss.use_velocity) {
    gnss_vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/input/gnss/velocity", sensor_qos,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {
          localizer_->addGnssVelocity(toCore(*m, gnss_stamp_offset_));
        });
  }
  if (lidar_enabled_) {
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/points", sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onPoints(m); });
  }
  init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/input/initial_pose", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        gll::Pose2D p{m->pose.pose.position.x, m->pose.pose.position.y, quaternionToYaw(m->pose.pose.orientation)};
        gll::Mat3 cov = fromCovariance6(m->pose.covariance);
        if (!(cov(0, 0) > 0.0) || !(cov(1, 1) > 0.0) || !(cov(2, 2) > 0.0)) {
          // 共分散が入っていなければ、1 m・30° とみなす
          cov = gll::Vec3(1.0, 1.0, std::pow(gll::deg2rad(30.0), 2)).asDiagonal();
        }
        localizer_->setInitialPose(toSec(m->header.stamp), p, cov);
        RCLCPP_INFO(get_logger(), "initial pose received: (%.3f, %.3f, %.2f deg)", p.x, p.y, gll::rad2deg(p.yaw));
      });

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/pose", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/output/odometry", 10);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/status", 10);
  raw_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/debug/raw_pose", 10);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  if (lidar_enabled_) {
    lidar_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/debug/lidar_pose", 10);
    map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/debug/map_points", rclcpp::QoS(1).transient_local().reliable());
  }
  if (publish_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 可視化用の map → map_local（UTM の大きな座標を避けるための固定オフセット）
  if (local_origin_) {
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = local_frame_;
    t.transform.translation.x = local_origin_->x();
    t.transform.translation.y = local_origin_->y();
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

  // 前回保存した位置から始める（地図の上なら、その周りで位置合わせしてから初期化する。設計書 3.11 節）
  if (use_saved_pose && !saved_pose_path_.empty()) {
    if (const auto sp = gll::loadPose(saved_pose_path_)) {
      const gll::Mat3 cov =
          gll::Vec3(saved_sxy * saved_sxy, saved_sxy * saved_sxy, saved_syaw * saved_syaw).asDiagonal();
      localizer_->setInitialPose(sp->t, sp->pose, cov, gll::Localizer::InitialPoseSource::SAVED);
      RCLCPP_INFO(get_logger(), "saved pose loaded from %s: (%.3f, %.3f, %.2f deg)", saved_pose_path_.c_str(),
                  sp->pose.x, sp->pose.y, gll::rad2deg(sp->pose.yaw));
    } else {
      RCLCPP_INFO(get_logger(), "no saved pose in %s", saved_pose_path_.c_str());
    }
  }

  output_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { onOutputTimer(); });
  diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { onDiagnosticsTimer(); });
  RCLCPP_INFO(get_logger(), "gll_localizer started (estimator: %s, UTM zone %d%s)", estimator.c_str(),
              cfg_.gnss.utm_zone, cfg_.gnss.utm_north ? "N" : "S");
}

LocalizerNode::~LocalizerNode() {
  // 終了時の位置を保存する（次の起動時の初期姿勢に使う）
  if (last_out_ && !saved_pose_path_.empty() && last_out_->status != gll::LocalizationStatus::INITIALIZING &&
      last_out_->status != gll::LocalizationStatus::LOST)
    gll::savePose(saved_pose_path_, gll::SavedPose{last_out_->t, last_out_->raw_pose});
  if (csv_.is_open()) csv_.close();
}

void LocalizerNode::onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  std::string field;
  gll::LidarScan scan = toCore(*msg, points_opt_, &field);
  if (!points_time_logged_) {
    points_time_logged_ = true;
    if (field.empty()) {
      RCLCPP_WARN(get_logger(), "point cloud has no per-point time field: deskew is disabled");
    } else {
      RCLCPP_INFO(get_logger(), "per-point time field: %s (deskew %s)", field.c_str(),
                  cfg_.lidar.deskew ? "enabled" : "disabled");
    }
  }
  localizer_->addLidarScan(std::move(scan));
}

void LocalizerNode::onOutputTimer() {
  const auto out = localizer_->getOutput();
  if (lidar_enabled_) {
    publishLidarDebug();
    publishMapPoints();
  }
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
  savePose(*out);

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

void LocalizerNode::publishLidarDebug() {
  const auto info = localizer_->lastLidarMatch();
  if (!info || info->t <= last_lidar_debug_t_) return;
  last_lidar_debug_t_ = info->t;
  geometry_msgs::msg::PoseWithCovarianceStamped m;
  m.header.stamp = toStamp(info->t);
  m.header.frame_id = map_frame_;
  m.pose.pose.position.x = info->pose.x;
  m.pose.pose.position.y = info->pose.y;
  m.pose.pose.orientation = yawToQuaternion(info->pose.yaw);
  gll::Mat3 T = gll::Mat3::Identity();
  T.topLeftCorner<2, 2>() = gll::SE2::rot(info->pose.yaw);
  m.pose.covariance = toCovariance6(T * info->cov_body * T.transpose(), other_var_);
  lidar_pose_pub_->publish(m);
}

void LocalizerNode::publishMapPoints() {
  const auto target = localizer_->currentMapTarget();
  if (!target || target.get() == last_published_target_ || !local_origin_) return;
  last_published_target_ = target.get();
  const std::vector<gll::Vec3f> pts = target->samplePoints(map_points_max_);
  std::vector<gll::Vec3f> local;
  local.reserve(pts.size());
  for (const auto& p : pts) {
    const gll::Vec3 q = target->anchor.mapToUtm(p.cast<double>());
    local.emplace_back(static_cast<float>(q.x() - local_origin_->x()), static_cast<float>(q.y() - local_origin_->y()),
                       static_cast<float>(q.z()));
  }
  map_points_pub_->publish(toPointCloud2(local, local_frame_, now()));
}

void LocalizerNode::savePose(const gll::LocalizationOutput& out) {
  if (saved_pose_path_.empty() || out.status == gll::LocalizationStatus::LOST) return;
  if (out.t - last_save_t_ < save_interval_) return;
  last_save_t_ = out.t;
  if (!gll::savePose(saved_pose_path_, gll::SavedPose{out.t, out.raw_pose}))
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "cannot write saved pose to %s",
                         saved_pose_path_.c_str());
}

void LocalizerNode::publishStatus(const gll::LocalizationOutput* out) {
  status_pub_->publish(makeLocalizationStatus(out, localizer_->diagnostics(), dr_error_distance_));
}

void LocalizerNode::onDiagnosticsTimer() {
  const auto d = localizer_->diagnostics();
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  // 自己位置推定の状態（デッドレコニング距離の超過は ERROR。設計書 3.12 節）、入力・棄却のカウンタ、地図の状態
  arr.status.push_back(makeLocalizationStatus(last_out_ ? &*last_out_ : nullptr, d, dr_error_distance_));
  arr.status.push_back(makeCounterStatus(d));
  if (lidar_enabled_) arr.status.push_back(makeMapStatus(d, cfg_.arbiter));
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
