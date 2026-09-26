// 設定（既定値は設計書 8 章）。読み込みは IF 層の責務。
#pragma once

#include "gll/common/types.hpp"

namespace gll {

struct EstimatorConfig {
  double sigma_v = 0.05;          ///< ODOM 前進速度のノイズ [m/s]
  double sigma_v_lat = 0.02;      ///< 横すべり [m/s]
  double sigma_omega = 0.01;      ///< ヨーレートのノイズ [rad/s]
  double sigma_bias_rw = 1e-4;    ///< ジャイロバイアスのランダムウォーク [rad/s/√s]
  double sigma_scale_rw = 1e-4;   ///< ODOM スケールのランダムウォーク [1/√s]
  bool estimate_odom_scale = true;
  double history_length = 2.0;    ///< 状態履歴の長さ [s]
  double gate_alpha = 0.001;      ///< Mahalanobis ゲートの有意水準
};

struct GnssConfig {
  int utm_zone = 54;
  bool utm_north = true;
  int rtk_fix_status = 2;             ///< RTK-FIX を表す NavSatStatus.status の値
  double max_stddev = 0.05;           ///< 採用する水平 σ の上限 [m]
  double min_stddev = 0.02;           ///< 観測共分散の下限 [m]
  double fix_settle_time = 1.0;       ///< FIX 後の安定待ち [s]
  bool accept_unknown_covariance = false;
  double default_stddev = 0.03;       ///< 共分散が UNKNOWN のときに使う σ [m]
  Vec3 lever_arm = Vec3::Zero();      ///< base_link から見たアンテナ位置 [m]（FLU）
  double attitude_stddev = deg2rad(0.5);  ///< roll / pitch の誤差の想定値 [rad]
  bool use_velocity = false;
  double cog_min_speed = 0.5;         ///< [m/s]
  double cog_max_yaw_rate = deg2rad(5.0);  ///< [rad/s]
};

struct AttitudeConfig {
  double kp = 1.0;                  ///< Mahony の比例ゲイン
  double ki = 0.0;                  ///< Mahony の積分ゲイン
  double static_init_time = 3.0;    ///< 起動時の静止初期化の時間 [s]
  double accel_tolerance = 1.0;     ///< |‖g‖ - g| がこれ以上なら補正ゲインを 0 にする [m/s^2]
  double history_length = 3.0;      ///< roll / pitch の履歴の長さ [s]
};

struct MotionConfig {
  double odom_hold_max = 0.1;  ///< ODOM の外挿の上限 [s]
  double imu_timeout = 0.1;    ///< これを超えて IMU が来なければ ODOM のヨーレートで代替 [s]
};

struct StopConfig {
  double v_threshold = 0.01;       ///< [m/s]
  double w_threshold = 0.02;       ///< [rad/s]
  double min_duration = 1.0;       ///< [s]
  double zaru_stddev = 0.002;      ///< ZARU の観測ノイズ [rad/s]
};

struct InitConfig {
  double heading_min_distance = 1.0;          ///< [m]
  double init_yaw_stddev = deg2rad(15.0);     ///< [rad]
  double ready_yaw_stddev = deg2rad(2.0);     ///< [rad]
  double ready_pos_stddev = 0.1;              ///< [m]
  double init_bias_stddev = 0.005;            ///< [rad/s]
  double init_scale_stddev = 0.03;
};

struct OutputConfig {
  double max_rate_xy = 0.1;                   ///< [m/s]
  double max_rate_yaw = deg2rad(2.0);         ///< [rad/s]
  double offset_error_xy = 1.0;               ///< [m]
  double offset_error_yaw = deg2rad(5.0);     ///< [rad]
};

struct MonitorConfig {
  double aid_timeout = 1.0;    ///< [s]
  double dr_max_stddev = 0.3;  ///< [m]
  double lost_stddev = 1.0;    ///< [m]
};

struct ArbiterConfig {
  double lidar_cov_inflation_under_fix = 4.0;
  double consistency_xy = 0.15;               ///< [m]
  double consistency_yaw = deg2rad(1.0);      ///< [rad]
};

struct RecoveryConfig {
  int reanchor_confirm_gnss = 3;
  int reanchor_confirm_lidar = 5;
  double reanchor_consistency_xy = 0.10;      ///< [m]
  double reanchor_consistency_yaw = deg2rad(1.0);
  double reanchor_extra_stddev_xy = 0.05;     ///< Σ_reanchor の位置成分 [m]
  double reanchor_extra_stddev_yaw = deg2rad(2.0);
  double candidate_max_age = 5.0;             ///< 候補を保持する時間 [s]
};

struct LocalizerConfig {
  EstimatorConfig estimator;
  GnssConfig gnss;
  AttitudeConfig attitude;
  MotionConfig motion;
  StopConfig stop;
  InitConfig init;
  OutputConfig output;
  MonitorConfig monitor;
  ArbiterConfig arbiter;
  RecoveryConfig recovery;
};

}  // namespace gll
