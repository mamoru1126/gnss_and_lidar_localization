// 設定（既定値は設計書 8 章）。読み込みは IF 層の責務。
#pragma once

#include "gll/common/types.hpp"
#include "tiled_pcd_map/map_manager_config.hpp"

#include <Eigen/Geometry>

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
  double settle_reset_gap = 3.0;      ///< GNSS のメッセージがこれ以上途切れたら安定待ちをやり直す [s]
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
  double lost_stddev = 1.0;    ///< 位置の標準偏差（いちばん大きい向き）がこれ以上なら LOST [m]
  /// 0 より大きければ、横方向（車の左右）の位置の標準偏差がこれ以上でも LOST にする [m]
  /// （Autoware の localization_error_monitor は横方向を別に見る）
  double lost_stddev_lateral = 0.0;
  /// 位置の観測（GNSS / LiDAR / 再アンカー / 初期姿勢）なしで走ったこの距離を超えたら、
  /// 診断でエラーを通知する [m]（設計書 3.12 節）
  double dr_error_distance = 30.0;
};

struct ArbiterConfig {
  double lidar_cov_inflation_under_fix = 4.0;
  double consistency_xy = 0.15;               ///< [m]
  double consistency_yaw = deg2rad(1.0);      ///< [rad]
  double anchor_mismatch_warn_xy = 0.10;      ///< アンカーずれの平均がこれを超えたら診断で WARN [m]
  double anchor_mismatch_warn_yaw = deg2rad(0.5);  ///< [rad]
  int anchor_mismatch_min_count = 20;         ///< WARN を判定するのに必要な件数
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

/// LiDAR の前処理と位置合わせ（設計書 6 章）。
struct LidarConfig {
  Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();  ///< LiDAR 座標系 → base_link
  double min_range = 1.0;            ///< LiDAR からの距離の下限 [m]
  double max_range = 50.0;           ///< LiDAR からの距離の上限 [m]
  bool crop_box_enabled = false;     ///< 車体の点を除く箱（base_link 座標系）を使うか
  Vec3 crop_box_min = Vec3(-1.0, -1.0, -1.0);
  Vec3 crop_box_max = Vec3(1.0, 1.0, 2.0);
  bool deskew = true;                ///< 点ごとの時刻があれば、回転と並進の歪みを補正する
  double source_voxel_size = 0.5;    ///< スキャンの間引き [m]
  int source_num_neighbors = 10;     ///< スキャンの点ごとの共分散に使う近傍点数
  int min_source_points = 100;       ///< 間引いた後の点数がこれ未満なら照合しない
  double max_correspondence_distance = 1.0;  ///< [m]
  /// 追跡の照合の方式: "gicp"（ターゲットは点と KdTree）か "vgicp"（ターゲットはボクセルごとのガウス分布。CPU で動く）
  std::string registration = "gicp";
  double vgicp_voxel_size = 1.0;     ///< VGICP のボクセル [m]
  int max_iterations = 20;
  double translation_eps = 1e-3;     ///< 収束判定 [m]
  double rotation_eps = 1e-3;        ///< 収束判定 [rad]
  int num_threads = 2;
  double min_inlier_ratio = 0.6;     ///< 対応点が見つかった点の割合の下限
  double overlap_distance = 0.3;     ///< overlap（位置合わせの良さ）を数える距離 [m]
  double min_overlap = 0.5;          ///< overlap_distance 以内に地図の点がある割合の下限
  /// 0 より大きければ、追跡の照合の overlap の分母を「この距離以内に地図の点がある構造の点」に絞る [m]。
  /// 地図に無い物（車・人・木など、地図から離れた点）で overlap が下がらないようにする
  /// （Autoware の NVTL が、近くにボクセルがある点だけで平均するのと同じ考え）。0 なら構造の点すべて
  double overlap_near_distance = 0.0;
  double max_jump_xy = 1.0;          ///< 初期値からの移動量の上限 [m]
  double max_jump_yaw = deg2rad(5.0);  ///< [rad]
  double cov_scale = 0.15;           ///< 観測共分散 Σ = cov_scale · N_inlier · H⁻¹（設計書 6.2 節）
  double min_stddev_xy = 0.02;       ///< 観測共分散の下限 Σ_floor [m]
  /// 観測共分散の下限を、車の前後（縦）と左右（横）で別に決める [m]。負なら min_stddev_xy を使う。
  /// 長い直線では縦が決まりにくく、H から求めた縦の共分散が小さすぎることがある
  double min_stddev_lon = -1.0;
  double min_stddev_lat = -1.0;
  double min_stddev_yaw = deg2rad(0.2);  ///< [rad]
  double min_interval = 0.0;         ///< 照合の最小間隔 [s]（0 ならすべてのスキャン）
  double base_link_height = 0.0;     ///< 地面から base_link までの高さ [m]（地図上の初期化で z を決めるのに使う）
  bool async = true;                 ///< 別スレッドで照合する（テストでは false にして同期で処理する）
};

// 地図タイルの管理の設定 MapManagerConfig は tiled_pcd_map の tiled_pcd_map/map_manager_config.hpp（設計書 5 章）。

/// 地図上での初期化と再位置推定（設計書 3.11 節・3.13.4 節）。
struct RelocalizeConfig {
  double init_min_radius = 0.5;      ///< 初期姿勢の周りを探す半径 [m]（初期姿勢の 3σ をこの範囲に収める）
  double init_max_radius = 5.0;
  double init_min_yaw = deg2rad(10.0);  ///< yaw の探索幅（±）[rad]
  double init_max_yaw = kPi;
  double position_step = 1.0;        ///< 初期値の候補を並べる格子の間隔 [m]
  double yaw_step = deg2rad(20.0);   ///< [rad]
  double coarse_voxel_size = 2.0;    ///< 粗い位置合わせ（VGICP）のボクセル [m]
  double coarse_source_voxel = 1.0;  ///< 粗い位置合わせに使うスキャンの間引き [m]
  int coarse_max_iterations = 10;
  int refine_candidates = 5;         ///< GICP で詰める候補の数
  double distinct_xy = 1.0;          ///< これ以上離れた解は別の解とみなす [m]
  double distinct_yaw = deg2rad(10.0);
  double uniqueness_ratio = 1.5;     ///< 最良の解の overlap が、次点のこの倍以上なら一意とみなす
  double min_overlap = 0.6;          ///< 採用する解の overlap の下限
  int init_max_attempts = 5;         ///< 地図上での初期化を試す回数（外部から与えた初期姿勢は、その後そのまま使う）
  double init_timeout = 15.0;        ///< 地図上での初期化がこの時間 [s] 決まらなければ、失敗が続いたときと同じに扱う
                                     ///< （LiDAR のデータが来ない、地図のターゲットができないなど）
  double init_map_distance = 10.0;   ///< 初期姿勢がタイルからこの距離以内なら、地図上で初期化する [m]
  int after_rejects = 20;            ///< 再位置推定を始める LiDAR の連続棄却数
  double min_radius = 1.0;           ///< 再位置推定の探索半径の下限 [m]
  double radius_per_dr_distance = 0.05;  ///< 探索半径の下限を、位置の観測なしで走った距離のこの割合まで広げる
  double max_radius = 3.0;           ///< 再位置推定の探索半径の上限 [m]
  double max_yaw = deg2rad(30.0);    ///< [rad]
  int max_attempts = 3;              ///< 再位置推定がこの回数失敗したら LOST（lost_on_failure のとき）
  /// false なら、再位置推定が続けて失敗しても LOST にしない（LOST は位置の標準偏差だけで決める）。
  /// 照合がまた after_rejects 回続けて捨てられたら、再位置推定をやり直す
  bool lost_on_failure = true;
  double lost_retry_interval = 5.0;  ///< LOST の間に再位置推定を試す間隔 [s]
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
  LidarConfig lidar;
  MapManagerConfig map;
  RelocalizeConfig relocalize;
};

}  // namespace gll
