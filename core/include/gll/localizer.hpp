// Localizer: 外部 API の窓口（ソフトウェア構成 3.2 節）。
// スレッド安全（フィルタの状態は内部の mutex で保護する）。時刻はすべてデータのタイムスタンプ [s]。
// LiDAR を使う場合は setMap() で地図とスキャンマッチャを渡す。lidar.async なら照合は専用のスレッドで行う。
#pragma once

#include "gll/common/config.hpp"
#include "tiled_pcd_map/logger.hpp"
#include "gll/estimation/initializer.hpp"
#include "gll/estimation/mahalanobis_gate.hpp"
#include "gll/estimation/output_smoother.hpp"
#include "gll/estimation/recovery_manager.hpp"
#include "gll/estimation/source_arbiter.hpp"
#include "gll/estimation/state_estimator.hpp"
#include "gll/estimation/state_history.hpp"
#include "gll/estimation/status_monitor.hpp"
#include "tiled_pcd_map/map_tile_manager.hpp"
#include "gll/matching/scan_matcher.hpp"
#include "gll/matching/scan_preprocessor.hpp"
#include "gll/measurement/attitude_estimator.hpp"
#include "gll/measurement/gnss_measurement_builder.hpp"
#include "gll/measurement/lidar_measurement_builder.hpp"
#include "gll/measurement/motion_input_builder.hpp"
#include "gll/measurement/stop_detector.hpp"

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace gll {

struct Diagnostics {
  bool attitude_initialized = false;
  bool filter_initialized = false;
  Initializer::Phase init_phase = Initializer::Phase::WAIT_FIX;
  std::size_t imu_count = 0;
  std::size_t odom_count = 0;
  std::size_t gnss_count = 0;
  std::size_t gnss_accepted = 0;
  std::size_t gnss_deferred = 0;
  std::size_t gnss_too_old = 0;
  std::size_t zaru_accepted = 0;
  std::size_t heading_accepted = 0;
  std::size_t reanchor_count = 0;
  std::size_t odom_stale_count = 0;
  std::size_t imu_fallback_count = 0;
  std::map<std::string, std::size_t> gnss_reject_reasons;
  double last_gnss_d2 = 0.0;
  // LiDAR（Phase 2）
  std::size_t lidar_count = 0;        ///< 受け取ったスキャン
  std::size_t lidar_dropped = 0;      ///< 処理が追いつかず捨てたスキャン
  std::size_t lidar_no_target = 0;    ///< 近くに地図が無く照合しなかった
  std::size_t lidar_few_points = 0;   ///< 点が少なく照合しなかった
  std::size_t lidar_matched = 0;      ///< 位置合わせを行った
  std::size_t lidar_accepted = 0;     ///< 観測として採用した
  std::size_t lidar_deferred = 0;     ///< ゲートに落ちて再アンカーの候補にした
  std::size_t lidar_mismatch = 0;     ///< GNSS FIX 中に食い違って棄却した
  std::size_t lidar_too_old = 0;
  std::size_t lidar_reanchor_count = 0;
  std::map<std::string, std::size_t> lidar_reject_reasons;  ///< 品質の条件で棄却した理由
  std::size_t relocalize_attempts = 0;
  std::size_t relocalize_success = 0;
  std::size_t map_init_attempts = 0;
  double last_match_ms = 0.0;
  double match_ms_sum = 0.0;          ///< 追跡の照合（前処理を含む）の時間の合計 [ms]（平均は ÷ lidar_matched）
  double match_ms_max = 0.0;
  double last_inlier_ratio = 0.0;
  double last_overlap = 0.0;
  MapTileManager::Stats map;
  std::map<std::string, MismatchStats> anchor_mismatch;  ///< 地図グループごとの GNSS とのずれ（世界座標系）
};

/// 直近の LiDAR の照合結果（デバッグ出力用）。
struct LidarMatchInfo {
  double t = 0.0;
  bool accepted = false;
  Pose2D pose;                  ///< 照合結果の base_link の姿勢（UTM）
  std::string group;
  std::string status;           ///< ACCEPTED / REJECTED の理由など
  double inlier_ratio = 0.0;
  double overlap = 0.0;
  double time_ms = 0.0;
  Mat3 cov_body = Mat3::Identity();
  /// 可視化用: 照合に使った点（base_link、前処理の後）と、照合結果の 3D の姿勢（UTM。base_link → UTM）
  std::shared_ptr<const std::vector<Vec3f>> points;
  Eigen::Isometry3d T_utm_base = Eigen::Isometry3d::Identity();
};

class Localizer {
 public:
  enum class InitialPoseSource { EXTERNAL, SAVED };

  explicit Localizer(const LocalizerConfig& cfg, std::unique_ptr<IStateEstimator> estimator = nullptr,
                     std::shared_ptr<ILogger> logger = nullptr);
  ~Localizer();
  Localizer(const Localizer&) = delete;
  Localizer& operator=(const Localizer&) = delete;

  /// 地図とスキャンマッチャを設定する（LiDAR を使う場合。起動時に 1 回）。maps は、領域として matcher のターゲットを
  /// 作るように targetBuilder(matcher) を渡して作ったもの（ほかの領域では照合しない）。
  void setMap(std::shared_ptr<MapTileManager> maps, std::shared_ptr<const IScanMatcher> matcher);

  void addImu(const ImuSample& s);
  void addOdom(const OdomSample& s);
  void addGnss(const GnssSample& s);
  void addGnssVelocity(const GnssVelocitySample& s);
  /// LiDAR のスキャン。lidar.async なら別スレッドに渡してすぐ戻る（処理中に届いたものは最新だけを残す）。
  void addLidarScan(LidarScan scan);

  /// 初期姿勢（map = UTM）。地図の近くなら、その周りで位置合わせして初期化する（設計書 3.11 節）。
  /// 地図が無ければ、与えた姿勢でそのまま初期化する。SAVED（前回保存した位置）の場合、位置合わせに
  /// 失敗したら初期化しない（EXTERNAL は、何度か失敗したら与えた姿勢で初期化する）。
  /// SAVED は起動時のためのものなので、フィルタがすでに動いていれば無視する。
  void setInitialPose(double t, const Pose2D& pose, const Mat3& cov_world,
                      InitialPoseSource source = InitialPoseSource::EXTERNAL);

  /// 最新の出力（フィルタが未初期化なら nullopt）。呼ぶたびに出力整形の時間が進み、地図のタイルも見直す。
  std::optional<LocalizationOutput> getOutput();

  Diagnostics diagnostics() const;
  std::optional<LidarMatchInfo> lastLidarMatch() const;
  /// 今の照合のターゲット（可視化用。地図が無い、または準備中なら nullptr）。
  std::shared_ptr<const MatchTarget> currentMapTarget() const;

  /// テスト用: 最新のフィルタ状態。
  std::optional<FilterState> latestState() const;
  /// テスト用: base_link の楕円体高の推定値。
  std::optional<double> baseHeight() const;

 private:
  struct PendingInit {
    double t = 0.0;
    Pose2D pose;
    Mat3 cov = Mat3::Identity();
    InitialPoseSource source = InitialPoseSource::EXTERNAL;
    int attempts = 0;
    double wait_start = -1.0;  ///< 待ち始めた時刻（データの時刻。姿勢推定器の初期化が済んでから数える）
  };

  void predictWith(const MotionInput& u);
  void applyCorrection(const UpdateResult& r, MeasurementKind kind, double t);
  void initializeFilter(const FilterState& st);
  void updateMaps(double t);
  /// 地図上での初期化を待ちすぎていないか確かめる（mtx_ を持って呼ぶ）。
  void checkPendingInit();
  /// 地図上での初期化をあきらめる。外部から与えた初期姿勢はそのまま使い、保存した位置は捨てる（mtx_ を持って呼ぶ）。
  void giveUpPendingInit(double t, const std::string& why);
  void processScan(const LidarScan& scan);
  void lidarWorker();
  /// LiDAR の観測を適用する（mtx_ を持って呼ぶ）。採用したら true。
  bool applyLidar(PoseMeasurement m);
  void reanchorWith(const Measurement& m, const Mat3& inflation, const std::string& source, double d2);

  LocalizerConfig cfg_;
  std::unique_ptr<IStateEstimator> est_;
  std::shared_ptr<ILogger> logger_;
  mutable std::mutex mtx_;

  MahalanobisGate gate_;
  StateHistory history_;
  AttitudeEstimator attitude_;
  MotionInputBuilder motion_;
  GnssMeasurementBuilder gnss_builder_;
  StopDetector stop_;
  OutputSmoother smoother_;
  StatusMonitor monitor_;
  Initializer initializer_;
  SourceArbiter arbiter_;
  RecoveryManager recovery_;
  ScanPreprocessor preprocessor_;
  LidarMeasurementBuilder lidar_builder_;

  std::shared_ptr<MapTileManager> maps_;
  std::shared_ptr<const IScanMatcher> matcher_;

  bool filter_initialized_ = false;
  bool was_ready_ = false;
  double last_imu_t_ = -1.0;
  double last_output_t_ = -1.0;
  Vec3 last_gyro_ = Vec3::Zero();
  MotionInput last_input_;
  std::optional<double> z_utm_;  ///< base_link の楕円体高（地図の照合の z の初期値）
  std::optional<PendingInit> pending_init_;
  std::optional<RelocalizeRequest> pending_relocalize_;
  double last_scan_t_ = -1e18;
  std::optional<LidarMatchInfo> last_match_;
  Diagnostics diag_;

  // LiDAR のワーカー（lidar.async のとき）
  std::mutex lidar_mtx_;
  std::condition_variable lidar_cv_;
  std::optional<LidarScan> lidar_slot_;
  bool lidar_stop_ = false;
  std::thread lidar_thread_;
};

}  // namespace gll
