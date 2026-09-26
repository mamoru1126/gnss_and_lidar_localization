// Localizer: 外部 API の窓口（ソフトウェア構成 3.2 節）。
// スレッド安全（内部の mutex で保護する）。時刻はすべてデータのタイムスタンプ [s]。
#pragma once

#include "gll/common/config.hpp"
#include "gll/common/logger.hpp"
#include "gll/estimation/initializer.hpp"
#include "gll/estimation/mahalanobis_gate.hpp"
#include "gll/estimation/output_smoother.hpp"
#include "gll/estimation/recovery_manager.hpp"
#include "gll/estimation/source_arbiter.hpp"
#include "gll/estimation/state_estimator.hpp"
#include "gll/estimation/state_history.hpp"
#include "gll/estimation/status_monitor.hpp"
#include "gll/measurement/attitude_estimator.hpp"
#include "gll/measurement/gnss_measurement_builder.hpp"
#include "gll/measurement/motion_input_builder.hpp"
#include "gll/measurement/stop_detector.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

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
};

class Localizer {
 public:
  explicit Localizer(const LocalizerConfig& cfg, std::unique_ptr<IStateEstimator> estimator = nullptr,
                     std::shared_ptr<ILogger> logger = nullptr);

  void addImu(const ImuSample& s);
  void addOdom(const OdomSample& s);
  void addGnss(const GnssSample& s);
  void addGnssVelocity(const GnssVelocitySample& s);
  void setInitialPose(double t, const Pose2D& pose, const Mat3& cov_world);

  /// 最新の出力（フィルタが未初期化なら nullopt）。呼ぶたびに出力整形の時間が進む。
  std::optional<LocalizationOutput> getOutput();

  Diagnostics diagnostics() const;

  /// テスト用: 最新のフィルタ状態。
  std::optional<FilterState> latestState() const;

 private:
  void predictWith(const MotionInput& u);
  void applyCorrection(const UpdateResult& r, MeasurementKind kind, double t);
  void initializeFilter(const FilterState& st);

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

  bool filter_initialized_ = false;
  bool was_ready_ = false;
  double last_imu_t_ = -1.0;
  double last_output_t_ = -1.0;
  MotionInput last_input_;
  Diagnostics diag_;
};

}  // namespace gll
