#include "gll/localizer.hpp"

#include "gll/estimation/inv_ekf_se2.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <sstream>

namespace gll {

Localizer::Localizer(const LocalizerConfig& cfg, std::unique_ptr<IStateEstimator> estimator,
                     std::shared_ptr<ILogger> logger)
    : cfg_(cfg),
      est_(estimator ? std::move(estimator) : std::make_unique<InvEkfSe2>(cfg.estimator)),
      logger_(logger ? std::move(logger) : std::make_shared<NullLogger>()),
      gate_(cfg.estimator.gate_alpha),
      history_(cfg.estimator.history_length),
      attitude_(cfg.attitude),
      motion_(cfg.motion),
      gnss_builder_(cfg.gnss),
      stop_(cfg.stop),
      smoother_(cfg.output),
      monitor_(cfg.monitor),
      initializer_(cfg.init),
      arbiter_(cfg.arbiter, cfg.monitor),
      recovery_(cfg.recovery) {}

void Localizer::initializeFilter(const FilterState& st) {
  // 初期状態の時刻から最新の IMU 時刻までは、最新の入力で伝播しておく
  FilterState s = st;
  if (last_imu_t_ > s.t) s = est_->predict(s, last_input_, last_imu_t_ - s.t);
  history_.reset(s);
  smoother_.reset();
  monitor_.resetTravel();
  filter_initialized_ = true;
  was_ready_ = false;
  logger_->info("filter initialized by " + est_->name());
}

void Localizer::predictWith(const MotionInput& u) {
  last_input_ = u;
  if (!filter_initialized_) return;
  const FilterState& latest = history_.latest();
  const double dt = u.t - latest.t;
  if (dt <= 0.0) return;
  FilterState st = est_->predict(latest, u, dt);
  st.t = u.t;
  history_.push(st, u);
  monitor_.addTravel(std::hypot(latest.s * u.v, u.v_lat) * dt);  // デッドレコニング距離（設計書 3.12 節）

  if (const auto z = stop_.update(u)) {
    const UpdateResult r = history_.applyDelayed(*z, *est_, gate_, GatePolicy::REJECT_ON_FAIL);
    if (r.accepted) {
      applyCorrection(r, MeasurementKind::ZERO_RATE, z->t);
      ++diag_.zaru_accepted;
    }
  }
}

void Localizer::applyCorrection(const UpdateResult& r, MeasurementKind kind, double t) {
  smoother_.onCorrection(r.world_delta);
  monitor_.onAccepted(kind, t);
}

void Localizer::addImu(const ImuSample& s) {
  std::lock_guard<std::mutex> lk(mtx_);
  ++diag_.imu_count;
  if (last_imu_t_ >= 0.0 && s.t <= last_imu_t_) return;
  if (!attitude_.initialized()) {
    attitude_.addStaticSample(s);
    last_imu_t_ = s.t;
    return;
  }
  const auto vel = motion_.velocityAt(s.t);
  attitude_.update(s, vel ? vel->v : 0.0, vel ? vel->v_dot : 0.0);
  last_imu_t_ = s.t;
  if (vel && vel->stale) ++diag_.odom_stale_count;
  if (const auto u = motion_.build(s, attitude_)) predictWith(*u);
}

void Localizer::addOdom(const OdomSample& s) {
  std::lock_guard<std::mutex> lk(mtx_);
  ++diag_.odom_count;
  motion_.addOdom(s);
  // IMU が途切れている場合は ODOM のヨーレートで予測する（設計書 3.4 節）
  const bool imu_timeout = last_imu_t_ < 0.0 || s.t - last_imu_t_ > cfg_.motion.imu_timeout;
  if (imu_timeout && attitude_.initialized()) {
    if (const auto u = motion_.buildFromOdom(s)) {
      ++diag_.imu_fallback_count;
      predictWith(*u);
    }
  }
}

void Localizer::addGnss(const GnssSample& s) {
  std::lock_guard<std::mutex> lk(mtx_);
  ++diag_.gnss_count;
  if (!attitude_.initialized()) return;
  const GnssPositionResult res = gnss_builder_.buildPosition(s, attitude_);
  if (!res.position) {
    ++diag_.gnss_reject_reasons[toString(res.reason)];
    return;
  }
  const GnssPositionMeasurement& m = *res.position;

  if (!filter_initialized_) {
    const double sign = last_input_.v < 0.0 ? -1.0 : 1.0;
    const Vec3 b0 = attitude_.gyroBias();
    if (const auto st = initializer_.onGnss(m, sign, b0.z(), cfg_.estimator.estimate_odom_scale))
      initializeFilter(*st);
    return;
  }

  const Measurement meas = m;
  const UpdateResult r = history_.applyDelayed(meas, *est_, gate_, arbiter_.gatePolicy(meas));
  diag_.last_gnss_d2 = r.d2;
  if (r.too_old) {
    ++diag_.gnss_too_old;
    return;
  }
  if (r.accepted) {
    ++diag_.gnss_accepted;
    applyCorrection(r, MeasurementKind::GNSS_POSITION, m.t);
    arbiter_.onGnssAccepted(m.t);
    recovery_.onAccepted(meas);
    return;
  }
  // ゲートに落ちた RTK-FIX は捨てずに再アンカーの候補にする（設計書 3.13.3 節）
  ++diag_.gnss_deferred;
  const RecoveryAction act = recovery_.onRejected(meas, r, history_, *est_);
  if (act.kind == RecoveryAction::Kind::REANCHOR && act.measurement) {
    const UpdateResult rr =
        history_.applyDelayed(*act.measurement, *est_, gate_, GatePolicy::SKIP, act.inflation);
    if (rr.accepted) {
      applyCorrection(rr, MeasurementKind::GNSS_POSITION, m.t);
      arbiter_.onGnssAccepted(m.t);
      recovery_.onReanchored();
      ++diag_.reanchor_count;
      std::ostringstream os;
      os << "re-anchored to GNSS (d2=" << r.d2 << ")";
      logger_->warn(os.str());
    }
  }
}

void Localizer::addGnssVelocity(const GnssVelocitySample& s) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!filter_initialized_) return;
  const auto h =
      gnss_builder_.buildHeading(s, last_input_.omega - history_.latest().b, arbiter_.isGnssFixActive(s.t));
  if (!h) return;
  const Measurement meas = *h;
  const UpdateResult r = history_.applyDelayed(meas, *est_, gate_, arbiter_.gatePolicy(meas));
  if (r.accepted) {
    ++diag_.heading_accepted;
    applyCorrection(r, MeasurementKind::HEADING, h->t);
  }
}

void Localizer::setInitialPose(double t, const Pose2D& pose, const Mat3& cov_world) {
  std::lock_guard<std::mutex> lk(mtx_);
  const double b0 = attitude_.initialized() ? attitude_.gyroBias().z() : 0.0;
  const FilterState st = initializer_.fromExternalPose(t, pose, cov_world, b0,
                                                       cfg_.estimator.estimate_odom_scale, *est_);
  initializeFilter(st);
}

std::optional<LocalizationOutput> Localizer::getOutput() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!filter_initialized_) return std::nullopt;
  const FilterState& st = history_.latest();
  const bool ready = initializer_.updateReady(st, *est_);
  if (ready && !was_ready_) {
    smoother_.reset();  // 出力を開始する時点の推定値から始める
    last_output_t_ = st.t;
  }
  was_ready_ = ready;

  const Mat3 raw_cov = est_->worldCovariance(st);
  const double dt = (last_output_t_ < 0.0) ? 0.0 : st.t - last_output_t_;
  last_output_t_ = st.t;
  const Pose2D raw = st.X.toPose();
  const SmoothedOutput so = smoother_.apply(raw, raw_cov, dt);

  const Eigen::SelfAdjointEigenSolver<Mat2> es(raw_cov.topLeftCorner<2, 2>());
  recovery_.updateLost(std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0)), cfg_.monitor.lost_stddev);

  LocalizationOutput out;
  out.t = st.t;
  out.pose = so.pose;
  out.raw_pose = raw;
  out.cov = so.cov;
  out.raw_cov = raw_cov;
  out.offset = smoother_.offset();
  out.v = last_input_.v * st.s;
  out.v_lat = last_input_.v_lat;
  out.yaw_rate = last_input_.omega - st.b;
  out.roll = attitude_.roll();
  out.pitch = attitude_.pitch();
  out.gyro_bias = st.b;
  out.odom_scale = st.s;
  out.recovery = recovery_.state();
  out.status = monitor_.evaluate(st.t, ready, so.cov, smoother_.offsetExceeded(), recovery_.state());
  out.dr_distance = monitor_.drDistance();
  out.dr_distance_exceeded = ready && monitor_.drDistanceExceeded();
  return out;
}

Diagnostics Localizer::diagnostics() const {
  std::lock_guard<std::mutex> lk(mtx_);
  Diagnostics d = diag_;
  d.attitude_initialized = attitude_.initialized();
  d.filter_initialized = filter_initialized_;
  d.init_phase = initializer_.phase();
  return d;
}

std::optional<FilterState> Localizer::latestState() const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!filter_initialized_) return std::nullopt;
  return history_.latest();
}

}  // namespace gll
