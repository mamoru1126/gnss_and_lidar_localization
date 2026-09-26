#include "gll/localizer.hpp"

#include "gll/estimation/inv_ekf_se2.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace gll {
namespace {

Mat3 blockT(double yaw) {
  Mat3 T = Mat3::Identity();
  T.topLeftCorner<2, 2>() = SE2::rot(yaw);
  return T;
}

std::string fmt(double v, int prec) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

}  // namespace

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
      recovery_(cfg.recovery, cfg.relocalize),
      preprocessor_(cfg.lidar),
      lidar_builder_(cfg.lidar) {}

Localizer::~Localizer() {
  {
    std::lock_guard<std::mutex> lk(lidar_mtx_);
    lidar_stop_ = true;
  }
  lidar_cv_.notify_all();
  if (lidar_thread_.joinable()) lidar_thread_.join();
}

void Localizer::setMap(std::shared_ptr<MapTileManager> maps, std::shared_ptr<const IScanMatcher> matcher) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    maps_ = std::move(maps);
    matcher_ = std::move(matcher);
    if (maps_) {
      for (const auto& [a, b] : maps_->overlappingGroups())
        logger_->warn("map groups " + a + " and " + b +
                      " overlap; the operation assumes that they are separated by a GNSS section (design 5.1)");
    }
  }
  if (cfg_.lidar.async && !lidar_thread_.joinable()) lidar_thread_ = std::thread([this] { lidarWorker(); });
}

void Localizer::initializeFilter(const FilterState& st) {
  // 初期状態の時刻から最新の IMU 時刻までは、最新の入力で伝播しておく
  FilterState s = st;
  if (last_imu_t_ > s.t) s = est_->predict(s, last_input_, last_imu_t_ - s.t);
  history_.reset(s);
  smoother_.reset();
  monitor_.resetTravel();
  recovery_.onExternalPose();
  pending_relocalize_.reset();
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

void Localizer::reanchorWith(const Measurement& m, const Mat3& inflation, const std::string& source, double d2) {
  const UpdateResult rr = history_.applyDelayed(m, *est_, gate_, GatePolicy::SKIP, inflation);
  if (!rr.accepted) return;
  const MeasurementKind kind = measurementKind(m);
  const double t = measurementTime(m);
  applyCorrection(rr, kind, t);
  if (kind == MeasurementKind::GNSS_POSITION) {
    arbiter_.onGnssAccepted(t);
    ++diag_.reanchor_count;
  } else {
    ++diag_.lidar_reanchor_count;
  }
  recovery_.onReanchored();
  logger_->warn("re-anchored to " + source + " (d2=" + fmt(d2, 1) + ")");
}

void Localizer::addImu(const ImuSample& s) {
  std::lock_guard<std::mutex> lk(mtx_);
  ++diag_.imu_count;
  if (last_imu_t_ >= 0.0 && s.t <= last_imu_t_) return;
  last_gyro_ = s.gyro;
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
    if (const auto st = initializer_.onGnss(m, sign, b0.z(), cfg_.estimator.estimate_odom_scale)) {
      initializeFilter(*st);
      z_utm_ = gnss_builder_.baseHeight(s, attitude_);
    }
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
    pending_relocalize_.reset();  // GNSS で位置が確定したので、LiDAR の再位置推定は要らない
    z_utm_ = gnss_builder_.baseHeight(s, attitude_);
    return;
  }
  // ゲートに落ちた RTK-FIX は捨てずに再アンカーの候補にする（設計書 3.13.3 節）
  ++diag_.gnss_deferred;
  const RecoveryAction act = recovery_.onRejected(meas, r, history_, *est_, true);
  if (act.kind == RecoveryAction::Kind::REANCHOR && act.measurement) {
    reanchorWith(*act.measurement, act.inflation, "GNSS", r.d2);
    z_utm_ = gnss_builder_.baseHeight(s, attitude_);
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

void Localizer::setInitialPose(double t, const Pose2D& pose, const Mat3& cov_world, InitialPoseSource source) {
  std::lock_guard<std::mutex> lk(mtx_);
  const Vec2 p(pose.x, pose.y);
  if (maps_ && matcher_ && maps_->hasMapWithin(p, cfg_.relocalize.init_map_distance)) {
    // 地図の上なら、その周りで位置合わせしてから初期化する（次のスキャンで行う）
    pending_init_ = PendingInit{t, pose, cov_world, source, 0};
    pending_relocalize_.reset();
    initializer_.waitForMapMatch();
    recovery_.onExternalPose();
    logger_->info("initial pose is on the map: searching around it (" + fmt(pose.x, 2) + ", " + fmt(pose.y, 2) +
                  ", " + fmt(rad2deg(pose.yaw), 1) + " deg)");
    maps_->update(t, p, pose.yaw, 0.0, true);  // 周りのタイルを読み込ませる
    return;
  }
  if (source == InitialPoseSource::SAVED) {
    logger_->warn("saved pose is not on any map; ignored (waiting for GNSS or an initial pose)");
    return;
  }
  pending_init_.reset();
  const double b0 = attitude_.initialized() ? attitude_.gyroBias().z() : 0.0;
  const FilterState st = initializer_.fromExternalPose(t, pose, cov_world, b0, cfg_.estimator.estimate_odom_scale, *est_);
  initializeFilter(st);
}

void Localizer::updateMaps(double t) {
  if (!maps_) return;
  if (pending_init_) {
    maps_->update(t, Vec2(pending_init_->pose.x, pending_init_->pose.y), pending_init_->pose.yaw, 0.0);
  } else if (filter_initialized_) {
    const FilterState& st = history_.latest();
    maps_->update(st.t, st.X.t(), st.X.yaw(), last_input_.v * st.s);
  }
}

void Localizer::addLidarScan(LidarScan scan) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.lidar_count;
  }
  if (!cfg_.lidar.async || !lidar_thread_.joinable()) {
    processScan(scan);
    return;
  }
  bool dropped = false;
  {
    std::lock_guard<std::mutex> lk(lidar_mtx_);
    dropped = lidar_slot_.has_value();
    lidar_slot_ = std::move(scan);
  }
  lidar_cv_.notify_one();
  if (dropped) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.lidar_dropped;
  }
}

void Localizer::lidarWorker() {
  while (true) {
    LidarScan scan;
    {
      std::unique_lock<std::mutex> lk(lidar_mtx_);
      lidar_cv_.wait(lk, [this] { return lidar_stop_ || lidar_slot_.has_value(); });
      if (lidar_stop_) return;
      scan = std::move(*lidar_slot_);
      lidar_slot_.reset();
    }
    try {
      processScan(scan);
    } catch (const std::exception& e) {
      logger_->error(std::string("LiDAR processing failed: ") + e.what());
    }
  }
}

bool Localizer::applyLidar(PoseMeasurement m) {
  const auto st_m = history_.stateAt(m.t, *est_);
  if (!st_m) {
    ++diag_.lidar_too_old;
    return false;
  }
  // GNSS FIX 中は、整合していれば共分散を膨らませて融合し、食い違えば棄却する（設計書 3.13.2 節）
  const LidarDecision dec = arbiter_.classifyLidar(m, *st_m);
  if (dec == LidarDecision::REJECT_MISMATCH) {
    ++diag_.lidar_mismatch;
    return false;
  }
  const Measurement meas = m;
  const UpdateResult r = history_.applyDelayed(meas, *est_, gate_, SourceArbiter::lidarPolicy(dec));
  if (r.too_old) {
    ++diag_.lidar_too_old;
    return false;
  }
  if (r.accepted) {
    ++diag_.lidar_accepted;
    applyCorrection(r, MeasurementKind::POSE, m.t);
    recovery_.onAccepted(meas);
    return true;
  }
  ++diag_.lidar_deferred;
  const RecoveryAction act =
      recovery_.onRejected(meas, r, history_, *est_, arbiter_.isGnssFixActive(m.t), monitor_.drDistance());
  if (act.kind == RecoveryAction::Kind::REANCHOR && act.measurement) {
    reanchorWith(*act.measurement, act.inflation, "LiDAR (" + m.map_group + ")", r.d2);
    return true;
  }
  if (act.kind == RecoveryAction::Kind::RELOCALIZE && act.relocalize) {
    pending_relocalize_ = act.relocalize;
    logger_->warn("LiDAR rejected repeatedly: relocalizing around the estimate");
  }
  return false;
}

void Localizer::processScan(const LidarScan& scan) {
  enum class Mode { TRACK, INIT, RELOCALIZE };
  Mode mode = Mode::TRACK;
  PendingInit init;
  RelocalizeRequest reloc;
  std::optional<FilterState> st;
  Attitude att;
  ScanMotion motion;
  std::optional<double> z_utm;
  std::shared_ptr<MapTileManager> maps;
  std::shared_ptr<const IScanMatcher> matcher;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!maps_ || !matcher_ || !attitude_.initialized()) return;
    if (scan.t - last_scan_t_ < cfg_.lidar.min_interval) return;
    maps = maps_;
    matcher = matcher_;
    if (pending_init_) {
      mode = Mode::INIT;
      init = *pending_init_;
    } else if (!filter_initialized_) {
      return;  // GNSS か初期姿勢を待つ
    } else {
      st = history_.stateAt(scan.t, *est_);
      if (!st) {
        ++diag_.lidar_too_old;
        return;
      }
      if (pending_relocalize_) {
        mode = Mode::RELOCALIZE;
        reloc = *pending_relocalize_;
      } else if (recovery_.relocalizeDue(scan.t) && !arbiter_.isGnssFixActive(scan.t)) {
        // LOST の間は、一定の間隔で最も広い範囲の再位置推定を試す
        mode = Mode::RELOCALIZE;
        reloc = recovery_.makeRelocalizeRequest(scan.t, *st, *est_, true, monitor_.drDistance());
      }
    }
    last_scan_t_ = scan.t;
    att = attitude_.attitudeAt(scan.t);
    motion.angular_velocity = last_gyro_ - attitude_.gyroBias();
    const double s = filter_initialized_ ? history_.latest().s : 1.0;
    motion.velocity = Vec3(s * last_input_.v, last_input_.v_lat, 0.0);
    z_utm = z_utm_;
  }

  const std::shared_ptr<const MatchTarget> target = maps->currentTarget();
  if (!target) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.lidar_no_target;
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsedMs = [&t0] {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  };
  const std::vector<Vec3f> pts = preprocessor_.process(scan, motion);
  // 照合の初期値の z（地図座標）: 推定した楕円体高 → 無ければ地図の地面の高さ（設計書 3.9 節・3.11 節）
  const auto zMap = [&](const Vec2& xy) -> double {
    if (z_utm) return target->anchor.utmToMap(Vec3(0.0, 0.0, *z_utm)).z();
    if (const auto g = target->groundHeight(xy.x(), xy.y(), 3.0)) return *g + cfg_.lidar.base_link_height;
    return target->anchor.mapPoint().z();
  };

  if (mode == Mode::INIT || mode == Mode::RELOCALIZE) {
    const SE2 center = mode == Mode::INIT ? SE2::fromPose(init.pose.x, init.pose.y, init.pose.yaw) : st->X;
    const SE2 cm = target->anchor.toMap(center);
    PoseSearchRequest req;
    req.center = makePose(Vec3(cm.t().x(), cm.t().y(), zMap(cm.t())), att.roll, att.pitch, cm.yaw());
    if (mode == Mode::INIT) {
      const double sxy = std::sqrt(std::max(init.cov(0, 0), init.cov(1, 1)));
      const double syaw = std::sqrt(std::max(init.cov(2, 2), 0.0));
      req.radius = std::clamp(3.0 * sxy, cfg_.relocalize.init_min_radius, cfg_.relocalize.init_max_radius);
      req.yaw_range = std::clamp(3.0 * syaw, cfg_.relocalize.init_min_yaw, cfg_.relocalize.init_max_yaw);
    } else {
      req.radius = reloc.radius;
      req.yaw_range = reloc.yaw_range;
    }
    const PoseSearchResult res = matcher->search(pts, *target, req);
    const double ms = elapsedMs();
    const Eigen::Isometry3d Tu = target->anchor.poseMapToUtm(res.best.T_map_base);
    const Pose2D found{Tu.translation().x(), Tu.translation().y(), yawOf(Tu.linear())};

    std::lock_guard<std::mutex> lk(mtx_);
    LidarMatchInfo info;
    info.t = scan.t;
    info.pose = found;
    info.group = target->group;
    info.inlier_ratio = res.best.inlier_ratio;
    info.overlap = res.best_overlap;
    info.time_ms = ms;
    std::ostringstream detail;
    detail << res.num_hypotheses << " hypotheses, overlap " << fmt(res.best_overlap, 2) << " (second "
           << fmt(res.second_overlap, 2) << "), " << fmt(ms, 0) << " ms";

    if (mode == Mode::INIT) {
      ++diag_.map_init_attempts;
      if (!pending_init_ || pending_init_->t != init.t) return;  // その間に別の初期姿勢が与えられた
      if (res.found) {
        const Mat3 T = blockT(found.yaw);
        const Mat3 cov = T * lidar_builder_.covarianceBody(res.best) * T.transpose() + target->anchor.covarianceWorld();
        const double b0 = attitude_.gyroBias().z();
        const FilterState s0 =
            initializer_.fromExternalPose(scan.t, found, cov, b0, cfg_.estimator.estimate_odom_scale, *est_);
        pending_init_.reset();
        initializeFilter(s0);
        monitor_.onAccepted(MeasurementKind::POSE, scan.t);
        z_utm_ = Tu.translation().z();
        info.accepted = true;
        info.status = "INITIALIZED";
        logger_->info("initialized on map " + target->group + " at (" + fmt(found.x, 2) + ", " + fmt(found.y, 2) +
                      ", " + fmt(rad2deg(found.yaw), 1) + " deg): " + detail.str());
      } else {
        ++pending_init_->attempts;
        info.status = "INIT_" + res.reason;
        logger_->warn("map initialization failed (" + res.reason + "): " + detail.str());
        if (pending_init_->attempts >= cfg_.relocalize.init_max_attempts) {
          if (pending_init_->source == InitialPoseSource::EXTERNAL) {
            logger_->warn("could not match the initial pose to the map; using the given pose as is");
            const double b0 = attitude_.gyroBias().z();
            const FilterState s0 = initializer_.fromExternalPose(
                scan.t, pending_init_->pose, pending_init_->cov, b0, cfg_.estimator.estimate_odom_scale, *est_);
            pending_init_.reset();
            initializeFilter(s0);
          } else {
            logger_->warn("could not match the saved pose to the map; waiting for GNSS or an initial pose");
            pending_init_.reset();
            if (!filter_initialized_) initializer_.reset();
          }
        }
      }
    } else {
      ++diag_.relocalize_attempts;
      pending_relocalize_.reset();
      if (!filter_initialized_ || pending_init_) return;
      if (res.found) {
        PoseMeasurement m;
        m.t = scan.t;
        m.Z = SE2::fromPose(found.x, found.y, found.yaw);
        m.cov_body = lidar_builder_.covarianceBody(res.best);
        m.anchor_cov_world = target->anchor.covarianceWorld();
        m.map_group = target->group;
        const auto x_t = history_.stateAt(scan.t, *est_);
        Vec3 e = x_t ? (x_t->X.inverse() * m.Z).Log() : Vec3::Zero();
        Mat3 inflation = e.cwiseProduct(e).asDiagonal();
        inflation(0, 0) += cfg_.recovery.reanchor_extra_stddev_xy * cfg_.recovery.reanchor_extra_stddev_xy;
        inflation(1, 1) += cfg_.recovery.reanchor_extra_stddev_xy * cfg_.recovery.reanchor_extra_stddev_xy;
        inflation(2, 2) += cfg_.recovery.reanchor_extra_stddev_yaw * cfg_.recovery.reanchor_extra_stddev_yaw;
        recovery_.onRelocalizeResult(scan.t, true);
        reanchorWith(m, inflation, "relocalization on " + target->group, 0.0);
        ++diag_.relocalize_success;
        z_utm_ = Tu.translation().z();
        info.accepted = true;
        info.status = "RELOCALIZED";
        logger_->warn("relocalized: " + detail.str());
      } else {
        recovery_.onRelocalizeResult(scan.t, false);
        info.status = "RELOCALIZE_" + res.reason;
        logger_->warn("relocalization failed (" + res.reason + "): " + detail.str());
      }
    }
    last_match_ = info;
    return;
  }

  // --- 通常の追跡: 予測姿勢を初期値にして GICP ---
  const SE2 Xm = target->anchor.toMap(st->X);
  const Eigen::Isometry3d init_T = makePose(Vec3(Xm.t().x(), Xm.t().y(), zMap(Xm.t())), att.roll, att.pitch, Xm.yaw());
  const auto src = matcher->prepareSource(pts, cfg_.lidar.source_voxel_size);
  if (static_cast<int>(src->size()) < cfg_.lidar.min_source_points) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.lidar_few_points;
    return;
  }
  const RegistrationResult r = matcher->align(*src, *target, init_T);
  const LidarMeasurementResult built = lidar_builder_.build(scan.t, r, *target, init_T);
  const double ms = elapsedMs();
  const Eigen::Isometry3d Tu = target->anchor.poseMapToUtm(r.T_map_base);

  std::lock_guard<std::mutex> lk(mtx_);
  ++diag_.lidar_matched;
  diag_.last_match_ms = ms;
  diag_.last_inlier_ratio = r.inlier_ratio;
  diag_.last_overlap = r.overlap;
  LidarMatchInfo info;
  info.t = scan.t;
  info.pose = Pose2D{Tu.translation().x(), Tu.translation().y(), yawOf(Tu.linear())};
  info.group = target->group;
  info.inlier_ratio = r.inlier_ratio;
  info.overlap = r.overlap;
  info.time_ms = ms;
  if (!filter_initialized_ || pending_init_) return;  // 処理中に初期化し直された
  if (!built.pose) {
    ++diag_.lidar_reject_reasons[toString(built.reason)];
    info.status = toString(built.reason);
    const RecoveryAction act = recovery_.onLidarFailure(scan.t, history_, *est_, arbiter_.isGnssFixActive(scan.t),
                                                        monitor_.drDistance());
    if (act.kind == RecoveryAction::Kind::RELOCALIZE && act.relocalize) {
      pending_relocalize_ = act.relocalize;
      logger_->warn("LiDAR matching failed repeatedly: relocalizing around the estimate");
    }
  } else {
    info.cov_body = built.pose->cov_body;
    info.accepted = applyLidar(*built.pose);
    info.status = info.accepted ? "ACCEPTED" : "REJECTED";
    if (info.accepted) z_utm_ = Tu.translation().z();
  }
  last_match_ = info;
}

std::optional<LocalizationOutput> Localizer::getOutput() {
  std::lock_guard<std::mutex> lk(mtx_);
  updateMaps(last_imu_t_);
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
  if (maps_) out.active_map_group = maps_->activeGroup();
  return out;
}

Diagnostics Localizer::diagnostics() const {
  std::lock_guard<std::mutex> lk(mtx_);
  Diagnostics d = diag_;
  d.attitude_initialized = attitude_.initialized();
  d.filter_initialized = filter_initialized_;
  d.init_phase = initializer_.phase();
  if (maps_) d.map = maps_->stats();
  d.anchor_mismatch = arbiter_.mismatchStats();
  return d;
}

std::optional<LidarMatchInfo> Localizer::lastLidarMatch() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return last_match_;
}

std::shared_ptr<const MatchTarget> Localizer::currentMapTarget() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return maps_ ? maps_->currentTarget() : nullptr;
}

std::optional<FilterState> Localizer::latestState() const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!filter_initialized_) return std::nullopt;
  return history_.latest();
}

std::optional<double> Localizer::baseHeight() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return z_utm_;
}

}  // namespace gll
