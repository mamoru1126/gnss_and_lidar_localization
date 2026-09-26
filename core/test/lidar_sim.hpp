// LiDAR を含む統合シミュレーションの共通部品（設計書 9 章「シミュレーション」）。
// 合成環境（sim_world.hpp）から地図グループのタイルを作り、真値の走行から IMU・ODOM・GNSS・LiDAR を生成して
// Localizer に流す。世界座標系 = 地図グループ「A」の座標系とし、UTM への変換はグループ A のアンカーで行う。
#pragma once

#include "gll/localizer.hpp"
#include "gll/map/map_config.hpp"
#include "gll/map/map_tiler.hpp"
#include "gll/matching/gicp_matcher.hpp"

#include "sim_world.hpp"

#include <unistd.h>

#include <deque>
#include <filesystem>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace gll::sim {

/// 世界座標系での真値の運動（平らな地面の上を、前進速度とヨーレートで動く）。
struct MotionProfile {
  Vec2 start = Vec2::Zero();
  double start_yaw = 0.0;
  /// 時刻 t の前進加速度 [m/s²] とヨーレート [rad/s]（速度は 0〜v_max に制限する）。
  std::function<double(double t, double v)> accel = [](double, double) { return 0.0; };
  std::function<double(double t, double v)> yaw_rate = [](double, double) { return 0.0; };
  double v_max = 1.5;
};

struct SensorPlan {
  double duration = 60.0;
  double lidar_rate = 10.0;                         ///< [Hz]
  std::function<bool(double t, const Vec2& world_xy)> gnss_fix = [](double, const Vec2&) { return false; };
  std::function<bool(double t)> lidar_on = [](double) { return true; };
  std::function<double(double t)> odom_scale_error = [](double) { return 0.0; };  ///< ODOM の速度の相対誤差
  double gnss_delay = 0.05;
  std::optional<double> init_time;   ///< 外部の初期姿勢を与える時刻
  Pose2D init_error;                 ///< 真値からのずれ（UTM）
  Mat3 init_cov = Vec3(1.0, 1.0, deg2rad(20.0) * deg2rad(20.0)).asDiagonal();
  unsigned seed = 11;
};

struct SimRecord {
  double t;
  Pose2D truth;  ///< UTM
  Vec2 world_xy;
  LocalizationOutput out;
};

struct SimRun {
  std::vector<SimRecord> records;
  Diagnostics diag;
  double ready_time = -1.0;
};

/// 地図グループ（世界の一部の点群を、そのグループの座標系で表したもの）。
struct SimGroup {
  std::string id;
  Eigen::Isometry3d T_world_group = Eigen::Isometry3d::Identity();  ///< グループの座標系 → 世界
  std::function<bool(const Vec3&)> contains = [](const Vec3&) { return true; };  ///< 世界座標の点を含めるか
  Pose2D anchor_error;  ///< アンカーに加える誤差（x, y [m], yaw [rad]。グループの座標系）
};

inline std::string makeTempDir(const std::string& name) {
  const auto d = std::filesystem::temp_directory_path() / ("gll_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(d);
  std::filesystem::create_directories(d);
  return d.string();
}

class LidarSimulator {
 public:
  LidarSimulator(World world, std::vector<SimGroup> groups, const LocalizerConfig& lc, const std::string& dir)
      : world_(std::move(world)), groups_(std::move(groups)), lc_(lc), utm_(lc.gnss.utm_zone, lc.gnss.utm_north) {
    // 世界（= グループ A の座標系）のアンカー: 東京駅付近、地図の x 軸の方位 30°
    AnchorConfig a;
    a.latitude = 35.681236;
    a.longitude = 139.767125;
    a.ellipsoid_height = 40.0;
    a.heading = deg2rad(30.0);
    world_anchor_ = MapAnchor::fromConfig(a, utm_);
    const auto pts = world_.samplePoints(0.2, 0.01, 3);
    for (const SimGroup& g : groups_) {
      std::vector<Vec3f> gp;
      const Eigen::Isometry3d Tgw = g.T_world_group.inverse();
      for (const auto& p : pts)
        if (g.contains(p.cast<double>())) gp.push_back((Tgw * p.cast<double>()).cast<float>());
      TilerOptions opt;
      opt.voxel_size = 0.25;
      opt.num_threads = lc.lidar.num_threads;
      const TilerResult r = tileMap(gp, opt, g.id);
      const std::string gdir = dir + "/" + g.id;
      writeTiles(gdir, r);
      MapGroup mg;
      mg.id = g.id;
      // グループのアンカー: グループ座標の原点の UTM 位置と、x 軸のグリッド方位（UTM で直接与える）
      AnchorConfig ga;
      ga.use_utm = true;
      Eigen::Isometry3d T0 = g.T_world_group;
      const Vec3 o = world_anchor_.mapToUtm(T0.translation());
      const double phi = world_anchor_.rotation() + yawOf(T0.linear());  // 東からの角度
      ga.easting = o.x() + g.anchor_error.x;
      ga.northing = o.y() + g.anchor_error.y;
      ga.ellipsoid_height = o.z();
      ga.grid_heading = kPi / 2.0 - (phi + g.anchor_error.yaw);
      ga.map_point = Vec3::Zero();
      mg.anchor = MapAnchor::fromConfig(ga, utm_);
      mg.index = TileIndex::load(gdir + "/tile_index.yaml", g.id);
      map_groups_.push_back(mg);
    }
  }

  const MapAnchor& worldAnchor() const { return world_anchor_; }
  const std::vector<MapGroup>& mapGroups() const { return map_groups_; }

  std::shared_ptr<MapTileManager> makeMapManager(std::shared_ptr<ILogger> logger = nullptr) const {
    return std::make_shared<MapTileManager>(lc_.map, map_groups_, std::make_shared<BinaryTileLoader>(),
                                            std::make_shared<GicpMatcher>(lc_.lidar, lc_.relocalize), logger);
  }

  Pose2D toUtm(const Vec2& xy, double yaw) const {
    const SE2 X = world_anchor_.toUtm(SE2::fromPose(xy.x(), xy.y(), yaw));
    return X.toPose();
  }

  SimRun run(Localizer& loc, const MotionProfile& mp, const SensorPlan& sp) {
    SimRun res;
    std::mt19937 rng(sp.seed);
    std::normal_distribution<double> n(0.0, 1.0);
    const double dt = 0.01;
    Vec2 p = mp.start;
    double yaw = mp.start_yaw, v = 0.0;
    const double s_true = 1.02;
    const Vec3 gyro_bias(0.002, -0.001, 0.003);
    std::deque<std::pair<double, GnssSample>> gnss_queue;
    bool init_sent = false;
    const int lidar_every = std::max(1, static_cast<int>(std::lround(1.0 / (sp.lidar_rate * dt))));
    LidarModel model;
    // スキャン中の運動を再現するため、直近の真値を覚えておく
    struct Past {
      double t;
      Vec2 p;
      double yaw;
    };
    std::deque<Past> past;

    for (int k = 0; k * dt <= sp.duration + 1e-9; ++k) {
      const double t = k * dt;
      double a = 0.0, w = 0.0;
      if (k > 0) {
        a = mp.accel(t, v);
        w = v > 1e-6 ? mp.yaw_rate(t, v) : 0.0;
        const double v_new = std::clamp(v + a * dt, 0.0, mp.v_max);
        a = (v_new - v) / dt;
        v = v_new;
        const double ym = yaw + 0.5 * w * dt;
        p += v * dt * Vec2(std::cos(ym), std::sin(ym));
        yaw = wrapAngle(yaw + w * dt);
      }
      past.push_back({t, p, yaw});
      while (past.size() > 20) past.pop_front();
      const Pose2D truth = toUtm(p, yaw);

      // IMU（100 Hz）
      ImuSample imu;
      imu.t = t;
      imu.gyro = Vec3(0, 0, w) + gyro_bias + 0.002 * Vec3(n(rng), n(rng), n(rng));
      imu.acc = Vec3(a, v * w, kGravity) + 0.02 * Vec3(n(rng), n(rng), n(rng));
      loc.addImu(imu);
      // ODOM（50 Hz）
      if (k % 2 == 0) {
        OdomSample o;
        o.t = t;
        o.v = (v > 1e-6) ? v / s_true * (1.0 + sp.odom_scale_error(t)) + 0.01 * n(rng) : 0.0;
        o.yaw_rate = w + 0.005 * n(rng);
        loc.addOdom(o);
      }
      // GNSS（10 Hz）
      if (k % 10 == 0 && sp.gnss_fix(t, p)) {
        const Vec2 lever = lc_.gnss.lever_arm.head<2>();
        const Vec2 ant = Vec2(truth.x, truth.y) + SE2::rot(truth.yaw) * lever + 0.015 * Vec2(n(rng), n(rng));
        const LatLon ll = utm_.inverse(ant.x(), ant.y());
        GnssSample g;
        g.t = t;
        g.lat = ll.lat;
        g.lon = ll.lon;
        g.h = 40.0 + lc_.gnss.lever_arm.z();
        g.raw_status = 2;
        g.cov_known = true;
        g.cov_enu = Mat3::Identity() * 0.015 * 0.015;
        gnss_queue.emplace_back(t + sp.gnss_delay, g);
      }
      while (!gnss_queue.empty() && gnss_queue.front().first <= t + 1e-9) {
        loc.addGnss(gnss_queue.front().second);
        gnss_queue.pop_front();
      }
      // LiDAR（スキャン終了時刻 t。スキャン中の 0.1 s の運動を再現する）
      if (k % lidar_every == 0 && t >= model.period && sp.lidar_on(t)) {
        const auto base_at = [&](double tau) {
          // tau はスキャン開始からの時間。past を線形補間する
          const double tq = t - model.period + tau;
          const Past* a0 = &past.front();
          const Past* a1 = &past.back();
          for (std::size_t i = 1; i < past.size(); ++i)
            if (past[i].t >= tq) {
              a0 = &past[i - 1];
              a1 = &past[i];
              break;
            }
          const double s = a1->t > a0->t ? std::clamp((tq - a0->t) / (a1->t - a0->t), 0.0, 1.0) : 0.0;
          const Vec2 pp = a0->p + s * (a1->p - a0->p);
          const double yy = a0->yaw + s * wrapAngle(a1->yaw - a0->yaw);
          return makePose(Vec3(pp.x(), pp.y(), 0.0), 0.0, 0.0, yy);
        };
        const LidarScan scan = simulateScan(
            world_, model, [&](double tau) { return base_at(tau) * lc_.lidar.T_base_lidar; }, t, rng);
        loc.addLidarScan(scan);
      }
      if (sp.init_time && !init_sent && t >= *sp.init_time) {
        Pose2D ip = truth;
        ip.x += sp.init_error.x;
        ip.y += sp.init_error.y;
        ip.yaw = wrapAngle(ip.yaw + sp.init_error.yaw);
        loc.setInitialPose(t, ip, sp.init_cov);
        init_sent = true;
      }
      if (const auto out = loc.getOutput()) {
        if (out->status != LocalizationStatus::INITIALIZING && res.ready_time < 0.0) res.ready_time = t;
        res.records.push_back(SimRecord{t, truth, p, *out});
      }
    }
    res.diag = loc.diagnostics();
    return res;
  }

 private:
  World world_;
  std::vector<SimGroup> groups_;
  LocalizerConfig lc_;
  UtmProjector utm_;
  MapAnchor world_anchor_;
  std::vector<MapGroup> map_groups_;
};

/// LiDAR を使うテストの既定の設定（同期処理。LiDAR は base_link の 0.5 m 前、1.8 m 上）。
inline LocalizerConfig lidarTestConfig() {
  LocalizerConfig c;
  c.gnss.lever_arm = Vec3(0.3, 0.2, 0.5);
  c.lidar.T_base_lidar = makePose(Vec3(0.5, 0.0, 1.8), 0.0, 0.0, 0.0);
  c.lidar.async = false;
  c.lidar.num_threads = 2;
  c.map.async = false;
  return c;
}

}  // namespace gll::sim
