// スキャンマッチング（前処理・GICP・共分散・多仮説の探索）の単体テスト。合成環境のレイキャストで作ったスキャンを使う。
#include "tiled_pcd_map/map_anchor.hpp"
#include "tiled_pcd_map/map_tiler.hpp"
#include "gll/matching/gicp_matcher.hpp"
#include "gll/matching/scan_preprocessor.hpp"

#include "sim_world.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <chrono>
#include <stdexcept>
#include <cstdio>

using namespace gll;

namespace {

LidarConfig lidarConfig() {
  LidarConfig c;
  c.T_base_lidar = makePose(Vec3(0.5, 0.0, 1.8), 0.0, 0.0, 0.0);
  c.num_threads = 2;
  return c;
}

/// SE(3) の指数写像（ξ = [ω, v]）。
Eigen::Isometry3d se3Exp(const Vec3& w, const Vec3& v) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const double th = w.norm();
  if (th < 1e-12) {
    T.translation() = v;
    return T;
  }
  const Vec3 a = w / th;
  Mat3 K;
  K << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
  T.linear() = Eigen::AngleAxisd(th, a).toRotationMatrix();
  T.translation() = (Mat3::Identity() + (1 - std::cos(th)) / th * K + (th - std::sin(th)) / th * K * K) * v;
  return T;
}

struct Fixture {
  sim::World world;
  std::shared_ptr<GicpMatcher> matcher;
  std::shared_ptr<const MatchTarget> target;
};

std::shared_ptr<const MatchTarget> buildTarget(const GicpMatcher& m, const sim::World& w, const std::string& group) {
  TilerOptions opt;
  opt.voxel_size = 0.25;
  opt.num_threads = 2;
  const TilerResult r = tileMap(w.samplePoints(0.2, 0.01), opt, group);
  std::vector<std::shared_ptr<const TileData>> tiles;
  for (const auto& t : r.tiles) tiles.push_back(std::make_shared<TileData>(t));
  return m.buildTarget(group, MapAnchor::identity(), tiles);
}

const Fixture& campus() {
  static const Fixture f = [] {
    Fixture x;
    x.world = sim::campusWorld();
    x.matcher = std::make_shared<GicpMatcher>(lidarConfig());
    x.target = buildTarget(*x.matcher, x.world, "campus");
    return x;
  }();
  return f;
}

/// 静止した車両の base_link の姿勢から 1 スキャンを作る。
LidarScan staticScan(const sim::World& w, const Eigen::Isometry3d& T_map_base, unsigned seed = 3) {
  std::mt19937 rng(seed);
  const LidarConfig c = lidarConfig();
  return sim::simulateScan(
      w, sim::LidarModel(), [&](double) { return T_map_base * c.T_base_lidar; }, 0.0, rng);
}

double transErr(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation().head<2>() - b.translation().head<2>()).norm();
}
double yawErrDeg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return rad2deg(std::abs(wrapAngle(yawOf(a.linear()) - yawOf(b.linear()))));
}

}  // namespace

TEST(ScanPreprocessor, DeskewRemovesMotionDistortion) {
  // 1.5 m/s・1 rad/s で旋回しながら取ったスキャン。スキャン終了時刻の base_link に戻したときの誤差を比べる
  const sim::World w = sim::campusWorld();
  const LidarConfig c = lidarConfig();
  const Vec3 omega(0, 0, 1.0), vel(1.5, 0, 0);
  const Eigen::Isometry3d T_end = makePose(Vec3(2, 3, 0), 0, 0, 0.3);
  const sim::LidarModel model;
  const auto base_at = [&](double tau) { return T_end * se3Exp(omega * (tau - model.period), vel * (tau - model.period)); };
  std::mt19937 rng(5);
  sim::LidarModel noiseless = model;
  noiseless.range_noise = 0.0;
  const LidarScan scan = sim::simulateScan(w, noiseless, [&](double tau) { return base_at(tau) * c.T_base_lidar; }, 10.0, rng);
  ASSERT_GT(scan.points.size(), 5000u);

  // 真値: 各点を、測った時刻の姿勢で世界に戻し、スキャン終了時刻の base_link で表す
  std::vector<Vec3> truth;
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const double tau = scan.times[i] + model.period;
    truth.push_back(T_end.inverse() * base_at(tau) * c.T_base_lidar * scan.points[i].cast<double>());
  }
  LidarConfig nc = c;
  nc.min_range = 0.0;
  nc.max_range = 1e3;
  const auto fixed = ScanPreprocessor(nc).process(scan, ScanMotion{omega, vel});
  nc.deskew = false;
  const auto raw = ScanPreprocessor(nc).process(scan, ScanMotion{omega, vel});
  ASSERT_EQ(fixed.size(), truth.size());
  double e_fixed = 0, e_raw = 0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    e_fixed = std::max(e_fixed, (fixed[i].cast<double>() - truth[i]).norm());
    e_raw = std::max(e_raw, (raw[i].cast<double>() - truth[i]).norm());
  }
  std::printf("[deskew] max error: %.4f m (deskewed) vs %.3f m (raw)\n", e_fixed, e_raw);
  EXPECT_LT(e_fixed, 0.01);
  EXPECT_GT(e_raw, 1.0);  // 1 rad/s × 0.1 s = 5.7°。遠くの点は 1 m 以上ずれる
}

TEST(ScanPreprocessor, CropsRangeAndVehicleBox) {
  LidarConfig c = lidarConfig();
  c.crop_box_enabled = true;
  c.crop_box_min = Vec3(-1, -1, 0);
  c.crop_box_max = Vec3(2, 1, 2.5);
  LidarScan s;
  s.points = {Vec3f(0.3f, 0, 0), Vec3f(10, 0, 0), Vec3f(100, 0, 0), Vec3f(0.0f, 0.0f, -1.5f)};
  const auto out = ScanPreprocessor(c).process(s, ScanMotion());
  // 0.3 m は近すぎ、100 m は遠すぎ、(0, 0, -1.5) は base_link で (0.5, 0, 0.3) の車体の箱の中
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x(), 10.5f, 1e-5f);
  EXPECT_NEAR(out[0].z(), 1.8f, 1e-5f);
}

TEST(GicpMatcher, AlignsFromPerturbedInitialGuess) {
  const Fixture& f = campus();
  const Eigen::Isometry3d truth = makePose(Vec3(3.0, -2.0, 0.0), 0.0, 0.0, 0.6);
  const LidarScan scan = staticScan(f.world, truth);
  const auto pts = ScanPreprocessor(lidarConfig()).process(scan, ScanMotion());
  const auto src = f.matcher->prepareSource(pts, 0.5);
  ASSERT_GT(src->size(), 500u);
  const Eigen::Isometry3d init = makePose(Vec3(3.6, -2.4, 0.2), 0.01, -0.01, 0.6 + deg2rad(4.0));
  const auto t0 = std::chrono::steady_clock::now();
  const RegistrationResult r = f.matcher->align(*src, *f.target, init);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("[gicp] %zu source points, %d iterations, %.1f ms, inlier %.2f, overlap %.2f, error/pt %.3f\n",
              r.num_source, r.iterations, ms, r.inlier_ratio, r.overlap, r.error_per_point);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(transErr(r.T_map_base, truth), 0.02);
  EXPECT_LT(yawErrDeg(r.T_map_base, truth), 0.1);
  EXPECT_NEAR(r.T_map_base.translation().z(), 0.0, 0.03);
  EXPECT_GT(r.inlier_ratio, 0.9);
  EXPECT_GT(r.overlap, 0.9);
  // 情報行列は正定値
  EXPECT_GT(Eigen::SelfAdjointEigenSolver<Mat6>(r.H).eigenvalues().minCoeff(), 0.0);
}

TEST(GicpMatcher, OverlapNearMapIgnoresUnmappedObjects) {
  // 地図に無い物（ここでは高い所に浮いた壁）の点を足すと、overlap は下がるが、
  // 地図の近くにある点だけで数えた overlap（overlap_near_distance）はほとんど下がらない
  LidarConfig c = lidarConfig();
  c.overlap_near_distance = 1.0;
  const Fixture& f = campus();
  const GicpMatcher m(c);
  const Eigen::Isometry3d truth = makePose(Vec3(3.0, -2.0, 0.0), 0.0, 0.0, 0.6);
  const LidarScan scan = staticScan(f.world, truth);
  auto pts = ScanPreprocessor(c).process(scan, ScanMotion());
  const auto clean = m.prepareSource(pts, 0.5);
  const auto [ov_clean, near_clean] = m.overlapWithNear(*clean, *f.target, truth, c.overlap_distance, 1.0);
  for (double y = -6.0; y <= 6.0; y += 0.1)
    for (double z = 12.0; z <= 16.0; z += 0.1) pts.push_back(Vec3f(6.0f, static_cast<float>(y), static_cast<float>(z)));
  const auto cluttered = m.prepareSource(pts, 0.5);
  const auto [ov, near] = m.overlapWithNear(*cluttered, *f.target, truth, c.overlap_distance, 1.0);
  std::printf("[overlap] clean %.2f (near %.2f), with an unmapped wall %.2f (near %.2f)\n", ov_clean, near_clean, ov,
              near);
  EXPECT_NEAR(near_clean, ov_clean, 0.05);
  EXPECT_LT(ov, ov_clean - 0.1);
  EXPECT_GT(near, ov_clean - 0.05);
  // align も同じ値を返す。overlap_near_distance が 0 なら overlap_near は overlap と同じ
  const RegistrationResult r = m.align(*cluttered, *f.target, truth);
  EXPECT_GT(r.overlap_near, r.overlap + 0.1);
  const RegistrationResult r0 = f.matcher->align(*cluttered, *f.target, truth);
  EXPECT_DOUBLE_EQ(r0.overlap_near, r0.overlap);
}

TEST(GicpMatcher, VgicpAlignsFromPerturbedInitialGuess) {
  // lidar.registration: vgicp（ターゲットはボクセルごとのガウス分布）でも、GICP と同じ精度で合う
  const Fixture& f = campus();
  LidarConfig c = lidarConfig();
  c.registration = "vgicp";
  c.vgicp_voxel_size = 1.0;
  const GicpMatcher m(c);
  const auto target = buildTarget(m, f.world, "campus");
  const Eigen::Isometry3d truth = makePose(Vec3(3.0, -2.0, 0.0), 0.0, 0.0, 0.6);
  const auto pts = ScanPreprocessor(c).process(staticScan(f.world, truth), ScanMotion());
  const auto src = m.prepareSource(pts, 0.5);
  const Eigen::Isometry3d init = makePose(Vec3(3.6, -2.4, 0.2), 0.01, -0.01, 0.6 + deg2rad(4.0));
  (void)m.align(*src, *target, init);  // 1 回目はボクセル地図を作る時間を含む
  const auto t0 = std::chrono::steady_clock::now();
  const RegistrationResult r = m.align(*src, *target, init);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("[vgicp] %zu source points, %d iterations, %.1f ms, inlier %.2f, overlap %.2f\n", r.num_source,
              r.iterations, ms, r.inlier_ratio, r.overlap);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(transErr(r.T_map_base, truth), 0.05);
  EXPECT_LT(yawErrDeg(r.T_map_base, truth), 0.2);
  EXPECT_GT(r.inlier_ratio, 0.9);  // 地図の点との距離で数え直した値（GICP と同じ意味）
  EXPECT_GT(r.overlap, 0.9);
  EXPECT_GT(Eigen::SelfAdjointEigenSolver<Mat6>(r.H).eigenvalues().minCoeff(), 0.0);
}

TEST(GicpMatcher, RejectsUnknownRegistration) {
  LidarConfig c = lidarConfig();
  c.registration = "ndt";
  EXPECT_THROW(GicpMatcher{c}, std::invalid_argument);
}

TEST(GicpMatcher, CorridorIsDegenerateAlongTheWalls) {
  // x 方向に長い 2 枚の壁と地面だけの環境。壁に沿った方向（x）の情報がほとんど無い
  sim::World w;
  w.ground_min = Vec2(-100, -10);
  w.ground_max = Vec2(100, 10);
  w.boxes.push_back({Vec3(0, 3.5, 1.5), Vec3(200, 1, 3), 0.0});
  w.boxes.push_back({Vec3(0, -3.5, 1.5), Vec3(200, 1, 3), 0.0});
  const GicpMatcher m(lidarConfig());
  const auto target = buildTarget(m, w, "corridor");
  const Eigen::Isometry3d truth = makePose(Vec3(0, 0.3, 0), 0, 0, 0.05);
  const auto pts = ScanPreprocessor(lidarConfig()).process(staticScan(w, truth), ScanMotion());
  const auto src = m.prepareSource(pts, 0.5);
  const RegistrationResult r = m.align(*src, *target, truth);
  const Mat6 cov = static_cast<double>(r.num_inliers) * (r.H + 1e-6 * Mat6::Identity()).inverse();
  // 並びは回転 (0..2) → 並進 (3..5)
  std::printf("[corridor] sigma x %.3g, y %.3g, yaw %.3g\n", std::sqrt(cov(3, 3)), std::sqrt(cov(4, 4)),
              std::sqrt(cov(2, 2)));
  EXPECT_GT(cov(3, 3), 30.0 * cov(4, 4));
}

TEST(GicpMatcher, SearchFindsPoseFromRoughGuess) {
  const Fixture& f = campus();
  const Eigen::Isometry3d truth = makePose(Vec3(-8.0, 12.0, 0.0), 0.0, 0.0, -2.0);
  const auto pts = ScanPreprocessor(lidarConfig()).process(staticScan(f.world, truth), ScanMotion());
  PoseSearchRequest req;
  req.center = makePose(Vec3(-6.8, 11.1, 0.0), 0.0, 0.0, -2.0 + deg2rad(35.0));
  req.radius = 2.5;
  req.yaw_range = deg2rad(60.0);
  const auto t0 = std::chrono::steady_clock::now();
  const PoseSearchResult r = f.matcher->search(pts, *f.target, req);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("[search] %d hypotheses, %.0f ms, best overlap %.2f, second %.2f (%s)\n", r.num_hypotheses, ms,
              r.best_overlap, r.second_overlap, r.reason.c_str());
  ASSERT_TRUE(r.found) << r.reason;
  EXPECT_LT(transErr(r.best.T_map_base, truth), 0.03);
  EXPECT_LT(yawErrDeg(r.best.T_map_base, truth), 0.2);
}

TEST(GicpMatcher, SearchWithUnknownYaw) {
  const Fixture& f = campus();
  const Eigen::Isometry3d truth = makePose(Vec3(20.0, 10.0, 0.0), 0.0, 0.0, 1.0);
  const auto pts = ScanPreprocessor(lidarConfig()).process(staticScan(f.world, truth, 9), ScanMotion());
  PoseSearchRequest req;
  req.center = makePose(Vec3(20.8, 9.5, 0.0), 0.0, 0.0, 1.0 + deg2rad(150.0));
  req.radius = 1.5;
  req.yaw_range = kPi;
  const PoseSearchResult r = f.matcher->search(pts, *f.target, req);
  std::printf("[search 360] %d hypotheses, best overlap %.2f, second %.2f (%s)\n", r.num_hypotheses, r.best_overlap,
              r.second_overlap, r.reason.c_str());
  ASSERT_TRUE(r.found) << r.reason;
  EXPECT_LT(transErr(r.best.T_map_base, truth), 0.03);
  EXPECT_LT(yawErrDeg(r.best.T_map_base, truth), 0.2);
}

TEST(GicpMatcher, SymmetricRoomIsAmbiguous) {
  // 同じ壁に囲まれた正方形の部屋の中心。90° 回した姿勢でも同じスキャンになるので、一意に決まらない
  sim::World w;
  w.ground_min = Vec2(-12, -12);
  w.ground_max = Vec2(12, 12);
  for (int k = 0; k < 4; ++k) {
    const double a = k * kPi / 2;
    w.boxes.push_back({Vec3(10.5 * std::cos(a), 10.5 * std::sin(a), 1.5), Vec3(1, 22, 3), a});
  }
  const GicpMatcher m(lidarConfig());
  const auto target = buildTarget(m, w, "room");
  const Eigen::Isometry3d truth = makePose(Vec3(0, 0, 0), 0, 0, 0.0);
  const auto pts = ScanPreprocessor(lidarConfig()).process(staticScan(w, truth), ScanMotion());
  PoseSearchRequest req;
  req.center = truth;
  req.radius = 0.5;
  req.yaw_range = kPi;
  const PoseSearchResult r = m.search(pts, *target, req);
  std::printf("[room] best overlap %.2f, second %.2f (%s)\n", r.best_overlap, r.second_overlap, r.reason.c_str());
  EXPECT_FALSE(r.found);
  EXPECT_EQ(r.reason, "ambiguous");
}

TEST(GicpMatcher, GroundHeight) {
  const Fixture& f = campus();
  const auto z = f.target->groundHeight(0.0, 0.0, 2.0);
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, 0.0, 0.05);
  EXPECT_FALSE(f.target->groundHeight(500.0, 500.0, 2.0).has_value());
}

TEST(GicpMatcher, DISABLED_BenchCoarse) {
  const Fixture& f = campus();
  const Eigen::Isometry3d truth = makePose(Vec3(-8.0, 12.0, 0.0), 0.0, 0.0, -2.0);
  const auto pts = ScanPreprocessor(lidarConfig()).process(staticScan(f.world, truth), ScanMotion());
  const auto coarse = f.matcher->prepareSource(pts, 1.0);
  std::printf("coarse size %zu\n", coarse->size());
  f.matcher->alignCoarse(*coarse, *f.target, truth);  // voxel map build
  for (double off : {0.0, 1.0, 2.0}) {
    const Eigen::Isometry3d init = makePose(Vec3(-8.0 + off, 12.0, 0.0), 0.0, 0.0, -2.0 + deg2rad(10 * off));
    auto t0 = std::chrono::steady_clock::now();
    const auto r = f.matcher->alignCoarse(*coarse, *f.target, init);
    double ms1 = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    t0 = std::chrono::steady_clock::now();
    const double ov = f.matcher->overlap(*coarse, *f.target, r.T_map_base, 1.0);
    double ms2 = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("off %.0f: coarse %.2f ms (%d it, conv %d), overlap %.2f ms = %.2f, err %.3f\n", off, ms1, r.iterations, r.converged, ms2, ov, transErr(r.T_map_base, truth));
  }
}
