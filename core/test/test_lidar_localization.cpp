// LiDAR を含む統合テスト（設計書 9 章）。合成環境のレイキャストで LiDAR を模擬し、Localizer 全体を動かす。
//  - 地図だけの環境で、ずれた初期姿勢から地図上で初期化して走る
//  - 地図1 ↔ GNSS ↔ 地図2 を走る（アンカーの誤差を含む）
//  - LiDAR が止まってずれた後、再位置推定で戻る
//  - 地図の近くで初期姿勢を与えたが LiDAR が来ない（地図上の初期化をあきらめる）
#include "gll/common/pose_store.hpp"
#include "gll/measurement/lidar_measurement_builder.hpp"

#include "lidar_sim.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <map>
#include <set>

using namespace gll;
using namespace gll::sim;

namespace {

struct Errors {
  double pos_rms = 0.0, pos_max = 0.0, yaw_rms_deg = 0.0, out_pos_rms = 0.0, max_step = 0.0;
  int n = 0;
};

/// t0 <= t < t1 の記録について、生の推定値と出力の誤差、出力の補正ステップ（真値の移動を除いた 1 周期の変化）を求める。
Errors evaluate(const SimRun& run, double t0, double t1) {
  Errors e;
  const SimRecord* prev = nullptr;
  for (const auto& r : run.records) {
    if (r.t < t0 || r.t >= t1 || r.out.status == LocalizationStatus::INITIALIZING) {
      prev = nullptr;
      continue;
    }
    const double ex = r.out.raw_pose.x - r.truth.x, ey = r.out.raw_pose.y - r.truth.y;
    const double d = std::hypot(ex, ey);
    e.pos_rms += d * d;
    e.pos_max = std::max(e.pos_max, d);
    const double dy = rad2deg(wrapAngle(r.out.raw_pose.yaw - r.truth.yaw));
    e.yaw_rms_deg += dy * dy;
    const double eo = std::hypot(r.out.pose.x - r.truth.x, r.out.pose.y - r.truth.y);
    e.out_pos_rms += eo * eo;
    if (prev) {
      const Vec2 dout(r.out.pose.x - prev->out.pose.x, r.out.pose.y - prev->out.pose.y);
      const Vec2 dtruth(r.truth.x - prev->truth.x, r.truth.y - prev->truth.y);
      e.max_step = std::max(e.max_step, (dout - dtruth).norm());
    }
    prev = &r;
    ++e.n;
  }
  if (e.n > 0) {
    e.pos_rms = std::sqrt(e.pos_rms / e.n);
    e.yaw_rms_deg = std::sqrt(e.yaw_rms_deg / e.n);
    e.out_pos_rms = std::sqrt(e.out_pos_rms / e.n);
  }
  return e;
}

void print(const char* name, const Errors& e) {
  std::printf("[%s] n=%d raw pos RMS %.3f m (max %.3f), yaw RMS %.3f deg, output pos RMS %.3f m, max step %.4f m\n",
              name, e.n, e.pos_rms, e.pos_max, e.yaw_rms_deg, e.out_pos_rms, e.max_step);
}

/// campusWorld の (0, 5) を中心とする半径 18 m の円を反時計回りに走る。
MotionProfile circleRoute() {
  MotionProfile mp;
  mp.start = Vec2(18.0, 5.0);
  mp.start_yaw = kPi / 2.0;
  mp.v_max = 1.5;
  mp.accel = [](double t, double) { return (t >= 3.0 && t < 4.0) ? 1.5 : 0.0; };
  mp.yaw_rate = [](double, double v) { return v / 18.0; };
  return mp;
}

}  // namespace

TEST(LidarMeasurementBuilder, CovarianceQualityAndConversion) {
  LidarConfig c;
  const LidarMeasurementBuilder b(c);
  RegistrationResult r;
  r.converged = true;
  r.num_source = 1000;
  r.num_inliers = 900;
  r.inlier_ratio = 0.9;
  r.overlap = 0.9;
  // x（並進 0）方向の情報が少ない H（回転 → 並進の順）
  r.H = Mat6::Identity() * 1e6;
  r.H(3, 3) = 1e3;
  const Mat3 cov = b.covarianceBody(r);
  EXPECT_GT(cov(0, 0), 100.0 * cov(1, 1));
  EXPECT_NEAR(cov(1, 1), c.cov_scale * 900 / 1e6 + c.min_stddev_xy * c.min_stddev_xy, 1e-9);

  // アンカーで UTM に変換される（地図の x 軸が東から 90° → 地図の (10, 0) は UTM の +y 方向）
  class T : public MatchTarget {
   public:
    std::optional<double> groundHeight(double, double, double, double) const override { return 0.0; }
    std::vector<Vec3f> samplePoints(std::size_t) const override { return {}; }
  } target;
  AnchorConfig ac;
  ac.use_utm = true;
  ac.easting = 500000;
  ac.northing = 4000000;
  ac.grid_heading = 0.0;
  ac.use_scale_factor = false;
  target.anchor = MapAnchor::fromConfig(ac, UtmProjector(54, true));
  target.group = "g";
  r.T_map_base = makePose(Vec3(10, 0, 0), 0, 0, 0.1);
  const auto ok = b.build(5.0, r, target, makePose(Vec3(10.2, 0, 0), 0, 0, 0.1));
  ASSERT_TRUE(ok.pose.has_value());
  EXPECT_NEAR(ok.pose->Z.t().x(), 500000.0, 1e-6);
  EXPECT_NEAR(ok.pose->Z.t().y(), 4000010.0, 1e-6);
  EXPECT_NEAR(ok.pose->Z.yaw(), 0.1 + kPi / 2.0, 1e-9);
  EXPECT_EQ(ok.pose->map_group, "g");
  // 初期値から 1 m 以上動いた結果は棄却する
  EXPECT_EQ(b.build(5.0, r, target, makePose(Vec3(11.5, 0, 0), 0, 0, 0.1)).reason, LidarRejectReason::JUMP);
  r.overlap = 0.3;
  r.overlap_near = 0.8;
  EXPECT_EQ(b.build(5.0, r, target, r.T_map_base).reason, LidarRejectReason::LOW_OVERLAP);
  // overlap_near_distance を決めると、地図の近くにある点だけで数えた overlap で判定する
  LidarConfig cn = c;
  cn.overlap_near_distance = 1.0;
  const LidarMeasurementBuilder bn(cn);
  EXPECT_EQ(bn.build(5.0, r, target, r.T_map_base).reason, LidarRejectReason::NONE);
  r.overlap_near = 0.4;
  EXPECT_EQ(bn.build(5.0, r, target, r.T_map_base).reason, LidarRejectReason::LOW_OVERLAP);
}

TEST(LidarMeasurementBuilder, SeparateLongitudinalAndLateralFloors) {
  LidarConfig c;
  c.min_stddev_lon = 0.06;  // 縦だけ下限を上げる。横は min_stddev_xy のまま
  const LidarMeasurementBuilder b(c);
  RegistrationResult r;
  r.num_inliers = 900;
  r.H = Mat6::Identity() * 1e9;  // H からの共分散はほぼ 0
  const Mat3 cov = b.covarianceBody(r);
  EXPECT_NEAR(cov(0, 0), 0.06 * 0.06, 1e-6);
  EXPECT_NEAR(cov(1, 1), c.min_stddev_xy * c.min_stddev_xy, 1e-6);
}

TEST(PoseStore, SaveAndLoad) {
  const std::string dir = makeTempDir("posestore");
  const std::string path = dir + "/last_pose.txt";
  EXPECT_FALSE(loadPose(path).has_value());
  ASSERT_TRUE(savePose(path, SavedPose{123.25, Pose2D{386123.456789, 3952000.125, -2.5}}));
  const auto p = loadPose(path);
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(p->t, 123.25);
  EXPECT_DOUBLE_EQ(p->pose.x, 386123.456789);
  EXPECT_DOUBLE_EQ(p->pose.yaw, -2.5);
  std::filesystem::remove_all(dir);
}

TEST(LidarLocalization, MapOnlyFromRoughInitialPose) {
  // GNSS 無し。起動後、1 m・12° ずれた初期姿勢を与え、地図上で初期化してから円を走る
  const LocalizerConfig lc = lidarTestConfig();
  const std::string dir = makeTempDir("maponly");
  LidarSimulator sim(campusWorld(), {SimGroup{"campus"}}, lc, dir);
  Localizer loc(lc, nullptr, std::make_shared<StderrLogger>());
  loc.setMap(sim.makeMapManager(), std::make_shared<GicpMatcher>(lc.lidar, lc.relocalize));
  SensorPlan sp;
  sp.duration = 60.0;
  sp.init_time = 2.5;
  sp.init_error = Pose2D{0.8, -0.6, deg2rad(12.0)};
  const SimRun run = sim.run(loc, circleRoute(), sp);

  ASSERT_GT(run.ready_time, 0.0);
  EXPECT_LT(run.ready_time, 4.0);
  const Errors e = evaluate(run, run.ready_time, 1e9);
  print("map only", e);
  EXPECT_LT(e.pos_rms, 0.05);
  EXPECT_LT(e.pos_max, 0.15);
  EXPECT_LT(e.yaw_rms_deg, 0.3);
  EXPECT_LT(e.max_step, 0.005);
  int lidar_aided = 0, total = 0;
  double max_dr = 0.0;
  for (const auto& r : run.records) {
    if (r.t < run.ready_time) continue;
    ++total;
    if (r.out.status == LocalizationStatus::LIDAR_AIDED) ++lidar_aided;
    max_dr = std::max(max_dr, r.out.dr_distance);
    EXPECT_FALSE(r.out.dr_distance_exceeded);
    EXPECT_EQ(r.out.active_map_group, "campus");
  }
  std::printf("[map only] LIDAR_AIDED %.1f %%, max DR distance %.2f m, accepted %zu / matched %zu, init attempts %zu\n",
              100.0 * lidar_aided / total, max_dr, run.diag.lidar_accepted, run.diag.lidar_matched,
              run.diag.map_init_attempts);
  EXPECT_GT(lidar_aided, 0.95 * total);
  EXPECT_LT(max_dr, 1.0);
  EXPECT_GT(run.diag.lidar_accepted, 500u);
  std::filesystem::remove_all(dir);
}

namespace {

/// 道路（y = 0 付近を東へ）の両側に建物がある 2 つの区域 A（x < 35）と B（x > 115）。間の 80 m は地面だけ。
World twoAreaWorld() {
  World w;
  w.ground_min = Vec2(-70, -45);
  w.ground_max = Vec2(215, 45);
  // 区域 A
  w.boxes.push_back({Vec3(-30, 14, 4), Vec3(18, 8, 8), 0.05});
  w.boxes.push_back({Vec3(-5, 16, 3), Vec3(10, 10, 6), -0.15});
  w.boxes.push_back({Vec3(15, 13, 5), Vec3(14, 6, 10), 0.0});
  w.boxes.push_back({Vec3(-20, -14, 3.5), Vec3(22, 8, 7), 0.1});
  w.boxes.push_back({Vec3(10, -16, 2.5), Vec3(8, 12, 5), 0.4});
  w.boxes.push_back({Vec3(28, -12, 1.0), Vec3(2, 6, 2), -0.3});
  w.boxes.push_back({Vec3(-8, 6.5, 0.8), Vec3(4.5, 1.8, 1.6), 0.0});
  for (int i = 0; i < 6; ++i) w.cylinders.push_back({Vec2(-40 + 13 * i + 2 * (i % 2), -7.5), 0.2, 0.0, 5.0});
  // 区域 B
  w.boxes.push_back({Vec3(125, 15, 6), Vec3(12, 10, 12), 0.25});
  w.boxes.push_back({Vec3(150, 12, 3), Vec3(20, 6, 6), 0.0});
  w.boxes.push_back({Vec3(178, 18, 4), Vec3(10, 16, 8), -0.2});
  w.boxes.push_back({Vec3(135, -14, 2.5), Vec3(16, 10, 5), -0.1});
  w.boxes.push_back({Vec3(165, -13, 4.5), Vec3(8, 8, 9), 0.6});
  w.boxes.push_back({Vec3(185, -9, 0.75), Vec3(4.2, 1.8, 1.5), 1.2});
  for (int i = 0; i < 5; ++i) w.cylinders.push_back({Vec2(120 + 14 * i + 3 * (i % 3), 8.0), 0.3, 0.0, 4.0});
  return w;
}

}  // namespace

TEST(LidarLocalization, MapGnssMapWithAnchorError) {
  // 地図1（グループ A）→ GNSS 区間（80 m）→ 地図2（グループ B。座標系は 40° 回っていて、アンカーに 10 cm・0.2° の誤差）
  LocalizerConfig lc = lidarTestConfig();
  const std::string dir = makeTempDir("mapgnssmap");
  SimGroup ga{"A"};
  ga.contains = [](const Vec3& p) { return p.x() < 50.0; };
  SimGroup gb{"B"};
  gb.T_world_group = makePose(Vec3(150, 0, 0), 0, 0, deg2rad(40.0));
  gb.contains = [](const Vec3& p) { return p.x() > 100.0; };
  gb.anchor_error = Pose2D{0.10, -0.05, deg2rad(0.2)};
  LidarSimulator sim(twoAreaWorld(), {ga, gb}, lc, dir);
  Localizer loc(lc, nullptr, std::make_shared<StderrLogger>());
  loc.setMap(sim.makeMapManager(std::make_shared<StderrLogger>()), std::make_shared<GicpMatcher>(lc.lidar, lc.relocalize));

  MotionProfile mp;
  mp.start = Vec2(-25.0, 0.0);
  mp.start_yaw = 0.0;
  mp.v_max = 1.6;
  mp.accel = [](double t, double) { return (t >= 3.0 && t < 4.0) ? 1.6 : 0.0; };
  mp.yaw_rate = [](double t, double) { return 0.03 * std::sin(2 * kPi * t / 25.0); };  // ゆるい S 字
  SensorPlan sp;
  sp.duration = 125.0;
  sp.lidar_rate = 5.0;
  sp.gnss_fix = [](double, const Vec2& p) { return p.x() > 25.0 && p.x() < 125.0; };  // 地図の縁と重なる
  sp.init_time = 2.5;
  sp.init_error = Pose2D{0.5, 0.4, deg2rad(8.0)};
  const SimRun run = sim.run(loc, mp, sp);
  ASSERT_GT(run.ready_time, 0.0);

  // 区間ごとの誤差（世界座標の x で分ける）
  const auto segment = [&](double x0, double x1) {
    SimRun sub;
    for (const auto& r : run.records)
      if (r.world_xy.x() >= x0 && r.world_xy.x() < x1) sub.records.push_back(r);
    return evaluate(sub, run.ready_time, 1e9);
  };
  const Errors ea = segment(-100, 25), eg = segment(35, 115), eb = segment(125, 1e9), all = evaluate(run, run.ready_time, 1e9);
  print("A (LiDAR)", ea);
  print("gap (GNSS)", eg);
  print("B (LiDAR)", eb);
  print("all", all);
  EXPECT_LT(ea.pos_rms, 0.05);
  EXPECT_LT(eg.pos_rms, 0.05);
  // B では、推定値は B の地図（アンカーの誤差 10 cm・0.2° を含む）に合う
  EXPECT_LT(eb.pos_rms, 0.25);
  // 地図の切り替え・GNSS との引き継ぎでも、出力は飛ばない（FR-4）
  EXPECT_LT(all.max_step, 0.005);

  std::vector<std::string> groups;
  std::set<LocalizationStatus> statuses;
  for (const auto& r : run.records) {
    if (r.t < run.ready_time) continue;
    if (groups.empty() || groups.back() != r.out.active_map_group) groups.push_back(r.out.active_map_group);
    statuses.insert(r.out.status);
    EXPECT_FALSE(r.out.dr_distance_exceeded);
  }
  std::string seq;
  for (const auto& g : groups) seq += (g.empty() ? "-" : g) + " ";
  std::printf("[map-gnss-map] active groups: %s\n", seq.c_str());
  EXPECT_EQ(groups, (std::vector<std::string>{"A", "B"}));
  EXPECT_TRUE(statuses.count(LocalizationStatus::LIDAR_AIDED));
  EXPECT_TRUE(statuses.count(LocalizationStatus::GNSS_LIDAR_AIDED));
  EXPECT_TRUE(statuses.count(LocalizationStatus::GNSS_AIDED));

  // GNSS FIX 中の LiDAR との差が、グループ B のアンカーずれとして記録される（設計書 3.13.2 節）
  ASSERT_TRUE(run.diag.anchor_mismatch.count("B"));
  const MismatchStats& mb = run.diag.anchor_mismatch.at("B");
  std::printf("[map-gnss-map] anchor mismatch B: n=%zu mean (%.3f, %.3f) m, %.3f deg; A: n=%zu; lidar reanchor %zu, "
              "gnss accepted %zu, lidar accepted %zu, mismatch rejected %zu\n",
              mb.count, mb.mean_world.x(), mb.mean_world.y(), rad2deg(mb.mean_world.z()),
              run.diag.anchor_mismatch.count("A") ? run.diag.anchor_mismatch.at("A").count : 0,
              run.diag.lidar_reanchor_count, run.diag.gnss_accepted, run.diag.lidar_accepted, run.diag.lidar_mismatch);
  EXPECT_GT(mb.count, 10u);
  EXPECT_GT(mb.mean_world.head<2>().norm(), 0.05);
  std::filesystem::remove_all(dir);
}

TEST(LidarLocalization, RelocalizesAfterLidarBlackout) {
  // 地図だけの環境で LiDAR が 25 s 止まり、その間 ODOM の速度が 5 % 大きく出る → 約 2 m ずれる。
  // デッドレコニング距離が 30 m を超えて ERROR になり、LiDAR が戻ったら再位置推定で戻る
  const LocalizerConfig lc = lidarTestConfig();
  const std::string dir = makeTempDir("blackout");
  LidarSimulator sim(campusWorld(), {SimGroup{"campus"}}, lc, dir);
  Localizer loc(lc, nullptr, std::make_shared<StderrLogger>());
  loc.setMap(sim.makeMapManager(), std::make_shared<GicpMatcher>(lc.lidar, lc.relocalize));
  SensorPlan sp;
  sp.duration = 90.0;
  sp.init_time = 2.5;
  sp.init_error = Pose2D{0.3, 0.3, deg2rad(5.0)};
  sp.lidar_on = [](double t) { return t < 20.0 || t >= 45.0; };
  sp.odom_scale_error = [](double t) { return (t >= 20.0 && t < 45.0) ? 0.05 : 0.0; };
  const SimRun run = sim.run(loc, circleRoute(), sp);
  ASSERT_GT(run.ready_time, 0.0);

  double err_at_45 = 0.0, first_error_t = -1.0, clear_t = -1.0;
  for (const auto& r : run.records) {
    if (r.t >= 44.9 && r.t < 45.0) err_at_45 = std::hypot(r.out.raw_pose.x - r.truth.x, r.out.raw_pose.y - r.truth.y);
    if (r.out.dr_distance_exceeded && first_error_t < 0.0) first_error_t = r.t;
    if (first_error_t > 0.0 && !r.out.dr_distance_exceeded && clear_t < 0.0) clear_t = r.t;
  }
  const Errors before = evaluate(run, run.ready_time, 20.0), after = evaluate(run, 60.0, 1e9);
  std::printf("[blackout] error at 45 s %.2f m, DR error %.1f s -> %.1f s, relocalize %zu / %zu, reanchor %zu\n",
              err_at_45, first_error_t, clear_t, run.diag.relocalize_success, run.diag.relocalize_attempts,
              run.diag.lidar_reanchor_count);
  print("before blackout", before);
  print("after recovery", after);
  EXPECT_GT(err_at_45, 0.8);                 // 止まっている間に大きくずれる
  EXPECT_GT(first_error_t, 20.0);            // 30 m 走ったところで ERROR
  EXPECT_LT(first_error_t, 45.0);
  EXPECT_GT(clear_t, 45.0);                  // LiDAR で位置が戻ると解除される
  EXPECT_LT(clear_t, 55.0);
  EXPECT_LT(after.pos_rms, 0.05);            // 生の推定値は元に戻る
  EXPECT_LT(after.max_step, 0.005);          // 出力はレート制限でゆっくり戻る（飛ばない）
  EXPECT_GE(run.diag.relocalize_success + run.diag.lidar_reanchor_count, 1u);
  std::filesystem::remove_all(dir);
}

TEST(LidarLocalization, MapInitTimesOutWithoutLidar) {
  // 地図の近くで初期姿勢を与えたが、LiDAR のデータが来ない（設計書 3.11 節の init_timeout）。
  //  - 保存した位置（SAVED）: 地図上の初期化をあきらめ、GNSS で初期化する（GNSS の初期化を止めたままにしない）
  //  - 外部から与えた初期姿勢（EXTERNAL）: 与えた姿勢でそのまま初期化する
  for (const auto source : {Localizer::InitialPoseSource::SAVED, Localizer::InitialPoseSource::EXTERNAL}) {
    const bool saved = source == Localizer::InitialPoseSource::SAVED;
    SCOPED_TRACE(saved ? "SAVED" : "EXTERNAL");
    LocalizerConfig lc = lidarTestConfig();
    lc.relocalize.init_timeout = 5.0;
    const std::string dir = makeTempDir(saved ? "timeout_saved" : "timeout_external");
    LidarSimulator sim(campusWorld(), {SimGroup{"campus"}}, lc, dir);
    Localizer loc(lc, nullptr, std::make_shared<StderrLogger>());
    loc.setMap(sim.makeMapManager(), std::make_shared<GicpMatcher>(lc.lidar, lc.relocalize));
    SensorPlan sp;
    sp.duration = 30.0;
    sp.init_time = 0.5;
    sp.init_source = source;
    sp.init_error = Pose2D{0.03, -0.02, deg2rad(0.5)};
    sp.init_cov = Vec3(0.05 * 0.05, 0.05 * 0.05, deg2rad(1.0) * deg2rad(1.0)).asDiagonal();
    sp.lidar_on = [](double) { return false; };
    sp.gnss_fix = [saved](double, const Vec2&) { return saved; };  // SAVED のときだけ GNSS がある
    MotionProfile mp = circleRoute();
    mp.accel = [](double t, double) { return (t >= 12.0 && t < 13.0) ? 1.0 : 0.0; };  // 12 s までは止まっている
    const SimRun run = sim.run(loc, mp, sp);

    EXPECT_EQ(run.diag.lidar_count, 0u);
    EXPECT_EQ(run.diag.map_init_attempts, 0u);
    ASSERT_FALSE(run.records.empty());
    const double first_output = run.records.front().t;
    std::printf("[init timeout, %s] first output %.2f s, ready %.2f s\n", saved ? "SAVED" : "EXTERNAL", first_output,
                run.ready_time);
    // 姿勢推定器の静止初期化（3 s）の後、init_timeout（5 s）までは地図上での初期化を待つ
    EXPECT_GT(first_output, 7.9);
    if (saved) {
      // GNSS で初期化する（走り出して 1 m 進んでから）
      EXPECT_GT(first_output, 12.0);
      ASSERT_GT(run.ready_time, 0.0);
      const Errors e = evaluate(run, run.ready_time, 1e9);
      print("init timeout, SAVED -> GNSS", e);
      EXPECT_LT(e.pos_rms, 0.1);
    } else {
      // 与えた姿勢でそのまま初期化する
      EXPECT_LT(first_output, 8.5);
      const SimRecord& r0 = run.records.front();
      EXPECT_LT(std::hypot(r0.out.raw_pose.x - r0.truth.x, r0.out.raw_pose.y - r0.truth.y), 0.1);
    }
    std::filesystem::remove_all(dir);
  }
}
