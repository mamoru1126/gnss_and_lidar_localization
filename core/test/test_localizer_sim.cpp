// 合成データによる統合テスト（設計書 9 章「シミュレーション」）。
// 真値の走行から IMU・ODOM・GNSS を生成し、Localizer に流して、収束・連続性・再アンカーを確かめる。
#include "gll/estimation/es_ekf_2d.hpp"
#include "gll/localizer.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <deque>
#include <random>
#include <vector>

using namespace gll;

namespace {

struct SimConfig {
  double duration = 60.0;
  double gnss_outage_start = -1.0;  ///< GNSS を止める区間
  double gnss_outage_end = -1.0;
  double gnss_jump_after = -1.0;    ///< この時刻以降、GNSS に一定のずれを加える
  Vec2 gnss_jump = Vec2::Zero();
  int gnss_status = 2;
  double gnss_delay = 0.05;         ///< GNSS が届くまでの遅延 [s]
  bool stop_at_end = true;
  unsigned seed = 7;
};

struct Record {
  double t;
  Pose2D truth;
  LocalizationOutput out;
};

struct SimResult {
  std::vector<Record> records;
  Diagnostics diag;
  double ready_time = -1.0;
};

class Simulator {
 public:
  Simulator(const SimConfig& sc, const LocalizerConfig& lc) : sc_(sc), lc_(lc), rng_(sc.seed) {}

  SimResult run(Localizer& loc, const std::optional<Pose2D>& external_init = std::nullopt,
                const Mat3& external_cov = Mat3::Identity()) {
    SimResult res;
    const UtmProjector proj(lc_.gnss.utm_zone, lc_.gnss.utm_north);
    const double dt = 0.01;
    double x = 450000.0, y = 3950000.0, yaw = 0.4, v = 0.0;
    const double s_true = 1.02;
    const Vec3 gyro_bias(0.002, -0.001, 0.003);
    std::normal_distribution<double> n(0.0, 1.0);
    std::deque<std::pair<double, GnssSample>> gnss_queue;  // (配送時刻, サンプル)
    bool init_sent = false;

    for (int k = 0; k * dt <= sc_.duration + 1e-9; ++k) {
      const double t = k * dt;
      // --- 真値の運動 ---
      double a = 0.0;
      if (t >= 3.0 && t < 4.0) a = 1.5;
      if (sc_.stop_at_end && t >= sc_.duration - 10.0 && t < sc_.duration - 9.0) a = -1.5;
      double w = 0.0;
      if (t > 6.0 && !(sc_.stop_at_end && t >= sc_.duration - 10.0)) w = 0.2 * std::sin(2 * kPi * (t - 6.0) / 20.0);
      if (k > 0) {
        v = std::max(0.0, v + a * dt);
        const double ym = yaw + 0.5 * w * dt;
        x += v * dt * std::cos(ym);
        y += v * dt * std::sin(ym);
        yaw = wrapAngle(yaw + w * dt);
      }
      // --- IMU（100 Hz）---
      ImuSample imu;
      imu.t = t;
      imu.gyro = Vec3(0, 0, w) + gyro_bias + 0.002 * Vec3(n(rng_), n(rng_), n(rng_));
      imu.acc = Vec3(a, v * w, kGravity) + 0.02 * Vec3(n(rng_), n(rng_), n(rng_));
      loc.addImu(imu);
      // --- ODOM（50 Hz）---
      if (k % 2 == 0) {
        OdomSample o;
        o.t = t;
        // 車輪エンコーダは停止中は 0 を返すので、動いているときだけノイズを乗せる
        o.v = (v > 1e-6) ? v / s_true + 0.01 * n(rng_) : 0.0;
        o.yaw_rate = w + 0.005 * n(rng_);
        loc.addOdom(o);
      }
      // --- GNSS（10 Hz、遅延あり）---
      if (k % 10 == 0) {
        const bool outage = t >= sc_.gnss_outage_start && t < sc_.gnss_outage_end;
        if (!outage) {
          const Vec2 lever = lc_.gnss.lever_arm.head<2>();  // 水平面なので傾きなし
          Vec2 ant = Vec2(x, y) + SE2::rot(yaw) * lever + 0.015 * Vec2(n(rng_), n(rng_));
          if (sc_.gnss_jump_after >= 0.0 && t >= sc_.gnss_jump_after) ant += sc_.gnss_jump;
          const LatLon ll = proj.inverse(ant.x(), ant.y());
          GnssSample g;
          g.t = t;
          g.lat = ll.lat;
          g.lon = ll.lon;
          g.h = 40.0;
          g.raw_status = sc_.gnss_status;
          g.cov_known = true;
          g.cov_enu = Mat3::Identity() * 0.015 * 0.015;
          gnss_queue.emplace_back(t + sc_.gnss_delay, g);
        }
      }
      while (!gnss_queue.empty() && gnss_queue.front().first <= t + 1e-9) {
        loc.addGnss(gnss_queue.front().second);
        gnss_queue.pop_front();
      }
      if (external_init && !init_sent && t >= 4.5) {
        loc.setInitialPose(t, *external_init, external_cov);
        init_sent = true;
      }
      // --- 出力 ---
      if (const auto out = loc.getOutput()) {
        if (out->status != LocalizationStatus::INITIALIZING && res.ready_time < 0.0) res.ready_time = t;
        res.records.push_back(Record{t, Pose2D{x, y, yaw}, *out});
      }
    }
    res.diag = loc.diagnostics();
    return res;
  }

 private:
  SimConfig sc_;
  LocalizerConfig lc_;
  std::mt19937 rng_;
};

LocalizerConfig baseConfig() {
  LocalizerConfig c;
  c.gnss.lever_arm = Vec3(0.3, 0.2, 0.5);
  return c;
}

struct Metrics {
  double pos_rms = 0.0, yaw_rms_deg = 0.0, max_step_err = 0.0;
  int n = 0;
};

// t in [t0, t1) の出力について、真値との誤差と、出力の 1 周期あたりの補正ステップを評価する。
Metrics evaluate(const SimResult& r, double t0, double t1, const Vec2& truth_offset = Vec2::Zero()) {
  Metrics m;
  double se = 0.0, sy = 0.0;
  const Record* prev = nullptr;
  for (const auto& rec : r.records) {
    if (rec.out.status == LocalizationStatus::INITIALIZING) {
      prev = nullptr;
      continue;
    }
    if (rec.t >= t0 && rec.t < t1) {
      const Vec2 e = Vec2(rec.out.pose.x - rec.truth.x, rec.out.pose.y - rec.truth.y) - truth_offset;
      se += e.squaredNorm();
      sy += std::pow(wrapAngle(rec.out.pose.yaw - rec.truth.yaw), 2);
      ++m.n;
    }
    if (prev) {
      // 出力の変化と真値の変化の差（= 補正によるステップ）
      const Vec2 d_out(rec.out.pose.x - prev->out.pose.x, rec.out.pose.y - prev->out.pose.y);
      const Vec2 d_true(rec.truth.x - prev->truth.x, rec.truth.y - prev->truth.y);
      m.max_step_err = std::max(m.max_step_err, (d_out - d_true).norm());
    }
    prev = &rec;
  }
  if (m.n > 0) {
    m.pos_rms = std::sqrt(se / m.n);
    m.yaw_rms_deg = rad2deg(std::sqrt(sy / m.n));
  }
  return m;
}

}  // namespace

TEST(LocalizerSim, NominalGnssAndDeadReckoning) {
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc);
  SimConfig sc;
  const SimResult r = Simulator(sc, lc).run(loc);

  ASSERT_GT(r.ready_time, 0.0) << "never became ready";
  EXPECT_LT(r.ready_time, 15.0);
  const Metrics m = evaluate(r, 20.0, 60.0);
  ASSERT_GT(m.n, 1000);
  EXPECT_LT(m.pos_rms, 0.03) << "pos rms " << m.pos_rms;
  EXPECT_LT(m.yaw_rms_deg, 0.5) << "yaw rms " << m.yaw_rms_deg;
  EXPECT_LT(m.max_step_err, 0.01) << "max step " << m.max_step_err;  // 出力が飛ばない（FR-4）
  std::printf("[nominal] ready=%.2fs pos_rms=%.4fm yaw_rms=%.3fdeg max_step=%.5fm zaru=%zu\n", r.ready_time,
              m.pos_rms, m.yaw_rms_deg, m.max_step_err, r.diag.zaru_accepted);
  EXPECT_GT(r.diag.gnss_accepted, 300u);
  EXPECT_GT(r.diag.zaru_accepted, 0u);  // 最後の停止区間で ZARU が働く

  // ジャイロバイアスと ODOM スケールが推定されている
  const auto st = loc.latestState();
  ASSERT_TRUE(st.has_value());
  EXPECT_NEAR(st->b, 0.003, 0.001);
  EXPECT_NEAR(st->s, 1.02, 0.01);
}

TEST(LocalizerSim, OutageThenOffsetTriggersReanchorWithoutJump) {
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc);
  SimConfig sc;
  sc.gnss_outage_start = 30.0;
  sc.gnss_outage_end = 33.0;
  sc.gnss_jump_after = 33.0;  // GNSS 復帰後に 0.6 m のずれ（アンカーずれ・誤 FIX を想定）
  sc.gnss_jump = Vec2(0.6, 0.0);
  const SimResult r = Simulator(sc, lc).run(loc);

  ASSERT_GT(r.ready_time, 0.0);
  std::printf("[reanchor] reanchors=%zu deferred=%zu\n", r.diag.reanchor_count, r.diag.gnss_deferred);
  EXPECT_GE(r.diag.reanchor_count, 1u);
  EXPECT_GT(r.diag.gnss_deferred, 0u);
  // 出力は 0.1 m/s でしか動かない（1 周期 0.01 s で 1 mm + 推定の速度誤差）
  const Metrics all = evaluate(r, 0.0, 60.0);
  EXPECT_LT(all.max_step_err, 0.01) << "max step " << all.max_step_err;
  // 最後は GNSS（ずれた側）に合っている
  const Metrics tail = evaluate(r, 50.0, 60.0, sc.gnss_jump);
  std::printf("[reanchor] max_step=%.5fm tail_pos_rms=%.4fm\n", all.max_step_err, tail.pos_rms);
  EXPECT_LT(tail.pos_rms, 0.05) << "tail pos rms " << tail.pos_rms;
}

TEST(LocalizerSim, NonFixGnssNeverInitializes) {
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc);
  SimConfig sc;
  sc.duration = 20.0;
  sc.gnss_status = 0;  // 単独測位
  const SimResult r = Simulator(sc, lc).run(loc);
  EXPECT_FALSE(r.diag.filter_initialized);
  EXPECT_GT(r.diag.gnss_reject_reasons.at("NOT_RTK_FIX"), 100u);
}

TEST(LocalizerSim, LargeInitialYawErrorConverges) {
  // 外部から 60° ずれた初期姿勢を与えても、Invariant EKF は収束する（アルゴリズム説明書 6 章）
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc);
  SimConfig sc;
  const double sig = deg2rad(60.0);
  Mat3 cov = Mat3::Zero();
  cov(0, 0) = cov(1, 1) = 0.5 * 0.5;
  cov(2, 2) = sig * sig;
  // t = 4.5 s の真値は (x ≈ 450000 + 1.5·cos 0.4 など) だが、位置は粗くてよい
  const Pose2D init{450000.5, 3950000.5, wrapAngle(0.4 + deg2rad(60.0))};
  const SimResult r = Simulator(sc, lc).run(loc, init, cov);
  ASSERT_GT(r.ready_time, 0.0);
  const Metrics m = evaluate(r, 25.0, 60.0);
  std::printf("[yaw60] ready=%.2fs pos_rms=%.4fm yaw_rms=%.3fdeg\n", r.ready_time, m.pos_rms, m.yaw_rms_deg);
  EXPECT_LT(m.yaw_rms_deg, 1.0) << "yaw rms " << m.yaw_rms_deg;
  EXPECT_LT(m.pos_rms, 0.05) << "pos rms " << m.pos_rms;
}

TEST(LocalizerSim, EsekfAlsoRunsThroughSameInterface) {
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc, std::make_unique<EsEkf2D>(lc.estimator));
  SimConfig sc;
  const SimResult r = Simulator(sc, lc).run(loc);
  ASSERT_GT(r.ready_time, 0.0);
  const Metrics m = evaluate(r, 20.0, 60.0);
  EXPECT_LT(m.pos_rms, 0.05);
}

TEST(LocalizerSim, RobustAcrossSeeds) {
  // 乱数の種を変えても、収束・連続性・再アンカーが安定していること
  for (unsigned seed = 1; seed <= 8; ++seed) {
    const LocalizerConfig lc = baseConfig();
    {
      Localizer loc(lc);
      SimConfig sc;
      sc.seed = seed;
      const SimResult r = Simulator(sc, lc).run(loc);
      const Metrics m = evaluate(r, 20.0, 60.0);
      EXPECT_LT(m.pos_rms, 0.03) << "seed " << seed;
      EXPECT_LT(m.yaw_rms_deg, 0.5) << "seed " << seed;
      EXPECT_LT(m.max_step_err, 0.01) << "seed " << seed;
    }
    {
      Localizer loc(lc);
      SimConfig sc;
      sc.seed = seed;
      sc.gnss_outage_start = 30.0;
      sc.gnss_outage_end = 33.0;
      sc.gnss_jump_after = 33.0;
      sc.gnss_jump = Vec2(0.6, 0.0);
      const SimResult r = Simulator(sc, lc).run(loc);
      EXPECT_GE(r.diag.reanchor_count, 1u) << "seed " << seed;
      EXPECT_LT(evaluate(r, 0.0, 60.0).max_step_err, 0.01) << "seed " << seed;
      EXPECT_LT(evaluate(r, 50.0, 60.0, sc.gnss_jump).pos_rms, 0.05) << "seed " << seed;
    }
  }
}

TEST(LocalizerSim, LongDeadReckoningRaisesError) {
  // GNSS が 30 秒（約 45 m）途切れると、デッドレコニング距離が閾値（既定 30 m）を超えてエラーになる。
  // GNSS が戻ると距離は 0 に戻り、エラーも消える（設計書 3.12 節）。
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc);
  SimConfig sc;
  sc.gnss_outage_start = 20.0;
  sc.gnss_outage_end = 50.0;
  const SimResult r = Simulator(sc, lc).run(loc);
  ASSERT_GT(r.ready_time, 0.0);

  double max_dr = 0.0, first_error_t = -1.0, first_error_dr = 0.0;
  bool error_before_outage = false, error_after_recovery = false;
  // デッドレコニング距離ごとの位置誤差（閾値の目安を知るため）
  std::vector<std::pair<double, double>> err_at;  // (dr_distance, pos error)
  double next_mark = 10.0;
  for (const auto& rec : r.records) {
    const auto& o = rec.out;
    if (o.status == LocalizationStatus::INITIALIZING) continue;
    max_dr = std::max(max_dr, o.dr_distance);
    if (o.dr_distance_exceeded && first_error_t < 0.0) {
      first_error_t = rec.t;
      first_error_dr = o.dr_distance;
    }
    if (rec.t < 20.0 && o.dr_distance_exceeded) error_before_outage = true;
    if (rec.t > 53.0 && o.dr_distance_exceeded) error_after_recovery = true;
    if (rec.t >= 20.0 && rec.t < 50.0 && o.dr_distance >= next_mark) {
      err_at.emplace_back(o.dr_distance, std::hypot(o.raw_pose.x - rec.truth.x, o.raw_pose.y - rec.truth.y));
      next_mark += 10.0;
    }
  }
  for (const auto& [d, e] : err_at) std::printf("[dead-reckoning] %.1f m -> position error %.3f m\n", d, e);
  std::printf("[dead-reckoning] error raised at t=%.2fs (%.1f m), max %.1f m\n", first_error_t, first_error_dr, max_dr);

  EXPECT_FALSE(error_before_outage);
  EXPECT_GT(first_error_t, 20.0);
  EXPECT_NEAR(first_error_dr, lc.monitor.dr_error_distance, 0.1);
  EXPECT_GT(max_dr, 40.0);  // 30 s × 1.5 m/s
  EXPECT_FALSE(error_after_recovery);
}

TEST(LocalizerSim, StandingStillDoesNotAccumulateDeadReckoning) {
  // 停止中は距離が増えない（GNSS なしで止まっていてもエラーにしない）
  const LocalizerConfig lc = baseConfig();
  Localizer loc(lc);
  SimConfig sc;
  sc.duration = 60.0;
  sc.gnss_outage_start = 52.0;  // 最後の停止区間（50 s 以降）で GNSS が途切れる
  sc.gnss_outage_end = 1e9;
  const SimResult r = Simulator(sc, lc).run(loc);
  double dr_end = -1.0;
  for (const auto& rec : r.records) dr_end = rec.out.dr_distance;
  EXPECT_LT(dr_end, 1.0);
}
