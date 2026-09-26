#include "gll_ros2/diagnostics.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace gll_ros2;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace {

std::string valueOf(const DiagStatus& st, const std::string& key) {
  for (const auto& v : st.values)
    if (v.key == key) return v.value;
  return "(none)";
}

gll::LocalizationOutput output(gll::LocalizationStatus status, double dr_distance, bool exceeded) {
  gll::LocalizationOutput o;
  o.status = status;
  o.cov = gll::Mat3::Identity() * 0.01;
  o.raw_cov = o.cov;
  o.dr_distance = dr_distance;
  o.dr_distance_exceeded = exceeded;
  return o;
}

}  // namespace

TEST(Diagnostics, InitializingIsWarn) {
  const DiagStatus st = makeLocalizationStatus(nullptr, gll::Diagnostics(), 30.0);
  EXPECT_EQ(st.level, DiagStatus::WARN);
  EXPECT_EQ(st.message, "INITIALIZING");
  EXPECT_EQ(valueOf(st, "init_phase"), "WAIT_FIX");
}

TEST(Diagnostics, AidedIsOk) {
  const auto o = output(gll::LocalizationStatus::GNSS_AIDED, 0.0, false);
  const DiagStatus st = makeLocalizationStatus(&o, gll::Diagnostics(), 30.0);
  EXPECT_EQ(st.level, DiagStatus::OK);
  EXPECT_EQ(st.message, "GNSS_AIDED");
  EXPECT_EQ(valueOf(st, "pos_stddev_m"), "0.100");
}

TEST(Diagnostics, ShortDeadReckoningIsOk) {
  const auto o = output(gll::LocalizationStatus::DEAD_RECKONING, 12.5, false);
  const DiagStatus st = makeLocalizationStatus(&o, gll::Diagnostics(), 30.0);
  EXPECT_EQ(st.level, DiagStatus::OK);
  EXPECT_EQ(st.message, "DEAD_RECKONING");
  EXPECT_EQ(valueOf(st, "dr_distance_m"), "12.50");
  EXPECT_EQ(valueOf(st, "dr_error_distance_m"), "30.0");
}

TEST(Diagnostics, LongDeadReckoningIsError) {
  // 位置の観測なしで上限を超えて走ったら ERROR とメッセージで知らせる（設計書 3.12 節）
  const auto o = output(gll::LocalizationStatus::DEAD_RECKONING, 31.24, true);
  const DiagStatus st = makeLocalizationStatus(&o, gll::Diagnostics(), 30.0);
  EXPECT_EQ(st.level, DiagStatus::ERROR);
  EXPECT_EQ(st.message, "DEAD_RECKONING: dead reckoning for 31.2 m without GNSS / LiDAR position (limit 30.0 m)");
}

TEST(Diagnostics, DistanceErrorOverridesWarn) {
  // 共分散が大きく DEGRADED（WARN）になっていても、距離の超過は ERROR にする
  const auto o = output(gll::LocalizationStatus::DEGRADED, 45.0, true);
  const DiagStatus st = makeLocalizationStatus(&o, gll::Diagnostics(), 30.0);
  EXPECT_EQ(st.level, DiagStatus::ERROR);
  EXPECT_EQ(st.message.rfind("DEGRADED: dead reckoning for 45.0 m", 0), 0u);
}

TEST(Diagnostics, LostIsError) {
  const auto o = output(gll::LocalizationStatus::LOST, 5.0, false);
  EXPECT_EQ(makeLocalizationStatus(&o, gll::Diagnostics(), 30.0).level, DiagStatus::ERROR);
}

TEST(Diagnostics, Counters) {
  gll::Diagnostics d;
  d.gnss_count = 10;
  d.gnss_accepted = 7;
  d.gnss_reject_reasons["NOT_RTK_FIX"] = 3;
  const DiagStatus st = makeCounterStatus(d);
  EXPECT_EQ(st.level, DiagStatus::OK);
  EXPECT_EQ(valueOf(st, "gnss"), "10");
  EXPECT_EQ(valueOf(st, "gnss_accepted"), "7");
  EXPECT_EQ(valueOf(st, "gnss_reject_NOT_RTK_FIX"), "3");
}

TEST(Diagnostics, MapStatusWarnsOnAnchorMismatch) {
  gll::Diagnostics d;
  d.map.active_group = "area_b";
  d.map.loaded_tiles = 12;
  gll::MismatchStats ok;
  ok.count = 100;
  ok.mean_world = gll::Vec3(0.02, -0.01, gll::deg2rad(0.1));
  gll::MismatchStats bad;
  bad.count = 100;
  bad.mean_world = gll::Vec3(0.15, 0.05, gll::deg2rad(0.2));
  d.anchor_mismatch["area_a"] = ok;
  gll::ArbiterConfig arb;
  DiagStatus st = makeMapStatus(d, arb);
  EXPECT_EQ(st.level, DiagStatus::OK);
  EXPECT_EQ(st.message, "active map group: area_b");
  EXPECT_EQ(valueOf(st, "loaded_tiles"), "12");

  d.anchor_mismatch["area_b"] = bad;
  st = makeMapStatus(d, arb);
  EXPECT_EQ(st.level, DiagStatus::WARN);
  EXPECT_NE(st.message.find("anchor of map group area_b needs calibration"), std::string::npos);
  EXPECT_EQ(st.message.find("area_a"), std::string::npos);

  // 件数が少ないうちは判定しない
  bad.count = 5;
  d.anchor_mismatch["area_b"] = bad;
  EXPECT_EQ(makeMapStatus(d, arb).level, DiagStatus::OK);
}

TEST(Diagnostics, CountersIncludeLidar) {
  gll::Diagnostics d;
  d.lidar_count = 100;
  d.lidar_accepted = 90;
  d.lidar_reject_reasons["LOW_OVERLAP"] = 4;
  const DiagStatus st = makeCounterStatus(d);
  EXPECT_EQ(valueOf(st, "lidar"), "100");
  EXPECT_EQ(valueOf(st, "lidar_accepted"), "90");
  EXPECT_EQ(valueOf(st, "lidar_reject_LOW_OVERLAP"), "4");
}
