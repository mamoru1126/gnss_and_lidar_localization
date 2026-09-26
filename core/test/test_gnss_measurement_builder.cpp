#include "gll/measurement/gnss_measurement_builder.hpp"

#include <gtest/gtest.h>

using namespace gll;

namespace {

GnssSample sample(double t, int status, double sigma) {
  GnssSample s;
  s.t = t;
  s.lat = 35.68;
  s.lon = 139.77;
  s.h = 40.0;
  s.raw_status = status;
  s.cov_known = true;
  s.cov_enu = Mat3::Identity() * sigma * sigma;
  return s;
}

AttitudeEstimator levelAttitude() {
  AttitudeEstimator att;
  for (int i = 0; i <= 300; ++i) {
    ImuSample s;
    s.t = i * 0.01;
    s.acc = Vec3(0, 0, kGravity);
    att.addStaticSample(s);
  }
  return att;
}

}  // namespace

TEST(GnssBuilder, RejectsNonRtkFix) {
  GnssMeasurementBuilder b;
  const AttitudeEstimator att = levelAttitude();
  EXPECT_EQ(b.buildPosition(sample(10.0, 0, 0.01), att).reason, GnssRejectReason::NOT_RTK_FIX);
  EXPECT_EQ(b.buildPosition(sample(10.1, -1, 0.01), att).reason, GnssRejectReason::NOT_RTK_FIX);
}

TEST(GnssBuilder, SettleTimeAndStddev) {
  GnssMeasurementBuilder b;
  const AttitudeEstimator att = levelAttitude();
  EXPECT_EQ(b.buildPosition(sample(10.0, 2, 0.01), att).reason, GnssRejectReason::SETTLING);
  EXPECT_EQ(b.buildPosition(sample(10.5, 2, 0.01), att).reason, GnssRejectReason::SETTLING);
  // σ が大きい（FLOAT が status 2 で届いた場合を想定）
  EXPECT_EQ(b.buildPosition(sample(11.1, 2, 0.20), att).reason, GnssRejectReason::STDDEV_TOO_LARGE);
  const auto ok = b.buildPosition(sample(11.2, 2, 0.01), att);
  ASSERT_TRUE(ok.position.has_value());
  // σ の下限（0.02 m）が効いている
  EXPECT_NEAR(ok.position->cov_world(0, 0), 0.02 * 0.02, 1e-12);
  // FIX が途切れたら安定待ちからやり直す
  b.buildPosition(sample(11.3, 0, 0.5), att);
  EXPECT_EQ(b.buildPosition(sample(11.4, 2, 0.01), att).reason, GnssRejectReason::SETTLING);
}

TEST(GnssBuilder, MessageGapRestartsSettling) {
  GnssMeasurementBuilder b;
  const AttitudeEstimator att = levelAttitude();
  b.buildPosition(sample(10.0, 2, 0.01), att);
  ASSERT_TRUE(b.buildPosition(sample(11.1, 2, 0.01), att).position.has_value());
  // 5 秒間メッセージが来なかった後の最初の FIX は、安定待ちからやり直す
  EXPECT_EQ(b.buildPosition(sample(16.1, 2, 0.01), att).reason, GnssRejectReason::SETTLING);
  EXPECT_TRUE(b.buildPosition(sample(17.2, 2, 0.01), att).position.has_value());
}

TEST(GnssBuilder, UnknownCovariance) {
  GnssConfig c;
  GnssMeasurementBuilder strict(c);
  const AttitudeEstimator att = levelAttitude();
  GnssSample s = sample(10.0, 2, 0.01);
  s.cov_known = false;
  EXPECT_EQ(strict.buildPosition(s, att).reason, GnssRejectReason::COVARIANCE_UNKNOWN);
  c.accept_unknown_covariance = true;
  c.fix_settle_time = 0.0;
  GnssMeasurementBuilder lenient(c);
  EXPECT_TRUE(lenient.buildPosition(s, att).position.has_value());
}

TEST(GnssBuilder, LeverArmTiltProjection) {
  // 設計書 3.6 節: l_z = 1 m のとき、5° の傾きでアンテナは約 8.7 cm 水平にずれる
  GnssConfig c;
  c.lever_arm = Vec3(0.0, 0.0, 1.0);
  const GnssMeasurementBuilder b(c);
  const Vec2 lp = b.projectLeverArm(0.0, deg2rad(5.0));
  EXPECT_NEAR(lp.x(), std::sin(deg2rad(5.0)), 1e-12);
  EXPECT_NEAR(lp.x(), 0.0872, 1e-3);
  const Vec2 lr = b.projectLeverArm(deg2rad(1.0), 0.0);
  EXPECT_NEAR(lr.y(), -std::sin(deg2rad(1.0)), 1e-12);  // 右に傾くとアンテナは右（-y）へ
}

TEST(GnssBuilder, LeverCovarianceFromAttitudeUncertainty) {
  GnssConfig c;
  c.lever_arm = Vec3(0.3, 0.2, 1.0);
  c.fix_settle_time = 0.0;
  GnssMeasurementBuilder b(c);
  const AttitudeEstimator att = levelAttitude();
  const auto r = b.buildPosition(sample(10.0, 2, 0.01), att);
  ASSERT_TRUE(r.position.has_value());
  const double expected = std::pow(1.0 * deg2rad(0.5), 2);  // 約 0.9 cm の 2 乗
  EXPECT_NEAR(r.position->lever_cov_body(0, 0), expected, 1e-12);
  EXPECT_NEAR(std::sqrt(expected), 0.0087, 1e-4);
}
