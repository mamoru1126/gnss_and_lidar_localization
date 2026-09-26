#include "gll/measurement/attitude_estimator.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace gll;

namespace {

ImuSample staticSample(double t, double roll, double pitch, const Vec3& bias) {
  const Mat3 R = (Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX()))
                     .toRotationMatrix();
  ImuSample s;
  s.t = t;
  s.acc = R.transpose() * Vec3(0, 0, kGravity);
  s.gyro = bias;
  return s;
}

}  // namespace

TEST(AttitudeEstimator, StaticInitializationRecoversTiltAndBias) {
  AttitudeEstimator att;
  const double roll = deg2rad(3.0), pitch = deg2rad(-5.0);
  const Vec3 bias(0.002, -0.001, 0.003);
  bool done = false;
  for (int i = 0; i <= 400 && !done; ++i) done = att.addStaticSample(staticSample(i * 0.01, roll, pitch, bias));
  ASSERT_TRUE(att.initialized());
  EXPECT_NEAR(att.roll(), roll, 1e-6);
  EXPECT_NEAR(att.pitch(), pitch, 1e-6);
  EXPECT_LT((att.gyroBias() - bias).norm(), 1e-9);
}

TEST(AttitudeEstimator, ConvergesAfterTiltChange) {
  AttitudeEstimator att;
  const Vec3 bias = Vec3::Zero();
  for (int i = 0; i <= 300; ++i) att.addStaticSample(staticSample(i * 0.01, 0.0, 0.0, bias));
  // 車両が 4° の坂に乗った状態（傾いた後）で静止している
  const double pitch = deg2rad(4.0);
  for (int i = 301; i < 1500; ++i) att.update(staticSample(i * 0.01, 0.0, pitch, bias), 0.0, 0.0);
  EXPECT_NEAR(att.pitch(), pitch, deg2rad(0.1));
  EXPECT_NEAR(att.roll(), 0.0, deg2rad(0.1));
}

TEST(AttitudeEstimator, CentripetalCompensation) {
  // 水平面を 1.5 m/s・0.3 rad/s で旋回（向心加速度 0.45 m/s² が左向き）。補償すれば roll は 0 のまま
  AttitudeEstimator att;
  for (int i = 0; i <= 300; ++i) att.addStaticSample(staticSample(i * 0.01, 0.0, 0.0, Vec3::Zero()));
  const double v = 1.5, w = 0.3;
  for (int i = 301; i < 3000; ++i) {
    ImuSample s;
    s.t = i * 0.01;
    s.gyro = Vec3(0, 0, w);
    s.acc = Vec3(0.0, v * w, kGravity);
    att.update(s, v, 0.0);
  }
  EXPECT_NEAR(att.roll(), 0.0, deg2rad(0.05));
  EXPECT_NEAR(att.verticalRate(Vec3(0, 0, w)), w, 1e-6);
}

TEST(AttitudeEstimator, VerticalRateProjectsTiltedGyro) {
  AttitudeEstimator att;
  const double pitch = deg2rad(10.0);
  for (int i = 0; i <= 300; ++i) att.addStaticSample(staticSample(i * 0.01, 0.0, pitch, Vec3::Zero()));
  // 世界の鉛直軸まわりに 0.2 rad/s 回るときの機体座標系の角速度
  const Mat3 R = Eigen::AngleAxisd(pitch, Vec3::UnitY()).toRotationMatrix();
  const Vec3 w_body = R.transpose() * Vec3(0, 0, 0.2);
  EXPECT_NEAR(att.verticalRate(w_body), 0.2, 1e-9);
}
