#include "gll/estimation/output_smoother.hpp"

#include <gtest/gtest.h>

using namespace gll;

TEST(OutputSmoother, CorrectionDoesNotMoveOutput) {
  OutputSmoother sm;
  Pose2D raw{10.0, 20.0, 0.5};
  const Mat3 cov = Mat3::Identity() * 1e-4;
  const SmoothedOutput before = sm.apply(raw, cov, 0.0);
  // 推定値が 0.3 m 横に動く
  sm.onCorrection(Vec3(0.0, 0.3, 0.0));
  raw.y += 0.3;
  const SmoothedOutput after = sm.apply(raw, cov, 0.0);
  EXPECT_NEAR(after.pose.x, before.pose.x, 1e-12);
  EXPECT_NEAR(after.pose.y, before.pose.y, 1e-12);
}

TEST(OutputSmoother, RateLimitedAbsorption) {
  OutputConfig c;
  c.max_rate_xy = 0.1;
  OutputSmoother sm(c);
  const Pose2D raw{0.0, 0.3, 0.0};
  sm.onCorrection(Vec3(0.0, 0.3, 0.0));
  const Mat3 cov = Mat3::Identity() * 9e-4;
  double y_prev = sm.apply(raw, cov, 0.0).pose.y;
  double max_step = 0.0;
  double t = 0.0;
  while (sm.offset().head<2>().norm() > 1e-12 && t < 10.0) {
    const double y = sm.apply(raw, cov, 0.02).pose.y;
    max_step = std::max(max_step, std::abs(y - y_prev));
    y_prev = y;
    t += 0.02;
  }
  EXPECT_LE(max_step, 0.1 * 0.02 + 1e-12);
  EXPECT_NEAR(t, 3.0, 0.03);  // 0.3 m を 0.1 m/s で吸収すると 3 秒（設計書 3.10 節）
}

TEST(OutputSmoother, CovarianceIncludesOffset) {
  OutputSmoother sm;
  sm.onCorrection(Vec3(0.0, 0.3, 0.0));
  const Mat3 cov = Mat3::Identity() * 0.03 * 0.03;
  const SmoothedOutput o = sm.apply(Pose2D{0.0, 0.3, 0.0}, cov, 0.0);
  // Σ_out = Σ_w + o oᵀ（横方向の σ はおよそ 0.3 m）
  EXPECT_NEAR(std::sqrt(o.cov(1, 1)), std::sqrt(0.03 * 0.03 + 0.09), 1e-12);
  EXPECT_NEAR(o.cov(0, 0), cov(0, 0), 1e-12);
}

TEST(OutputSmoother, OffsetExceededFlag) {
  OutputSmoother sm;
  sm.onCorrection(Vec3(1.5, 0.0, 0.0));
  EXPECT_TRUE(sm.offsetExceeded());
  sm.reset();
  EXPECT_FALSE(sm.offsetExceeded());
}
