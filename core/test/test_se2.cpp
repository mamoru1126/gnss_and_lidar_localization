#include "gll/common/se2.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace gll;

TEST(SE2, ExpLogRoundTrip) {
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  for (int i = 0; i < 200; ++i) {
    const Vec3 xi(u(rng), u(rng), u(rng));
    const Vec3 back = SE2::Exp(xi).Log();
    EXPECT_LT((back - xi).norm(), 1e-9) << xi.transpose();
  }
}

TEST(SE2, SmallAngleIsContinuous) {
  const Vec3 a(0.3, -0.2, 1e-9), b(0.3, -0.2, 1e-5);
  EXPECT_LT((SE2::Exp(a).t() - SE2::Exp(b).t()).norm(), 1e-5);
  EXPECT_LT((SE2::Exp(Vec3(0.3, -0.2, 0.0)).Log() - Vec3(0.3, -0.2, 0.0)).norm(), 1e-12);
}

TEST(SE2, AdjointIdentity) {
  // X Exp(ξ) X⁻¹ = Exp(Ad_X ξ)
  const SE2 X = SE2::fromPose(3.0, -2.0, 0.7);
  const Vec3 xi(0.1, -0.05, 0.02);
  const SE2 lhs = X * SE2::Exp(xi) * X.inverse();
  const SE2 rhs = SE2::Exp(X.Ad() * xi);
  EXPECT_LT((lhs.t() - rhs.t()).norm(), 1e-12);
  EXPECT_NEAR(lhs.yaw(), rhs.yaw(), 1e-12);
}

TEST(SE2, InverseAndCompose) {
  const SE2 A = SE2::fromPose(1.0, 2.0, 0.3), B = SE2::fromPose(-0.5, 0.4, -1.2);
  const SE2 I = A * A.inverse();
  EXPECT_LT(I.t().norm(), 1e-12);
  EXPECT_NEAR(I.yaw(), 0.0, 1e-12);
  const Vec2 q(0.7, -0.1);
  EXPECT_LT(((A * B).act(q) - A.act(B.act(q))).norm(), 1e-12);
}

TEST(Angle, Wrap) {
  EXPECT_NEAR(wrapAngle(3.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrapAngle(-3.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrapAngle(0.1), 0.1, 1e-15);
}
