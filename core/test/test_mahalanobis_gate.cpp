#include "gll/estimation/mahalanobis_gate.hpp"

#include <gtest/gtest.h>

using namespace gll;

TEST(Chi2, KnownQuantiles) {
  // 設計書 3.5 節の値（α = 0.001）
  EXPECT_NEAR(chi2Quantile(0.999, 1), 10.828, 1e-3);
  EXPECT_NEAR(chi2Quantile(0.999, 2), 13.816, 1e-3);
  EXPECT_NEAR(chi2Quantile(0.999, 3), 16.266, 1e-3);
  EXPECT_NEAR(chi2Quantile(0.95, 2), 5.991, 1e-3);
}

TEST(Chi2, CdfMonotone) {
  double prev = 0.0;
  for (double x = 0.1; x < 40.0; x += 0.5) {
    const double c = chi2Cdf(x, 3);
    EXPECT_GE(c, prev);
    prev = c;
  }
  EXPECT_NEAR(chi2Cdf(1e3, 3), 1.0, 1e-12);
}

TEST(Gate, PassAndFail) {
  const MahalanobisGate g(0.001);
  EXPECT_TRUE(g.pass(13.0, 2));
  EXPECT_FALSE(g.pass(14.0, 2));
}
