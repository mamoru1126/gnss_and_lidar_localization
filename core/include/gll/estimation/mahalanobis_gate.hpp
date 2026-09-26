// Mahalanobis 距離による外れ値判定（設計書 3.5 節）。
#pragma once

#include <array>

namespace gll {

/// 自由度 k のカイ二乗分布の累積分布関数。
double chi2Cdf(double x, int k);

/// 自由度 k のカイ二乗分布の分位点（CDF = p となる x）。
double chi2Quantile(double p, int k);

class MahalanobisGate {
 public:
  explicit MahalanobisGate(double alpha = 0.001);

  /// d2 が自由度 dof の閾値以下なら true。
  bool pass(double d2, int dof) const;
  double threshold(int dof) const;
  double alpha() const { return alpha_; }

 private:
  static constexpr int kMaxDof = 6;
  double alpha_;
  std::array<double, kMaxDof + 1> thresholds_{};
};

}  // namespace gll
