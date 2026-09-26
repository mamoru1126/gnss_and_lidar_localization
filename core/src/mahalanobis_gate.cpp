#include "gll/estimation/mahalanobis_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace gll {
namespace {

// 正則化された下側不完全ガンマ関数 P(a, x)（Numerical Recipes の gser / gcf）。
double gammaP(double a, double x) {
  if (x <= 0.0) return 0.0;
  const double gln = std::lgamma(a);
  if (x < a + 1.0) {
    double ap = a, sum = 1.0 / a, del = sum;
    for (int n = 0; n < 1000; ++n) {
      ap += 1.0;
      del *= x / ap;
      sum += del;
      if (std::abs(del) < std::abs(sum) * 1e-15) break;
    }
    return sum * std::exp(-x + a * std::log(x) - gln);
  }
  // 連分数展開で Q(a, x) を求める
  const double fpmin = 1e-300;
  double b = x + 1.0 - a, c = 1.0 / fpmin, d = 1.0 / b, h = d;
  for (int i = 1; i < 1000; ++i) {
    const double an = -i * (i - a);
    b += 2.0;
    d = an * d + b;
    if (std::abs(d) < fpmin) d = fpmin;
    c = b + an / c;
    if (std::abs(c) < fpmin) c = fpmin;
    d = 1.0 / d;
    const double del = d * c;
    h *= del;
    if (std::abs(del - 1.0) < 1e-15) break;
  }
  return 1.0 - std::exp(-x + a * std::log(x) - gln) * h;
}

}  // namespace

double chi2Cdf(double x, int k) { return gammaP(0.5 * k, 0.5 * x); }

double chi2Quantile(double p, int k) {
  if (p <= 0.0 || p >= 1.0 || k <= 0) throw std::invalid_argument("chi2Quantile: invalid argument");
  double lo = 0.0, hi = 1.0;
  while (chi2Cdf(hi, k) < p) hi *= 2.0;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    (chi2Cdf(mid, k) < p ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

MahalanobisGate::MahalanobisGate(double alpha) : alpha_(alpha) {
  thresholds_[0] = 0.0;
  for (int k = 1; k <= kMaxDof; ++k) thresholds_[k] = chi2Quantile(1.0 - alpha, k);
}

double MahalanobisGate::threshold(int dof) const {
  if (dof < 1 || dof > kMaxDof) throw std::out_of_range("MahalanobisGate: unsupported dof");
  return thresholds_[dof];
}

bool MahalanobisGate::pass(double d2, int dof) const { return d2 <= threshold(dof); }

}  // namespace gll
