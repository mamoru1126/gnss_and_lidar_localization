// 比較用の ESEKF（SO(2) × R²: p = p̂ + δp, θ = θ̂ + δθ）。アルゴリズム説明書 5 章。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state_estimator.hpp"

namespace gll {

class EsEkf2D : public IStateEstimator {
 public:
  explicit EsEkf2D(const EstimatorConfig& cfg = EstimatorConfig()) : cfg_(cfg) {}

  FilterState predict(const FilterState& st, const MotionInput& u, double dt) const override;
  Linearization linearize(const FilterState& st, const Measurement& m) const override;
  void inject(FilterState& st, const Vec5& dx) const override;
  Mat3 worldCovariance(const FilterState& st) const override { return st.P.topLeftCorner<3, 3>(); }
  Mat3 errorCovarianceFromWorld(const FilterState&, const Mat3& cov_world) const override {
    return cov_world;
  }
  std::string name() const override { return "ESEKF"; }

 private:
  EstimatorConfig cfg_;
};

}  // namespace gll
