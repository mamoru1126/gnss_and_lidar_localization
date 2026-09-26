// Invariant EKF（SE(2) 上の左不変誤差 X = X̂ Exp(ξ)）。設計書 3.3〜3.7 節。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state_estimator.hpp"

namespace gll {

class InvEkfSe2 : public IStateEstimator {
 public:
  explicit InvEkfSe2(const EstimatorConfig& cfg = EstimatorConfig()) : cfg_(cfg) {}

  FilterState predict(const FilterState& st, const MotionInput& u, double dt) const override;
  Linearization linearize(const FilterState& st, const Measurement& m) const override;
  void inject(FilterState& st, const Vec5& dx) const override;
  Mat3 worldCovariance(const FilterState& st) const override;
  Mat3 errorCovarianceFromWorld(const FilterState& st, const Mat3& cov_world) const override;
  std::string name() const override { return "InvariantEKF"; }

  /// 予測の遷移行列 F と入力ノイズ行列 G（テスト用に公開）。
  void transitionMatrices(const FilterState& st, const MotionInput& u, double dt, Mat5& F,
                          Eigen::Matrix<double, 5, 3>& G) const;

 private:
  EstimatorConfig cfg_;
};

}  // namespace gll
