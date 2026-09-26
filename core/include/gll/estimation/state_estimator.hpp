// 推定器のインターフェース（状態を持たない純粋関数の集まり。ソフトウェア構成 3.3 節）。
#pragma once

#include "gll/estimation/mahalanobis_gate.hpp"
#include "gll/estimation/state.hpp"

#include <string>

namespace gll {

class IStateEstimator {
 public:
  virtual ~IStateEstimator() = default;

  /// 状態を入力 u で dt だけ伝播する（返り値の t は st.t + dt）。
  virtual FilterState predict(const FilterState& st, const MotionInput& u, double dt) const = 0;

  /// 観測を線形化する（残差・観測行列・観測共分散）。
  virtual Linearization linearize(const FilterState& st, const Measurement& m) const = 0;

  /// 誤差の推定値 dx を名目状態に注入する。
  virtual void inject(FilterState& st, const Vec5& dx) const = 0;

  /// 世界座標系の (x, y, yaw) の共分散。
  virtual Mat3 worldCovariance(const FilterState& st) const = 0;

  /// 世界座標系の (x, y, yaw) の共分散を、誤差の座標系の共分散に変換する（初期化用）。
  virtual Mat3 errorCovarianceFromWorld(const FilterState& st, const Mat3& cov_world) const = 0;

  virtual std::string name() const = 0;
};

/// 共通の観測更新（Joseph 形式）。設計書 3.5 節の手順 3〜7。
/// policy が REJECT_ON_FAIL / DEFER_TO_RECOVERY で判定に落ちた場合は状態を変更しない。
UpdateResult correct(const IStateEstimator& est, FilterState& st, const Linearization& lin,
                     GatePolicy policy, const MahalanobisGate& gate);

}  // namespace gll
