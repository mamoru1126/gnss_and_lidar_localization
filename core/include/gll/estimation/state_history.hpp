// 状態履歴と遅延観測の再適用（設計書 3.8 節、アルゴリズム説明書 11 章）。
#pragma once

#include "gll/estimation/state_estimator.hpp"

#include <deque>
#include <optional>

namespace gll {

class StateHistory {
 public:
  struct Entry {
    FilterState state;  ///< 入力 input を適用した後の状態（state.t == input.t）
    MotionInput input;  ///< 直前のエントリからこのエントリまでの区間の入力
  };

  explicit StateHistory(double length = 2.0) : length_(length) {}

  bool empty() const { return buf_.empty(); }
  void reset(const FilterState& st);
  void push(const FilterState& st, const MotionInput& u);
  const FilterState& latest() const { return buf_.back().state; }
  const MotionInput& latestInput() const { return buf_.back().input; }
  double oldestTime() const { return buf_.front().state.t; }

  /// 時刻 t の状態（履歴の範囲外なら nullopt）。t が最新より新しい場合は最新の入力で外挿する。
  std::optional<FilterState> stateAt(double t, const IStateEstimator& est) const;

  /// 観測を時刻 t_z の状態に適用し、それ以降を保存した入力で再伝播する。
  /// inflation を与えると、更新の前に姿勢の誤差共分散（左上 3×3）に加える（再アンカー）。
  UpdateResult applyDelayed(const Measurement& m, const IStateEstimator& est,
                            const MahalanobisGate& gate, GatePolicy policy,
                            const std::optional<Mat3>& inflation = std::nullopt);

  /// 時刻 t1 から t2 までの推定上の相対運動 T(t1)⁻¹ T(t2)。
  std::optional<SE2> relativeMotion(double t1, double t2, const IStateEstimator& est) const;

  std::size_t size() const { return buf_.size(); }

 private:
  void prune();
  /// state.t <= t となる最後のエントリの添字（なければ -1）。
  int indexAtOrBefore(double t) const;

  double length_;
  std::deque<Entry> buf_;
};

}  // namespace gll
