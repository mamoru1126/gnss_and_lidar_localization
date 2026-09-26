// 停止検出とゼロ角速度観測（ZARU）。設計書 3.4 節。
#pragma once

#include "gll/common/config.hpp"
#include "gll/estimation/state.hpp"

#include <optional>

namespace gll {

class StopDetector {
 public:
  explicit StopDetector(const StopConfig& cfg = StopConfig()) : cfg_(cfg) {}

  /// 予測の入力ごとに呼ぶ。停止が min_duration 続くたびに ZARU の観測を返す。
  std::optional<ZeroRateMeasurement> update(const MotionInput& u);
  bool stopped() const { return stop_since_ >= 0.0; }

 private:
  StopConfig cfg_;
  double stop_since_ = -1.0;
  double window_start_ = -1.0;
  double sum_ = 0.0;
  int n_ = 0;
};

}  // namespace gll
