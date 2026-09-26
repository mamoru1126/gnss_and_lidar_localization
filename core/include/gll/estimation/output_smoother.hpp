// 出力整形（補正オフセット吸収方式）。設計書 3.10 節。
#pragma once

#include "gll/common/config.hpp"
#include "gll/common/types.hpp"

namespace gll {

struct SmoothedOutput {
  Pose2D pose;
  Mat3 cov = Mat3::Identity();  ///< Σ_w + o oᵀ
};

class OutputSmoother {
 public:
  explicit OutputSmoother(const OutputConfig& cfg = OutputConfig()) : cfg_(cfg) {}

  void reset() { offset_.setZero(); }

  /// 観測更新で推定値が世界座標系で world_delta だけ動いたときに呼ぶ（出力は動かさない）。
  void onCorrection(const Vec3& world_delta);

  /// オフセットを dt の間にレート制限つきで 0 に近づけ、出力を返す。
  SmoothedOutput apply(const Pose2D& raw, const Mat3& raw_cov, double dt);

  const Vec3& offset() const { return offset_; }
  bool offsetExceeded() const;

 private:
  OutputConfig cfg_;
  Vec3 offset_ = Vec3::Zero();
};

}  // namespace gll
