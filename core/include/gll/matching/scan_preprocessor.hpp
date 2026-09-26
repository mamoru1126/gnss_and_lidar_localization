// スキャンの前処理（設計書 6.1 節）: デスキュー → base_link 座標系への変換 → クロップ。
// 間引きと点ごとの共分散は IScanMatcher::prepareSource で行う。
#pragma once

#include "gll/common/config.hpp"
#include "gll/common/types.hpp"

#include <vector>

namespace gll {

/// デスキューに使う運動（スキャンの間は一定とみなす）。どちらも base_link 座標系。
struct ScanMotion {
  Vec3 angular_velocity = Vec3::Zero();  ///< [rad/s]（バイアス補正済みのジャイロ）
  Vec3 velocity = Vec3::Zero();          ///< [m/s]
};

class ScanPreprocessor {
 public:
  explicit ScanPreprocessor(const LidarConfig& cfg = LidarConfig()) : cfg_(cfg) {}

  /// LiDAR 座標系の点を、時刻 scan.t の base_link 座標系に変換する。
  /// 点ごとの時刻があり、deskew が有効なら、スキャン中の回転と並進を補正する。
  std::vector<Vec3f> process(const LidarScan& scan, const ScanMotion& motion) const;

 private:
  LidarConfig cfg_;
};

}  // namespace gll
