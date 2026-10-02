// small_gicp による位置合わせ（設計書 6.2 節）。追跡は GICP（KdTree）か VGICP（lidar.registration）、粗い探索は VGICP を使う。
// どれも CPU（OpenMP）で動く。
#pragma once

#include "gll/matching/scan_matcher.hpp"

#include <utility>

namespace gll {

class GicpMatcher : public IScanMatcher {
 public:
  explicit GicpMatcher(const LidarConfig& lidar = LidarConfig(), const RelocalizeConfig& reloc = RelocalizeConfig());

  std::shared_ptr<const MatchTarget> buildTarget(const std::string& group, const MapAnchor& anchor,
                                                 const std::vector<std::shared_ptr<const TileData>>& tiles) const override;
  std::shared_ptr<const SourceCloud> prepareSource(const std::vector<Vec3f>& points_base,
                                                   double voxel_size) const override;
  RegistrationResult align(const SourceCloud& source, const MatchTarget& target,
                           const Eigen::Isometry3d& init) const override;
  PoseSearchResult search(const std::vector<Vec3f>& points_base, const MatchTarget& target,
                          const PoseSearchRequest& request) const override;

  /// 粗い位置合わせ（VGICP。ターゲットのボクセル地図は初回に作る）。num_threads <= 0 なら設定の値を使う。
  RegistrationResult alignCoarse(const SourceCloud& source, const MatchTarget& target,
                                 const Eigen::Isometry3d& init, int num_threads = 0) const;

  /// スキャンの点のうち、ターゲットの点から distance 以内にあるものの割合（structure_only なら水平でない面（壁・柱など）
  /// の点で数える。そうした点が少なければ全点）。num_threads <= 0 なら設定の値を使う。
  double overlap(const SourceCloud& source, const MatchTarget& target, const Eigen::Isometry3d& T,
                 double distance, int num_threads = 0, bool structure_only = true) const;

  /// overlap と同じ点を数え、分母を「near_distance 以内に地図の点がある点」に絞った割合も返す（{全体, 近い点だけ}）。
  /// 近い点が 1 つもなければ、近い点だけの割合は 0。
  std::pair<double, double> overlapWithNear(const SourceCloud& source, const MatchTarget& target,
                                            const Eigen::Isometry3d& T, double distance, double near_distance,
                                            int num_threads = 0, bool structure_only = true) const;

 private:
  LidarConfig cfg_;
  RelocalizeConfig reloc_;
};

}  // namespace gll
