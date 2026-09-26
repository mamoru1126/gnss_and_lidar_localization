// スキャンマッチングのインターフェース（設計書 6 章、ソフトウェア構成 3.5 節）。
// small_gicp の型は実装（GicpMatcher）の中に閉じ込め、ほかのクラスはこのインターフェースだけを使う。
#pragma once

#include "gll/common/config.hpp"
#include "gll/map/map_anchor.hpp"
#include "gll/map/tile.hpp"

#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gll {

/// 位置合わせのターゲット（アクティブグループのロード済みタイルを結合したもの）。中身は実装ごとに持つ。
class MatchTarget {
 public:
  virtual ~MatchTarget() = default;

  std::string group;          ///< 地図グループ ID
  MapAnchor anchor;           ///< このグループのアンカー（照合の結果はこれで UTM に変換する。設計書 5.5 節）
  std::vector<TileId> tiles;  ///< 含まれるタイル
  std::size_t num_points = 0;

  /// 地図座標 (x, y) から水平距離 radius 以内の点の z を小さい順に並べ、下から fraction の位置の値を返す
  /// （地面の高さの推定。地図上での初期化で z を決めるのに使う）。点が無ければ nullopt。
  virtual std::optional<double> groundHeight(double x, double y, double radius, double fraction = 0.1) const = 0;

  /// 可視化用に、ターゲットの点を最大 max_points 点まで間引いて返す（地図座標系）。
  virtual std::vector<Vec3f> samplePoints(std::size_t max_points) const = 0;
};

/// 前処理した（間引いて点ごとの共分散を付けた）スキャン。base_link 座標系。
class SourceCloud {
 public:
  virtual ~SourceCloud() = default;
  virtual std::size_t size() const = 0;
};

/// 位置合わせの結果と品質指標。
struct RegistrationResult {
  bool converged = false;
  int iterations = 0;
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();  ///< 地図座標系の base_link の姿勢
  Mat6 H = Mat6::Zero();       ///< 情報行列（機体座標系側の摂動 T ← T Exp(δ)。δ の並びは回転 3 → 並進 3）
  double error = 0.0;          ///< GICP の誤差（0.5 Σ rᵀ W r）
  std::size_t num_source = 0;  ///< 間引いた後のスキャンの点数
  std::size_t num_inliers = 0;
  double inlier_ratio = 0.0;     ///< num_inliers / num_source
  double error_per_point = 0.0;  ///< error / num_inliers
  double overlap = 0.0;          ///< overlap_distance 以内に地図の点がある、スキャンの点の割合
};

/// 複数の初期値から位置合わせを試す（地図上での初期化と再位置推定。設計書 3.11 節・3.13.4 節）。
struct PoseSearchRequest {
  Eigen::Isometry3d center = Eigen::Isometry3d::Identity();  ///< 探索の中心（地図座標系。z・roll・pitch も使う）
  double radius = 1.0;     ///< 位置の探索半径 [m]
  double yaw_range = 0.5;  ///< yaw の探索幅（±）[rad]
};

struct PoseSearchResult {
  bool found = false;            ///< 品質と一意性の条件を満たす解があったか
  RegistrationResult best;       ///< overlap が最大の解（found = false でも入る）
  double best_overlap = 0.0;
  double second_overlap = 0.0;   ///< 最良の解から十分離れた解のうち、overlap が最大のもの（無ければ 0）
  int num_hypotheses = 0;        ///< 試した初期値の数
  std::string reason;            ///< found = false の理由
};

class IScanMatcher {
 public:
  virtual ~IScanMatcher() = default;

  /// ロード済みのタイルを結合してターゲットを作る（地図ロードワーカーから呼ぶ）。
  virtual std::shared_ptr<const MatchTarget> buildTarget(
      const std::string& group, const MapAnchor& anchor,
      const std::vector<std::shared_ptr<const TileData>>& tiles) const = 0;

  /// base_link 座標系の点を間引き、点ごとの共分散を付ける。
  virtual std::shared_ptr<const SourceCloud> prepareSource(const std::vector<Vec3f>& points_base,
                                                           double voxel_size) const = 0;

  /// 初期値 init（地図座標系の base_link の姿勢）から位置合わせする。
  virtual RegistrationResult align(const SourceCloud& source, const MatchTarget& target,
                                   const Eigen::Isometry3d& init) const = 0;

  /// 初期値の周りに候補を並べ、粗い位置合わせ → 上位の候補を詰める → 一意性を確かめる。
  virtual PoseSearchResult search(const std::vector<Vec3f>& points_base, const MatchTarget& target,
                                  const PoseSearchRequest& request) const = 0;
};

}  // namespace gll
