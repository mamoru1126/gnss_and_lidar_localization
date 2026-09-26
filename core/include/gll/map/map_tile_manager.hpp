// 地図タイルの動的ロードと、位置合わせのターゲットの管理（設計書 5.3〜5.5 節）。
//
// - 必要なタイル: 現在位置と先読みした位置のどちらかから load_radius 以内（全グループ）。次に入るグループのタイルも
//   前もってキャッシュに入る。現在位置から unload_radius より遠くなったタイルは捨てる。
// - アクティブグループ: 現在位置（先読みを含まない）から最も近いグループ（load_radius 以内）。今のグループより
//   group_switch_margin 以上近いグループが現れたときだけ切り替える。
// - ターゲット: アクティブグループのロード済みタイルだけで作り、できたら差し替える（ダブルバッファ）。
//   照合中のスレッドは shared_ptr でターゲットを持つので、差し替えの影響を受けない。
#pragma once

#include "gll/common/config.hpp"
#include "gll/common/logger.hpp"
#include "gll/map/map_config.hpp"
#include "gll/matching/scan_matcher.hpp"

#include <array>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace gll {

class MapTileManager {
 public:
  struct Stats {
    std::string active_group;
    std::size_t loaded_tiles = 0;
    std::map<std::string, std::size_t> loaded_tiles_per_group;
    std::size_t target_tiles = 0;
    std::size_t target_points = 0;
    std::size_t tile_loads = 0;
    std::size_t tile_load_failures = 0;
    std::size_t target_builds = 0;
  };

  MapTileManager(const MapManagerConfig& cfg, std::vector<MapGroup> groups, std::shared_ptr<const ITileLoader> loader,
                 std::shared_ptr<const IScanMatcher> matcher, std::shared_ptr<ILogger> logger = nullptr);
  ~MapTileManager();
  MapTileManager(const MapTileManager&) = delete;
  MapTileManager& operator=(const MapTileManager&) = delete;

  /// 現在位置（UTM）・進行方向・速さを与える。update_distance 以上動いたか update_interval 以上たっていれば、
  /// 必要なタイルとアクティブグループを見直す。非同期モードではすぐ戻り、同期モードではその場でロードする。
  /// force = true なら間隔によらず見直す。
  void update(double t, const Vec2& position, double yaw, double speed, bool force = false);

  /// 今のターゲット（アクティブグループが無い、またはまだ準備中なら nullptr）。
  std::shared_ptr<const MatchTarget> currentTarget() const;
  /// アクティブグループ（無ければ空文字列）。
  std::string activeGroup() const;
  /// 位置 p（UTM）から、いずれかのグループのタイルまでの距離が radius 以内か。
  bool hasMapWithin(const Vec2& p, double radius) const;
  /// 位置 p（UTM）から最も近いグループとその距離（グループが無ければ空文字列）。
  std::pair<std::string, double> nearestGroup(const Vec2& p) const;
  const std::vector<MapGroup>& groups() const { return groups_; }
  const MapGroup* group(const std::string& id) const;
  /// タイルの範囲（UTM）が重なっているグループの組（運用上の前提では重ならない。設計書 5.1 節）。
  std::vector<std::pair<std::string, std::string>> overlappingGroups() const;
  /// ロードとターゲットの構築が終わるまで待つ（テスト用）。
  void waitIdle();
  Stats stats() const;

 private:
  struct TileEntry {
    std::size_t group_index;
    TileMeta meta;
    std::array<Vec2, 4> corners;  ///< タイルの四隅（UTM）
    Vec2 bb_min, bb_max;          ///< 四隅を囲む矩形（UTM）
  };

  double distanceToTile(const TileEntry& e, const Vec2& p) const;
  void workerLoop();
  void processPending();

  MapManagerConfig cfg_;
  std::vector<MapGroup> groups_;
  std::shared_ptr<const ITileLoader> loader_;
  std::shared_ptr<const IScanMatcher> matcher_;
  std::shared_ptr<ILogger> logger_;
  std::vector<TileEntry> tiles_;                       ///< 全グループのタイル（起動時に作り、以後は読むだけ）
  std::vector<std::pair<Vec2, Vec2>> group_bounds_;    ///< グループごとのタイルの範囲（UTM）

  // update() の状態（呼び出し側のスレッド）
  std::mutex update_mtx_;
  bool have_last_ = false;
  Vec2 last_position_ = Vec2::Zero();
  double last_t_ = -1e18;

  // 要求と結果（mtx_ で保護）
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::condition_variable idle_cv_;
  bool pending_ = false;
  bool busy_ = false;
  bool stop_ = false;
  std::set<std::size_t> requested_tiles_;  ///< tiles_ の添字
  int requested_active_ = -1;              ///< groups_ の添字（-1 は無し）
  Vec2 requested_position_ = Vec2::Zero();
  int active_ = -1;                        ///< 見直しで決めたアクティブグループ（ヒステリシス用）
  std::shared_ptr<const MatchTarget> front_;
  Stats stats_;

  // ロードワーカーだけが触る
  std::map<std::size_t, std::shared_ptr<const TileData>> cache_;
  std::set<std::size_t> target_tiles_;
  int target_group_ = -1;

  std::thread worker_;
};

}  // namespace gll
