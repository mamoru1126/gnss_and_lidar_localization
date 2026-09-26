// 地図タイルマネージャ（ロード・アンロード・アクティブグループ・重なりの検出）の単体テスト。
#include "gll/map/map_tile_manager.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>

using namespace gll;

namespace {

/// タイルの中身は使わないので、決まった数の点を返すだけのローダー。
class FakeLoader : public ITileLoader {
 public:
  std::shared_ptr<const TileData> load(const TileMeta& meta) const override {
    ++loads;
    auto t = std::make_shared<TileData>();
    t->id = meta.id;
    t->points.assign(100, Vec3f::Zero());
    return t;
  }
  mutable std::atomic<int> loads{0};
};

class FakeTarget : public MatchTarget {
 public:
  std::optional<double> groundHeight(double, double, double, double) const override { return 0.0; }
};

/// ターゲットにタイル ID だけを記録するマッチャ。
class FakeMatcher : public IScanMatcher {
 public:
  std::shared_ptr<const MatchTarget> buildTarget(const std::string& group, const MapAnchor& anchor,
                                                 const std::vector<std::shared_ptr<const TileData>>& tiles) const override {
    auto t = std::make_shared<FakeTarget>();
    t->group = group;
    t->anchor = anchor;
    for (const auto& d : tiles) {
      t->tiles.push_back(d->id);
      t->num_points += d->points.size();
    }
    return t;
  }
  std::shared_ptr<const SourceCloud> prepareSource(const std::vector<Vec3f>&, double) const override { return nullptr; }
  RegistrationResult align(const SourceCloud&, const MatchTarget&, const Eigen::Isometry3d&) const override { return {}; }
  PoseSearchResult search(const std::vector<Vec3f>&, const MatchTarget&, const PoseSearchRequest&) const override {
    return {};
  }
};

/// 地図座標で [-40, 40)² を覆う 20 m タイル 16 枚のグループ。アンカーは UTM で直接与える。
MapGroup makeGroup(const std::string& id, double easting, double northing, double grid_heading_deg) {
  MapGroup g;
  g.id = id;
  AnchorConfig a;
  a.use_utm = true;
  a.easting = easting;
  a.northing = northing;
  a.grid_heading = deg2rad(grid_heading_deg);
  a.use_scale_factor = false;
  g.anchor = MapAnchor::fromConfig(a, UtmProjector(54, true));
  g.index.tile_size = 20.0;
  for (int ix = -2; ix < 2; ++ix)
    for (int iy = -2; iy < 2; ++iy) {
      TileMeta m;
      m.id = TileId{id, ix, iy};
      m.num_points = 100;
      g.index.tiles.push_back(m);
    }
  return g;
}

MapManagerConfig managerConfig(bool async) {
  MapManagerConfig c;
  c.async = async;
  c.min_target_points = 100;
  return c;
}

const Vec2 kOriginA(500000.0, 4000000.0);
const Vec2 kOriginB(500200.0, 4000000.0);  // A の 200 m 東。地図の x 軸はグリッド北から 60°（東から 30°）

std::vector<MapGroup> twoGroups() { return {makeGroup("A", 500000, 4000000, 90), makeGroup("B", 500200, 4000000, 60)}; }

}  // namespace

TEST(MapTileManager, LoadsActiveGroupAroundPosition) {
  auto loader = std::make_shared<FakeLoader>();
  MapTileManager mgr(managerConfig(false), twoGroups(), loader, std::make_shared<FakeMatcher>());
  mgr.update(0.0, kOriginA + Vec2(5, 5), 0.0, 0.0, true);
  EXPECT_EQ(mgr.activeGroup(), "A");
  const auto target = mgr.currentTarget();
  ASSERT_TRUE(target);
  EXPECT_EQ(target->group, "A");
  EXPECT_EQ(target->tiles.size(), 16u);  // A の全タイルが 60 m 以内
  for (const auto& id : target->tiles) EXPECT_EQ(id.group, "A");
  EXPECT_EQ(mgr.stats().loaded_tiles, 16u);  // B（約 150 m 先）は読まない
  EXPECT_TRUE(mgr.overlappingGroups().empty());
}

TEST(MapTileManager, SwitchesGroupOnceWithHysteresis) {
  auto loader = std::make_shared<FakeLoader>();
  MapTileManager mgr(managerConfig(false), twoGroups(), loader, std::make_shared<FakeMatcher>());
  // A の中心から B の中心まで東へ 2 m ずつ進む
  std::string prev;
  int switches = 0;
  double switch_x = 0.0;
  bool saw_both_loaded = false;
  for (double x = 0.0; x <= 200.0; x += 2.0) {
    mgr.update(x, kOriginA + Vec2(x, 0.0), 0.0, 1.5, true);
    const std::string g = mgr.activeGroup();
    if (!prev.empty() && g != prev) {
      ++switches;
      switch_x = x;
    }
    prev = g;
    // 切り替える前から、B のタイルは先読みでキャッシュに入る
    const auto s = mgr.stats();
    if (g == "A" && s.loaded_tiles_per_group.count("B")) saw_both_loaded = true;
    const auto t = mgr.currentTarget();
    if (t) {
      for (const auto& id : t->tiles) EXPECT_EQ(id.group, t->group);  // 別のグループの点を混ぜない
    }
  }
  std::printf("[tile manager] switched A -> B at x = %.0f m\n", switch_x);
  EXPECT_EQ(switches, 1);
  EXPECT_EQ(prev, "B");
  EXPECT_TRUE(saw_both_loaded);
  // A から十分離れたら、A のタイルは捨てられている
  EXPECT_EQ(mgr.stats().loaded_tiles_per_group.count("A"), 0u);
  EXPECT_EQ(mgr.stats().loaded_tiles_per_group.at("B"), 16u);

  // 切り替えた地点の少し手前（A の方がまだ近いがマージン以内）に戻っても、B のまま
  mgr.update(300.0, kOriginA + Vec2(switch_x - 4.0, 0.0), kPi, 1.5, true);
  EXPECT_EQ(mgr.activeGroup(), "B");
}

TEST(MapTileManager, NoActiveGroupFarFromMaps) {
  MapTileManager mgr(managerConfig(false), twoGroups(), std::make_shared<FakeLoader>(), std::make_shared<FakeMatcher>());
  mgr.update(0.0, kOriginA + Vec2(0, 500), 0.0, 0.0, true);
  EXPECT_EQ(mgr.activeGroup(), "");
  EXPECT_FALSE(mgr.currentTarget());
  EXPECT_FALSE(mgr.hasMapWithin(kOriginA + Vec2(0, 500), 100.0));
  EXPECT_TRUE(mgr.hasMapWithin(kOriginA + Vec2(0, 50), 15.0));
  const auto [g, d] = mgr.nearestGroup(kOriginA + Vec2(0, 50));
  EXPECT_EQ(g, "A");
  EXPECT_NEAR(d, 10.0, 1e-9);
}

TEST(MapTileManager, DetectsOverlappingGroups) {
  auto groups = twoGroups();
  groups.push_back(makeGroup("C", 500050, 4000010, 90));  // A と 30 m ほど重なる
  MapTileManager mgr(managerConfig(false), groups, std::make_shared<FakeLoader>(), std::make_shared<FakeMatcher>());
  const auto ov = mgr.overlappingGroups();
  ASSERT_EQ(ov.size(), 1u);
  EXPECT_EQ(ov[0].first, "A");
  EXPECT_EQ(ov[0].second, "C");
}

TEST(MapTileManager, AsyncLoading) {
  auto loader = std::make_shared<FakeLoader>();
  MapTileManager mgr(managerConfig(true), twoGroups(), loader, std::make_shared<FakeMatcher>());
  mgr.update(0.0, kOriginB, 0.0, 0.0, true);
  mgr.waitIdle();
  const auto target = mgr.currentTarget();
  ASSERT_TRUE(target);
  EXPECT_EQ(target->group, "B");
  EXPECT_EQ(loader->loads.load(), 16);
  // 間隔が短く、ほとんど動いていなければ見直さない
  mgr.update(0.1, kOriginB + Vec2(0.2, 0), 0.0, 0.0);
  mgr.waitIdle();
  EXPECT_EQ(mgr.stats().target_builds, 1u);
}
