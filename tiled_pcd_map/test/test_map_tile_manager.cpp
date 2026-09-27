// 地図タイルマネージャ（ロード・アンロード・アクティブグループ・重なりの検出・領域の作り方）の単体テスト。
#include "tiled_pcd_map/map_tile_manager.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>

using namespace tiled_pcd_map;

namespace {

/// タイルの中身は使わないので、決まった数の点を返すだけのローダー。
class FakeLoader : public ITileLoader {
 public:
  std::shared_ptr<const TileData> load(const TileMeta& meta) const override {
    ++loads;
    auto t = std::make_shared<TileData>();
    t->id = meta.id;
    // タイルの中心に 100 点（地図座標）
    t->points.assign(100, Vec3f((meta.id.ix + 0.5f) * 20.0f, (meta.id.iy + 0.5f) * 20.0f, 0.0f));
    return t;
  }
  mutable std::atomic<int> loads{0};
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
  MapTileManager mgr(managerConfig(false), twoGroups(), loader);
  mgr.update(0.0, kOriginA + Vec2(5, 5), 0.0, 0.0, true);
  EXPECT_EQ(mgr.activeGroup(), "A");
  const auto target = mgr.currentRegion();
  ASSERT_TRUE(target);
  EXPECT_EQ(target->group, "A");
  EXPECT_EQ(target->tiles.size(), 16u);  // A の全タイルが 60 m 以内
  for (const auto& id : target->tiles) EXPECT_EQ(id.group, "A");
  EXPECT_EQ(mgr.stats().loaded_tiles, 16u);  // B（約 150 m 先）は読まない
  EXPECT_TRUE(mgr.overlappingGroups().empty());
}

TEST(MapTileManager, SwitchesGroupOnceWithHysteresis) {
  auto loader = std::make_shared<FakeLoader>();
  MapTileManager mgr(managerConfig(false), twoGroups(), loader);
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
    const auto t = mgr.currentRegion();
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
  MapTileManager mgr(managerConfig(false), twoGroups(), std::make_shared<FakeLoader>());
  mgr.update(0.0, kOriginA + Vec2(0, 500), 0.0, 0.0, true);
  EXPECT_EQ(mgr.activeGroup(), "");
  EXPECT_FALSE(mgr.currentRegion());
  EXPECT_FALSE(mgr.hasMapWithin(kOriginA + Vec2(0, 500), 100.0));
  EXPECT_TRUE(mgr.hasMapWithin(kOriginA + Vec2(0, 50), 15.0));
  const auto [g, d] = mgr.nearestGroup(kOriginA + Vec2(0, 50));
  EXPECT_EQ(g, "A");
  EXPECT_NEAR(d, 10.0, 1e-9);
}

TEST(MapTileManager, DetectsOverlappingGroups) {
  auto groups = twoGroups();
  groups.push_back(makeGroup("C", 500050, 4000010, 90));  // A と 30 m ほど重なる
  MapTileManager mgr(managerConfig(false), groups, std::make_shared<FakeLoader>());
  const auto ov = mgr.overlappingGroups();
  ASSERT_EQ(ov.size(), 1u);
  EXPECT_EQ(ov[0].first, "A");
  EXPECT_EQ(ov[0].second, "C");
}

TEST(MapTileManager, AsyncLoading) {
  auto loader = std::make_shared<FakeLoader>();
  MapTileManager mgr(managerConfig(true), twoGroups(), loader);
  mgr.update(0.0, kOriginB, 0.0, 0.0, true);
  mgr.waitIdle();
  const auto target = mgr.currentRegion();
  ASSERT_TRUE(target);
  EXPECT_EQ(target->group, "B");
  EXPECT_EQ(loader->loads.load(), 16);
  // 間隔が短く、ほとんど動いていなければ見直さない
  mgr.update(0.1, kOriginB + Vec2(0.2, 0), 0.0, 0.0);
  mgr.waitIdle();
  EXPECT_EQ(mgr.stats().target_builds, 1u);
}

TEST(MapTileManager, DefaultRegionHoldsTilePoints) {
  // 既定の領域（TileSetRegion）: 読み込んだタイルの点をそのまま持ち、アンカーで UTM にできる
  MapTileManager mgr(managerConfig(false), twoGroups(), std::make_shared<FakeLoader>());
  mgr.update(0.0, kOriginA + Vec2(5, 5), 0.0, 0.0, true);
  const auto r = mgr.currentRegionAs<TileSetRegion>();
  ASSERT_TRUE(r);
  EXPECT_EQ(r->tile_data.size(), 16u);
  EXPECT_EQ(r->points().size(), 1600u);
  const auto utm = r->pointsUtm();
  ASSERT_EQ(utm.size(), 1600u);
  // グループ A はアンカーが kOriginA、地図の x 軸がグリッド北から 90°（= UTM の東）なので、地図座標 + kOriginA
  const Vec3f& p = r->points().front();
  EXPECT_NEAR(utm.front().x(), kOriginA.x() + p.x(), 1e-6);
  EXPECT_NEAR(utm.front().y(), kOriginA.y() + p.y(), 1e-6);
}

TEST(MapTileManager, CustomBuilderAndCallback) {
  // 使う側の型の領域を作るビルダーと、差し替えのたびに呼ばれる関数
  struct Counted : MapRegion {
    std::size_t tile_count = 0;
  };
  int built = 0;
  const RegionBuilder builder = [&built](const std::string& group, const MapAnchor& anchor,
                                         const std::vector<std::shared_ptr<const TileData>>& tiles) {
    auto r = std::make_shared<Counted>();
    r->group = group;
    r->anchor = anchor;
    for (const auto& t : tiles) {
      r->tiles.push_back(t->id);
      r->num_points += t->points.size();
    }
    r->tile_count = tiles.size();
    ++built;
    return r;
  };
  MapTileManager mgr(managerConfig(true), twoGroups(), std::make_shared<FakeLoader>(), builder);
  std::vector<std::string> seen;
  mgr.setRegionCallback([&seen](std::shared_ptr<const MapRegion> r) { seen.push_back(r ? r->group : "(none)"); });
  mgr.update(0.0, kOriginA, 0.0, 0.0, true);
  mgr.waitIdle();
  const auto r = mgr.currentRegionAs<Counted>();
  ASSERT_TRUE(r);
  EXPECT_EQ(r->tile_count, 16u);
  EXPECT_EQ(built, 1);
  EXPECT_FALSE(mgr.currentRegionAs<TileSetRegion>());  // 型が違えば nullptr
  mgr.update(1.0, kOriginA + Vec2(0, 500), 0.0, 0.0, true);  // 地図から離れると領域が無くなる
  mgr.waitIdle();
  EXPECT_FALSE(mgr.currentRegion());
  EXPECT_EQ(seen, (std::vector<std::string>{"A", "(none)"}));
}
