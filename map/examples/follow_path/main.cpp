// gll_map の使い方の例: タイル化した地図の上を直線に動かし、読み込み中の領域を表示する。
//
//   follow_path <tile_index.yaml> <x0> <y0> <x1> <y1>
//
// 座標は地図座標 [m]（アンカーを恒等変換にしているので、UTM = 地図座標）。
#include <gll/map/map_config.hpp>
#include <gll/map/map_tile_manager.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
  if (argc != 6) {
    std::fprintf(stderr, "usage: follow_path tile_index.yaml x0 y0 x1 y1\n");
    return 2;
  }
  // 地図グループ（maps.yaml を使う場合は gll::loadMapSetConfig と gll::loadMapGroups）
  gll::MapGroup group;
  group.id = "map";
  group.anchor = gll::MapAnchor::identity();
  group.index = gll::TileIndex::load(argv[1], group.id);

  gll::MapManagerConfig cfg;  // 既定: load 60 m / unload 90 m / 先読み 3 s
  cfg.async = true;           // 別スレッドで読み込む
  gll::MapTileManager maps(cfg, {group}, std::make_shared<gll::BinaryTileLoader>());
  maps.setRegionCallback([](std::shared_ptr<const gll::MapRegion> r) {
    if (r) std::printf("  region updated: %zu tiles, %zu points\n", r->tiles.size(), r->num_points);
  });

  const gll::Vec2 a(std::atof(argv[2]), std::atof(argv[3])), b(std::atof(argv[4]), std::atof(argv[5]));
  const double yaw = std::atan2(b.y() - a.y(), b.x() - a.x()), speed = 1.5, dt = 0.1;
  const double length = (b - a).norm();
  double t = 0.0;
  for (double s = 0.0; s <= length; s += speed * dt, t += dt) {
    maps.update(t, a + (b - a) * (s / length), yaw, speed);  // 自己位置が来るたびに呼ぶ（すぐ戻る）
    if (std::fmod(s, 20.0) < speed * dt) {
      maps.waitIdle();  // 例なので読み込みを待つ（実際の使い方では待たない）
      // 使う側は、今の領域を shared_ptr で受け取る。既定の領域はタイルの点をそのまま持つ
      if (const auto r = maps.currentRegionAs<gll::TileSetRegion>())
        std::printf("s = %5.1f m: %zu tiles loaded, %zu points\n", s, r->tiles.size(), r->points().size());
    }
  }
  return 0;
}
