// tiled_pcd_map_demo: 地図タイルの読み込みのデモ用の記録（tools/tile_demo/README.md）。
//
//   tiled_pcd_map_demo tile_index.yaml route.txt out.json [--speed 1.5] [--record-interval 0.5]
//
// 実際の MapTileManager（既定の設定、同期モード）を、経路（route.txt の折れ線）に沿って一定の速さで動かし、
// record-interval [s]（0.1 の倍数）ごとに自己位置と、読み込み中の領域（MapRegion）に入っているタイルを JSON に書く。
// route.txt は 1 行に「x y」（地図座標 [m]）。# で始まる行は無視する。
#include "tiled_pcd_map/map_anchor.hpp"
#include "tiled_pcd_map/map_config.hpp"
#include "tiled_pcd_map/map_tile_manager.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<tiled_pcd_map::Vec2> readRoute(const std::string& path) {
  std::ifstream is(path);
  if (!is) throw std::runtime_error("cannot open " + path);
  std::vector<tiled_pcd_map::Vec2> wp;
  std::string line;
  while (std::getline(is, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    double x, y;
    if (ls >> x >> y) wp.emplace_back(x, y);
  }
  if (wp.size() < 2) throw std::runtime_error(path + ": the route needs at least 2 points");
  return wp;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr,
                 "usage: tiled_pcd_map_demo tile_index.yaml route.txt out.json [--speed 1.5] [--record-interval 0.5]\n");
    return 2;
  }
  double speed = 1.5, record_interval = 0.5;
  for (int i = 4; i + 1 < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--speed") speed = std::atof(argv[++i]);
    if (a == "--record-interval") record_interval = std::atof(argv[++i]);
  }
  const double dt = 0.1;
  const long record_every = std::lround(record_interval / dt);  // 既定は 5 ステップ（0.5 s）ごと
  if (speed <= 0.0 || record_every < 1) {
    std::fprintf(stderr, "error: --speed must be > 0 and --record-interval >= 0.1\n");
    return 2;
  }

  try {
    using namespace tiled_pcd_map;
    const std::vector<Vec2> wp = readRoute(argv[2]);
    MapManagerConfig mc;  // 既定値（load 60 m / unload 90 m / 先読み 3 s）
    mc.async = false;
    MapGroup g;
    g.id = "demo";
    g.anchor = MapAnchor::identity();
    g.index = TileIndex::load(argv[1], g.id);
    const std::vector<TileMeta> metas = g.index.tiles;
    MapTileManager mgr(mc, {g}, std::make_shared<BinaryTileLoader>());  // 既定の領域（TileSetRegion）

    std::ofstream os(argv[3]);
    if (!os) throw std::runtime_error(std::string("cannot write ") + argv[3]);
    os << "{\"tile_size\":" << g.index.tile_size << ",\"load_radius\":" << mc.load_radius
       << ",\"unload_radius\":" << mc.unload_radius << ",\"lookahead_time\":" << mc.lookahead_time
       << ",\"speed\":" << speed << ",\"tiles\":[";
    for (std::size_t i = 0; i < metas.size(); ++i)
      os << (i ? "," : "") << "[" << metas[i].id.ix << "," << metas[i].id.iy << "," << metas[i].num_points << "]";
    os << "],\"frames\":[";

    double t = 0.0;
    long step = 0;
    bool first = true;
    std::size_t prev_loads = 0;
    for (std::size_t k = 0; k + 1 < wp.size(); ++k) {
      const Vec2 a = wp[k], b = wp[k + 1];
      const double len = (b - a).norm();
      if (len < 1e-6) continue;
      const double yaw = std::atan2(b.y() - a.y(), b.x() - a.x());
      for (double s = 0.0; s < len; s += speed * dt, t += dt, ++step) {
        const Vec2 p = a + (b - a) * (s / len);
        mgr.update(t, p, yaw, speed);
        mgr.waitIdle();
        if (step % record_every != 0) continue;
        const auto st = mgr.stats();
        const auto target = mgr.currentRegion();
        os << (first ? "" : ",") << "{\"t\":" << std::round(t * 10) / 10 << ",\"x\":" << p.x() << ",\"y\":" << p.y()
           << ",\"yaw\":" << yaw << ",\"loads\":" << st.tile_loads - prev_loads << ",\"builds\":" << st.target_builds
           << ",\"points\":" << st.target_points << ",\"tiles\":[";
        prev_loads = st.tile_loads;
        if (target) {
          bool f = true;
          for (const auto& id : target->tiles)
            for (std::size_t i = 0; i < metas.size(); ++i)
              if (metas[i].id == id) {
                os << (f ? "" : ",") << i;
                f = false;
              }
        }
        os << "]}";
        first = false;
      }
    }
    os << "]}\n";
    const auto st = mgr.stats();
    std::printf("%.1f s of driving, %zu tile loads, %zu target builds -> %s\n", t, st.tile_loads, st.target_builds,
                argv[3]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
