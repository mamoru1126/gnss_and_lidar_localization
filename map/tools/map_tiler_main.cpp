// gll_map_tiler: 統合済みの点群地図（PCD）をタイルに分割する（設計書 5.2 節）。
//
//   gll_map_tiler -i map.pcd [-i more.pcd ...] -o out_dir [--tile-size 20] [--voxel-size 0.2]
//                 [--num-neighbors 20] [--threads 4] [--no-covariance]
//
// out_dir/tiles/<ix>_<iy>.bin と out_dir/tile_index.yaml を書く。1 回の実行で 1 つの地図グループを作る。
#include "gll/map/map_tiler.hpp"
#include "gll/map/pcd_io.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: gll_map_tiler -i map.pcd [-i more.pcd ...] -o out_dir [--tile-size 20] [--voxel-size 0.2]\n"
               "                     [--num-neighbors 20] [--threads 4] [--no-covariance]\n"
               "  --no-covariance: 点ごとの共分散を計算しない（NDT など、共分散の要らない照合に使う場合）\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> inputs;
  std::string out_dir;
  gll::TilerOptions opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "-i" || a == "--input") {
      inputs.push_back(next());
    } else if (a == "-o" || a == "--output") {
      out_dir = next();
    } else if (a == "--tile-size") {
      opt.tile_size = std::stod(next());
    } else if (a == "--voxel-size") {
      opt.voxel_size = std::stod(next());
    } else if (a == "--num-neighbors") {
      opt.num_neighbors = std::stoi(next());
    } else if (a == "--threads") {
      opt.num_threads = std::stoi(next());
    } else if (a == "--no-covariance") {
      opt.compute_covariance = false;
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "unknown option: %s\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (inputs.empty() || out_dir.empty() || opt.tile_size <= 0.0) {
    usage();
    return 2;
  }

  try {
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<gll::Vec3f> points;
    for (const auto& path : inputs) {
      const auto p = gll::readPcd(path);
      std::printf("read %zu points from %s\n", p.size(), path.c_str());
      points.insert(points.end(), p.begin(), p.end());
    }
    if (opt.compute_covariance && !gll::tilerCanComputeCovariance())
      std::printf("note: gll_map was built without small_gicp; tiles are written without per-point covariances\n");
    const gll::TilerResult r = gll::tileMap(points, opt);
    gll::writeTiles(out_dir, r);
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("wrote %zu tiles (%zu points after %.2f m voxel, %s) to %s in %.1f s\n", r.tiles.size(),
                r.output_points, opt.voxel_size, r.has_covariance ? "with covariances" : "no covariances",
                out_dir.c_str(), sec);
    std::printf(
        "\nmaps.yaml に次のように登録してください（アンカーは地図の 1 点の緯度経度と、地図の x 軸の方位）:\n"
        "map_groups:\n"
        "  - id: <group_id>\n"
        "    tile_index: %s/tile_index.yaml\n"
        "    anchor:\n"
        "      map_point: [0.0, 0.0, 0.0]\n"
        "      latitude: <deg>\n"
        "      longitude: <deg>\n"
        "      ellipsoid_height: <m>\n"
        "      heading_deg: <地図の x 軸の方位。真北から時計回り>\n",
        out_dir.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
