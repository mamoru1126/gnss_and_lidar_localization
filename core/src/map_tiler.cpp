// small_gicp の kdtree.hpp は <cstdint> を含んでいないので、先に含める（gicp_matcher.cpp を参照）。
#include <cstdint>

#include "gll/map/map_tiler.hpp"

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <filesystem>
#include <map>
#include <utility>

namespace gll {

TilerResult tileMap(const std::vector<Vec3f>& points, const TilerOptions& opt, const std::string& group) {
  TilerResult res;
  res.input_points = points.size();
  res.index.tile_size = opt.tile_size;
  res.index.voxel_size = opt.voxel_size;
  res.index.num_neighbors = opt.num_neighbors;
  if (points.empty()) return res;

  using Cloud = small_gicp::PointCloud;
  const Cloud raw(points);
  std::shared_ptr<Cloud> cloud =
      opt.voxel_size > 0.0 ? small_gicp::voxelgrid_sampling_omp(raw, opt.voxel_size, opt.num_threads)
                           : std::make_shared<Cloud>(raw);
  const small_gicp::KdTree<Cloud> tree(cloud, small_gicp::KdTreeBuilderOMP(opt.num_threads));
  small_gicp::estimate_covariances_omp(*cloud, tree, opt.num_neighbors, opt.num_threads);
  res.output_points = cloud->size();

  std::map<std::pair<int, int>, TileData> tiles;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const Eigen::Vector4d& p = cloud->points[i];
    const auto key = std::make_pair(tileCoord(p.x(), opt.tile_size), tileCoord(p.y(), opt.tile_size));
    TileData& t = tiles[key];
    t.points.emplace_back(p.head<3>().cast<float>());
    const Eigen::Matrix4d& c = cloud->covs[i];
    t.covs.push_back(PackedCov{static_cast<float>(c(0, 0)), static_cast<float>(c(0, 1)), static_cast<float>(c(0, 2)),
                               static_cast<float>(c(1, 1)), static_cast<float>(c(1, 2)), static_cast<float>(c(2, 2))});
  }
  for (auto& [key, t] : tiles) {
    t.id = TileId{group, key.first, key.second};
    TileMeta m;
    m.id = t.id;
    m.file = "tiles/" + std::to_string(key.first) + "_" + std::to_string(key.second) + ".bin";
    m.num_points = t.points.size();
    Vec3f lo = t.points.front(), hi = t.points.front();
    for (const auto& q : t.points) {
      lo = lo.cwiseMin(q);
      hi = hi.cwiseMax(q);
    }
    m.bounds_min = lo.cast<double>();
    m.bounds_max = hi.cast<double>();
    res.index.tiles.push_back(m);
    res.tiles.push_back(std::move(t));
  }
  return res;
}

void writeTiles(const std::string& dir, const TilerResult& r) {
  namespace fs = std::filesystem;
  fs::create_directories(fs::path(dir) / "tiles");
  for (const auto& t : r.tiles)
    writeTileFile((fs::path(dir) / "tiles" / (std::to_string(t.id.ix) + "_" + std::to_string(t.id.iy) + ".bin")).string(), t);
  r.index.save((fs::path(dir) / "tile_index.yaml").string());
}

}  // namespace gll
