// small_gicp の kdtree.hpp は <cstdint> を含んでいないので、先に含める。
#include <cstdint>

#include "gll/map/map_tiler.hpp"

#ifdef GLL_MAP_HAVE_SMALL_GICP
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#endif

#include <cmath>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace gll {

bool tilerCanComputeCovariance() {
#ifdef GLL_MAP_HAVE_SMALL_GICP
  return true;
#else
  return false;
#endif
}

std::vector<Vec3f> voxelDownsample(const std::vector<Vec3f>& points, double voxel_size) {
  if (voxel_size <= 0.0 || points.empty()) return points;
  Eigen::Vector3d lo = points.front().cast<double>();
  for (const auto& p : points) lo = lo.cwiseMin(p.cast<double>());
  constexpr std::int64_t kBits = 21, kMax = (std::int64_t(1) << kBits) - 1;
  struct Acc {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    std::uint32_t n = 0;
  };
  std::unordered_map<std::uint64_t, Acc> cells;
  cells.reserve(points.size() / 2 + 1);
  std::vector<std::uint64_t> order;  // 出力の順を決めるため、最初に現れた順に覚える
  for (const auto& p : points) {
    const Eigen::Vector3d q = (p.cast<double>() - lo) / voxel_size;
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(q.x()));
    const std::int64_t iy = static_cast<std::int64_t>(std::floor(q.y()));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(q.z()));
    if (ix > kMax || iy > kMax || iz > kMax)
      throw std::runtime_error("voxelDownsample: the point cloud is too large for the voxel size");
    const std::uint64_t key = (static_cast<std::uint64_t>(ix) << (2 * kBits)) |
                              (static_cast<std::uint64_t>(iy) << kBits) | static_cast<std::uint64_t>(iz);
    Acc& a = cells[key];
    if (a.n == 0) order.push_back(key);
    a.sum += p.cast<double>();
    ++a.n;
  }
  std::vector<Vec3f> out;
  out.reserve(order.size());
  for (const auto key : order) {
    const Acc& a = cells[key];
    out.emplace_back((a.sum / a.n).cast<float>());
  }
  return out;
}

TilerResult tileMap(const std::vector<Vec3f>& points, const TilerOptions& opt, const std::string& group) {
  TilerResult res;
  res.input_points = points.size();
  res.index.tile_size = opt.tile_size;
  res.index.voxel_size = opt.voxel_size;
  res.index.num_neighbors = opt.num_neighbors;
  if (points.empty()) return res;

  const std::vector<Vec3f> pts = voxelDownsample(points, opt.voxel_size);
  res.output_points = pts.size();

  // 点ごとの共分散は、分割する前の点群全体で計算する（タイルの境界で近傍点が欠けないように。設計書 6.2 節）
  std::vector<PackedCov> covs;
#ifdef GLL_MAP_HAVE_SMALL_GICP
  if (opt.compute_covariance) {
    using Cloud = small_gicp::PointCloud;
    auto cloud = std::make_shared<Cloud>(pts);
    const small_gicp::KdTree<Cloud> tree(cloud, small_gicp::KdTreeBuilderOMP(opt.num_threads));
    small_gicp::estimate_covariances_omp(*cloud, tree, opt.num_neighbors, opt.num_threads);
    covs.reserve(pts.size());
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      const Eigen::Matrix4d& c = cloud->covs[i];
      covs.push_back(PackedCov{static_cast<float>(c(0, 0)), static_cast<float>(c(0, 1)), static_cast<float>(c(0, 2)),
                               static_cast<float>(c(1, 1)), static_cast<float>(c(1, 2)), static_cast<float>(c(2, 2))});
    }
  }
#endif
  res.has_covariance = !covs.empty();

  std::map<std::pair<int, int>, TileData> tiles;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const Vec3f& p = pts[i];
    const auto key = std::make_pair(tileCoord(p.x(), opt.tile_size), tileCoord(p.y(), opt.tile_size));
    TileData& t = tiles[key];
    t.points.push_back(p);
    if (res.has_covariance) t.covs.push_back(covs[i]);
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
