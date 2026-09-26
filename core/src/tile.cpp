#include "gll/map/tile.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace gll {
namespace {

constexpr char kTileMagic[8] = {'G', 'L', 'L', 'T', 'I', 'L', 'E', '1'};

Vec3 readVec3(const YAML::Node& n, const std::string& what) {
  if (!n || !n.IsSequence() || n.size() != 3) throw std::runtime_error("tile index: '" + what + "' must be [x, y, z]");
  return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

std::string fmt3(const Vec3& v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return os.str();
}

}  // namespace

TileIndex TileIndex::load(const std::string& path, const std::string& group) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot read tile index '" + path + "': " + e.what());
  }
  if (root["format"] && root["format"].as<std::string>() != "gll_tiles_v1")
    throw std::runtime_error("tile index '" + path + "': unsupported format " + root["format"].as<std::string>());
  TileIndex idx;
  idx.tile_size = root["tile_size"].as<double>(20.0);
  idx.voxel_size = root["voxel_size"].as<double>(0.0);
  idx.num_neighbors = root["num_neighbors"].as<int>(0);
  const std::filesystem::path base = std::filesystem::path(path).parent_path();
  const YAML::Node tiles = root["tiles"];
  if (!tiles || !(tiles.IsSequence() || tiles.IsNull()))
    throw std::runtime_error("tile index '" + path + "': 'tiles' is missing");
  for (const auto& t : tiles) {
    TileMeta m;
    m.id.group = group;
    m.id.ix = t["ix"].as<int>();
    m.id.iy = t["iy"].as<int>();
    const std::filesystem::path f = t["file"].as<std::string>();
    m.file = (f.is_absolute() ? f : base / f).lexically_normal().string();
    m.num_points = t["num_points"].as<std::size_t>(0);
    m.bounds_min = readVec3(t["bounds_min"], "bounds_min");
    m.bounds_max = readVec3(t["bounds_max"], "bounds_max");
    idx.tiles.push_back(m);
  }
  return idx;
}

void TileIndex::save(const std::string& path) const {
  std::ofstream os(path);
  if (!os) throw std::runtime_error("cannot write tile index '" + path + "'");
  os << "# gll_map_tiler が生成したタイルの索引（設計書 5.2 節）\n";
  os << "format: gll_tiles_v1\n";
  os << "tile_size: " << tile_size << "\n";
  os << "voxel_size: " << voxel_size << "\n";
  os << "num_neighbors: " << num_neighbors << "\n";
  os << (tiles.empty() ? "tiles: []\n" : "tiles:\n");
  for (const auto& t : tiles) {
    os << "  - {ix: " << t.id.ix << ", iy: " << t.id.iy << ", file: tiles/" << t.id.ix << "_" << t.id.iy
       << ".bin, num_points: " << t.num_points << ", bounds_min: " << fmt3(t.bounds_min)
       << ", bounds_max: " << fmt3(t.bounds_max) << "}\n";
  }
}

void writeTileFile(const std::string& path, const TileData& tile) {
  if (!tile.covs.empty() && tile.covs.size() != tile.points.size())
    throw std::runtime_error("tile " + tile.id.str() + ": covs and points differ in size");
  std::ofstream os(path, std::ios::binary);
  if (!os) throw std::runtime_error("cannot write tile file '" + path + "'");
  const std::uint64_t n = tile.points.size();
  const std::uint32_t flags = tile.covs.empty() ? 0u : 1u;  // bit 0: 共分散あり
  os.write(kTileMagic, sizeof(kTileMagic));
  os.write(reinterpret_cast<const char*>(&n), sizeof(n));
  os.write(reinterpret_cast<const char*>(&flags), sizeof(flags));
  for (const auto& p : tile.points) os.write(reinterpret_cast<const char*>(p.data()), 3 * sizeof(float));
  for (const auto& c : tile.covs) os.write(reinterpret_cast<const char*>(c.data()), 6 * sizeof(float));
  if (!os) throw std::runtime_error("failed to write tile file '" + path + "'");
}

TileData readTileFile(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) throw std::runtime_error("cannot open tile file '" + path + "'");
  char magic[8];
  std::uint64_t n = 0;
  std::uint32_t flags = 0;
  is.read(magic, sizeof(magic));
  is.read(reinterpret_cast<char*>(&n), sizeof(n));
  is.read(reinterpret_cast<char*>(&flags), sizeof(flags));
  if (!is || std::memcmp(magic, kTileMagic, sizeof(kTileMagic)) != 0)
    throw std::runtime_error("'" + path + "' is not a gll tile file");
  TileData t;
  t.points.resize(n);
  for (auto& p : t.points) is.read(reinterpret_cast<char*>(p.data()), 3 * sizeof(float));
  if (flags & 1u) {
    t.covs.resize(n);
    for (auto& c : t.covs) is.read(reinterpret_cast<char*>(c.data()), 6 * sizeof(float));
  }
  if (!is) throw std::runtime_error("tile file '" + path + "' is truncated");
  return t;
}

std::shared_ptr<const TileData> BinaryTileLoader::load(const TileMeta& meta) const {
  auto t = std::make_shared<TileData>(readTileFile(meta.file));
  t->id = meta.id;
  return t;
}

}  // namespace gll
