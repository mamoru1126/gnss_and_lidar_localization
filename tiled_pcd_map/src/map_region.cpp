#include "tiled_pcd_map/map_region.hpp"

namespace tiled_pcd_map {

std::vector<Vec3f> TileSetRegion::points() const {
  std::vector<Vec3f> out;
  out.reserve(num_points);
  for (const auto& t : tile_data) out.insert(out.end(), t->points.begin(), t->points.end());
  return out;
}

std::vector<Vec3> TileSetRegion::pointsUtm() const {
  std::vector<Vec3> out;
  out.reserve(num_points);
  for (const auto& t : tile_data)
    for (const auto& p : t->points) out.push_back(anchor.mapToUtm(p.cast<double>()));
  return out;
}

std::shared_ptr<const MapRegion> buildTileSetRegion(const std::string& group, const MapAnchor& anchor,
                                                    const std::vector<std::shared_ptr<const TileData>>& tiles) {
  auto r = std::make_shared<TileSetRegion>();
  r->group = group;
  r->anchor = anchor;
  r->tile_data = tiles;
  for (const auto& t : tiles) {
    r->tiles.push_back(t->id);
    r->num_points += t->points.size();
  }
  return r;
}

}  // namespace tiled_pcd_map
