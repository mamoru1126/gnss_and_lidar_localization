#include "gll/map/map_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <set>
#include <stdexcept>

namespace gll {
namespace {

AnchorConfig parseAnchor(const YAML::Node& n, const std::string& group) {
  AnchorConfig a;
  const auto req = [&](const char* key) {
    if (!n[key]) throw std::runtime_error("maps.yaml: group '" + group + "': anchor." + key + " is missing");
    return n[key];
  };
  if (n["map_point"]) {
    const YAML::Node mp = n["map_point"];
    if (!mp.IsSequence() || mp.size() != 3)
      throw std::runtime_error("maps.yaml: group '" + group + "': anchor.map_point must be [x, y, z]");
    a.map_point = Vec3(mp[0].as<double>(), mp[1].as<double>(), mp[2].as<double>());
  }
  if (n["easting"] || n["northing"]) {
    a.use_utm = true;
    a.easting = req("easting").as<double>();
    a.northing = req("northing").as<double>();
    a.grid_heading = deg2rad(req("grid_heading_deg").as<double>());
  } else {
    a.latitude = req("latitude").as<double>();
    a.longitude = req("longitude").as<double>();
    a.heading = deg2rad(req("heading_deg").as<double>());
  }
  a.ellipsoid_height = n["ellipsoid_height"].as<double>(0.0);
  a.use_scale_factor = n["use_scale_factor"].as<bool>(true);
  a.stddev_xy = n["stddev_xy"].as<double>(a.stddev_xy);
  if (n["stddev_yaw_deg"]) a.stddev_yaw = deg2rad(n["stddev_yaw_deg"].as<double>());
  return a;
}

}  // namespace

MapSetConfig loadMapSetConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot read map config '" + path + "': " + e.what());
  }
  MapSetConfig cfg;
  if (const YAML::Node utm = root["utm"]) {
    if (utm["zone"]) cfg.utm_zone = utm["zone"].as<int>();
    if (utm["hemisphere"]) {
      const std::string h = utm["hemisphere"].as<std::string>();
      if (h != "north" && h != "south") throw std::runtime_error("maps.yaml: utm.hemisphere must be north or south");
      cfg.utm_north = (h == "north");
    }
  }
  const std::filesystem::path base = std::filesystem::path(path).parent_path();
  const YAML::Node groups = root["map_groups"];
  if (!groups || !groups.IsSequence() || groups.size() == 0)
    throw std::runtime_error("maps.yaml '" + path + "': 'map_groups' is missing or empty");
  std::set<std::string> ids;
  for (const auto& g : groups) {
    MapGroupConfig mg;
    mg.id = g["id"].as<std::string>("");
    if (mg.id.empty()) throw std::runtime_error("maps.yaml: a map group has no id");
    if (!ids.insert(mg.id).second) throw std::runtime_error("maps.yaml: duplicate group id '" + mg.id + "'");
    if (!g["tile_index"]) throw std::runtime_error("maps.yaml: group '" + mg.id + "': tile_index is missing");
    const std::filesystem::path ti = g["tile_index"].as<std::string>();
    mg.tile_index = (ti.is_absolute() ? ti : base / ti).lexically_normal().string();
    const YAML::Node an = g["anchor"];
    if (!an) throw std::runtime_error("maps.yaml: group '" + mg.id + "': anchor is missing");
    if (an.IsScalar() && an.as<std::string>() == "local") {
      mg.local_anchor = true;
    } else {
      mg.anchor = parseAnchor(an, mg.id);
    }
    cfg.groups.push_back(mg);
  }
  return cfg;
}

std::vector<MapGroup> loadMapGroups(const MapSetConfig& cfg, const UtmProjector& utm) {
  std::vector<MapGroup> out;
  for (const auto& g : cfg.groups) {
    MapGroup mg;
    mg.id = g.id;
    mg.anchor = g.local_anchor ? MapAnchor::identity() : MapAnchor::fromConfig(g.anchor, utm);
    mg.index = TileIndex::load(g.tile_index, g.id);
    out.push_back(std::move(mg));
  }
  return out;
}

}  // namespace gll
