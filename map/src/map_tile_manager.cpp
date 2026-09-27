#include "gll/map/map_tile_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace gll {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double cross2(const Vec2& a, const Vec2& b) { return a.x() * b.y() - a.y() * b.x(); }

double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 ab = b - a;
  const double l2 = ab.squaredNorm();
  const double s = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
  return (a + s * ab - p).norm();
}

double boxDistance(const Vec2& lo, const Vec2& hi, const Vec2& p) {
  return (lo - p).cwiseMax(p - hi).cwiseMax(Vec2::Zero()).norm();
}

/// 2 つの凸四角形が、面積を持って重なるか（分離軸の判定）。
bool quadsOverlap(const std::array<Vec2, 4>& a, const std::array<Vec2, 4>& b) {
  const auto separated = [](const std::array<Vec2, 4>& p, const std::array<Vec2, 4>& q) {
    for (int k = 0; k < 4; ++k) {
      const Vec2 e = p[(k + 1) % 4] - p[k];
      const Vec2 n(-e.y(), e.x());
      double pmin = kInf, pmax = -kInf, qmin = kInf, qmax = -kInf;
      for (const auto& v : p) {
        pmin = std::min(pmin, n.dot(v));
        pmax = std::max(pmax, n.dot(v));
      }
      for (const auto& v : q) {
        qmin = std::min(qmin, n.dot(v));
        qmax = std::max(qmax, n.dot(v));
      }
      const double eps = 1e-6 * n.norm();
      if (pmax <= qmin + eps || qmax <= pmin + eps) return true;
    }
    return false;
  };
  return !separated(a, b) && !separated(b, a);
}

}  // namespace

MapTileManager::MapTileManager(const MapManagerConfig& cfg, std::vector<MapGroup> groups,
                               std::shared_ptr<const ITileLoader> loader, RegionBuilder builder,
                               std::shared_ptr<ILogger> logger)
    : cfg_(cfg),
      groups_(std::move(groups)),
      loader_(std::move(loader)),
      builder_(builder ? std::move(builder) : RegionBuilder(buildTileSetRegion)),
      logger_(logger ? std::move(logger) : std::make_shared<NullLogger>()) {
  for (std::size_t g = 0; g < groups_.size(); ++g) {
    const MapGroup& grp = groups_[g];
    const double ts = grp.index.tile_size;
    Vec2 glo(kInf, kInf), ghi(-kInf, -kInf);
    for (const TileMeta& m : grp.index.tiles) {
      TileEntry e;
      e.group_index = g;
      e.meta = m;
      const double x0 = m.id.ix * ts, y0 = m.id.iy * ts;
      const Vec2 c[4] = {Vec2(x0, y0), Vec2(x0 + ts, y0), Vec2(x0 + ts, y0 + ts), Vec2(x0, y0 + ts)};
      e.bb_min = Vec2(kInf, kInf);
      e.bb_max = Vec2(-kInf, -kInf);
      for (int k = 0; k < 4; ++k) {
        e.corners[k] = grp.anchor.mapToUtm(Vec3(c[k].x(), c[k].y(), 0.0)).head<2>();
        e.bb_min = e.bb_min.cwiseMin(e.corners[k]);
        e.bb_max = e.bb_max.cwiseMax(e.corners[k]);
      }
      glo = glo.cwiseMin(e.bb_min);
      ghi = ghi.cwiseMax(e.bb_max);
      tiles_.push_back(e);
    }
    group_bounds_.emplace_back(glo, ghi);
  }
  if (cfg_.async) worker_ = std::thread([this] { workerLoop(); });
}

MapTileManager::~MapTileManager() {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

double MapTileManager::distanceToTile(const TileEntry& e, const Vec2& p) const {
  bool inside = true;
  for (int k = 0; k < 4 && inside; ++k) inside = cross2(e.corners[(k + 1) % 4] - e.corners[k], p - e.corners[k]) >= 0.0;
  if (inside) return 0.0;
  double d = kInf;
  for (int k = 0; k < 4; ++k) d = std::min(d, pointSegmentDistance(p, e.corners[k], e.corners[(k + 1) % 4]));
  return d;
}

void MapTileManager::update(double t, const Vec2& p, double yaw, double speed, bool force) {
  std::lock_guard<std::mutex> ulk(update_mtx_);
  if (!force && have_last_ && (p - last_position_).norm() < cfg_.update_distance && t - last_t_ < cfg_.update_interval)
    return;
  have_last_ = true;
  last_position_ = p;
  last_t_ = t;

  // グループごとの、現在位置からタイルまでの距離
  std::vector<double> dg(groups_.size(), kInf);
  for (const TileEntry& e : tiles_) {
    double& d = dg[e.group_index];
    if (d == 0.0 || boxDistance(e.bb_min, e.bb_max, p) >= d) continue;
    d = std::min(d, distanceToTile(e, p));
  }
  int best = -1;
  for (std::size_t g = 0; g < groups_.size(); ++g)
    if (dg[g] <= cfg_.load_radius && (best < 0 || dg[g] < dg[static_cast<std::size_t>(best)])) best = static_cast<int>(g);

  // 必要なタイル（現在位置と、先読みした位置のまわり）
  const Vec2 ahead = p + cfg_.lookahead_time * speed * Vec2(std::cos(yaw), std::sin(yaw));
  const double R = cfg_.load_radius;
  std::set<std::size_t> needed;
  for (std::size_t i = 0; i < tiles_.size(); ++i) {
    const TileEntry& e = tiles_[i];
    const bool near_p = boxDistance(e.bb_min, e.bb_max, p) <= R && distanceToTile(e, p) <= R;
    const bool near_a = !near_p && boxDistance(e.bb_min, e.bb_max, ahead) <= R && distanceToTile(e, ahead) <= R;
    if (near_p || near_a) needed.insert(i);
  }

  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    int active = active_;
    if (active >= 0 && dg[static_cast<std::size_t>(active)] <= cfg_.load_radius) {
      // ヒステリシス: ほかのグループが group_switch_margin 以上近くなったときだけ切り替える
      if (best >= 0 && best != active &&
          dg[static_cast<std::size_t>(best)] < dg[static_cast<std::size_t>(active)] - cfg_.group_switch_margin)
        active = best;
    } else {
      active = best;
    }
    if (active != active_) {
      const std::string from = active_ >= 0 ? groups_[static_cast<std::size_t>(active_)].id : "(none)";
      const std::string to = active >= 0 ? groups_[static_cast<std::size_t>(active)].id : "(none)";
      logger_->info("active map group: " + from + " -> " + to);
    }
    active_ = active;
    stats_.active_group = active >= 0 ? groups_[static_cast<std::size_t>(active)].id : "";
    changed = needed != requested_tiles_ || active != requested_active_;
    requested_tiles_ = std::move(needed);
    requested_active_ = active;
    requested_position_ = p;
    if (changed) pending_ = true;
  }
  if (!changed) return;
  if (cfg_.async) {
    cv_.notify_one();
  } else {
    processPending();
  }
}

void MapTileManager::workerLoop() {
  std::unique_lock<std::mutex> lk(mtx_);
  while (true) {
    cv_.wait(lk, [this] { return stop_ || pending_; });
    if (stop_) break;
    lk.unlock();
    processPending();
    lk.lock();
  }
}

void MapTileManager::processPending() {
  std::set<std::size_t> needed;
  int active = -1;
  Vec2 pos = Vec2::Zero();
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!pending_) return;
    needed = requested_tiles_;
    active = requested_active_;
    pos = requested_position_;
    pending_ = false;
    busy_ = true;
  }

  std::size_t loads = 0, failures = 0;
  for (const std::size_t i : needed) {
    if (cache_.count(i)) continue;
    try {
      cache_[i] = loader_->load(tiles_[i].meta);
      ++loads;
    } catch (const std::exception& e) {
      ++failures;
      logger_->error(std::string("failed to load tile ") + tiles_[i].meta.id.str() + ": " + e.what());
    }
  }
  for (auto it = cache_.begin(); it != cache_.end();) {
    if (!needed.count(it->first) && distanceToTile(tiles_[it->first], pos) > cfg_.unload_radius) {
      it = cache_.erase(it);
    } else {
      ++it;
    }
  }

  std::set<std::size_t> tset;
  if (active >= 0)
    for (const auto& [i, data] : cache_)
      if (static_cast<int>(tiles_[i].group_index) == active) tset.insert(i);
  const bool rebuild = active != target_group_ || tset != target_tiles_;
  std::shared_ptr<const MapRegion> target;
  if (rebuild && active >= 0 && !tset.empty()) {
    std::vector<std::shared_ptr<const TileData>> data;
    for (const std::size_t i : tset) data.push_back(cache_.at(i));
    const MapGroup& g = groups_[static_cast<std::size_t>(active)];
    target = builder_(g.id, g.anchor, data);
    if (target && static_cast<int>(target->num_points) < cfg_.min_target_points) {
      std::ostringstream os;
      os << "map group " << g.id << ": only " << target->num_points << " points around here (min_target_points "
         << cfg_.min_target_points << "), no region";
      logger_->warn(os.str());
      target.reset();
    }
  }
  if (rebuild) {
    target_group_ = active;
    target_tiles_ = tset;
  }

  RegionCallback cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (rebuild) {
      front_ = target;
      cb = on_region_;
      ++stats_.target_builds;
      stats_.target_tiles = target ? target->tiles.size() : 0;
      stats_.target_points = target ? target->num_points : 0;
    }
    stats_.loaded_tiles = cache_.size();
    stats_.loaded_tiles_per_group.clear();
    for (const auto& [i, data] : cache_) ++stats_.loaded_tiles_per_group[groups_[tiles_[i].group_index].id];
    stats_.tile_loads += loads;
    stats_.tile_load_failures += failures;
  }
  if (cb) cb(target);  // waitIdle() がこの呼び出しの後に戻るよう、busy_ はここで下ろす
  {
    std::lock_guard<std::mutex> lk(mtx_);
    busy_ = false;
  }
  idle_cv_.notify_all();
}

std::shared_ptr<const MapRegion> MapTileManager::currentRegion() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return front_;
}

void MapTileManager::setRegionCallback(RegionCallback cb) {
  std::lock_guard<std::mutex> lk(mtx_);
  on_region_ = std::move(cb);
}

std::string MapTileManager::activeGroup() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return active_ >= 0 ? groups_[static_cast<std::size_t>(active_)].id : std::string();
}

bool MapTileManager::hasMapWithin(const Vec2& p, double radius) const {
  for (const TileEntry& e : tiles_)
    if (boxDistance(e.bb_min, e.bb_max, p) <= radius && distanceToTile(e, p) <= radius) return true;
  return false;
}

std::pair<std::string, double> MapTileManager::nearestGroup(const Vec2& p) const {
  std::pair<std::string, double> best{"", kInf};
  for (const TileEntry& e : tiles_) {
    if (boxDistance(e.bb_min, e.bb_max, p) >= best.second) continue;
    const double d = distanceToTile(e, p);
    if (d < best.second) best = {groups_[e.group_index].id, d};
  }
  return best;
}

const MapGroup* MapTileManager::group(const std::string& id) const {
  for (const auto& g : groups_)
    if (g.id == id) return &g;
  return nullptr;
}

std::vector<std::pair<std::string, std::string>> MapTileManager::overlappingGroups() const {
  std::vector<std::pair<std::string, std::string>> out;
  for (std::size_t a = 0; a < groups_.size(); ++a) {
    for (std::size_t b = a + 1; b < groups_.size(); ++b) {
      const auto& [alo, ahi] = group_bounds_[a];
      const auto& [blo, bhi] = group_bounds_[b];
      if ((alo.array() > bhi.array()).any() || (blo.array() > ahi.array()).any()) continue;
      bool found = false;
      for (std::size_t i = 0; i < tiles_.size() && !found; ++i) {
        if (tiles_[i].group_index != a) continue;
        for (std::size_t j = 0; j < tiles_.size() && !found; ++j) {
          if (tiles_[j].group_index != b) continue;
          const auto& ei = tiles_[i];
          const auto& ej = tiles_[j];
          if ((ei.bb_min.array() >= ej.bb_max.array()).any() || (ej.bb_min.array() >= ei.bb_max.array()).any()) continue;
          found = quadsOverlap(ei.corners, ej.corners);
        }
      }
      if (found) out.emplace_back(groups_[a].id, groups_[b].id);
    }
  }
  return out;
}

void MapTileManager::waitIdle() {
  std::unique_lock<std::mutex> lk(mtx_);
  idle_cv_.wait(lk, [this] { return !pending_ && !busy_; });
}

MapTileManager::Stats MapTileManager::stats() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return stats_;
}

}  // namespace gll
