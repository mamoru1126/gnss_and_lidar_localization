// small_gicp の kdtree.hpp は std::uint32_t を使うが <cstdint> を含んでいないので、先に含める
// （GCC 13 と Eigen 3.3 の組み合わせではコンパイルエラーになる）。
#include <cstdint>

#include "gll/matching/gicp_matcher.hpp"

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace gll {
namespace {

using SgCloud = small_gicp::PointCloud;
using SgTree = small_gicp::KdTree<SgCloud>;
using SgVoxelMap = small_gicp::GaussianVoxelMap;

/// 法線がこれより鉛直に近い点（地面・天井）は、overlap の計算から外す。
/// 地面はどの水平位置でも重なるので、含めると誤った解との差が小さくなるため。
constexpr double kHorizontalNormalZ = 0.7;
/// 水平でない面（壁・柱など）の点がこれより少ない場合は、すべての点で overlap を数える。
constexpr std::size_t kMinStructurePoints = 50;

class GicpTarget : public MatchTarget {
 public:
  std::shared_ptr<SgCloud> cloud;
  std::shared_ptr<SgTree> tree;
  double coarse_voxel = 2.0;
  double fine_voxel = 1.0;

  /// 粗い位置合わせ用のボクセル地図（地図上での初期化と再位置推定のときだけ必要なので、初回に作る）。
  const SgVoxelMap& voxels() const {
    std::call_once(voxel_once_, [this] {
      voxelmap_ = std::make_shared<SgVoxelMap>(coarse_voxel);
      voxelmap_->insert(*cloud);
    });
    return *voxelmap_;
  }

  /// 追跡の VGICP 用のボクセル地図（lidar.registration: vgicp のとき、初回に作る）。
  const SgVoxelMap& fineVoxels() const {
    std::call_once(fine_once_, [this] {
      fine_voxelmap_ = std::make_shared<SgVoxelMap>(fine_voxel);
      fine_voxelmap_->insert(*cloud);
    });
    return *fine_voxelmap_;
  }

  std::optional<double> groundHeight(double x, double y, double radius, double fraction) const override {
    std::vector<double> zs;
    const double r2 = radius * radius;
    for (const auto& p : cloud->points) {
      const double dx = p.x() - x, dy = p.y() - y;
      if (dx * dx + dy * dy <= r2) zs.push_back(p.z());
    }
    if (zs.empty()) return std::nullopt;
    const auto k = std::min(zs.size() - 1, static_cast<std::size_t>(std::max(0.0, fraction) * zs.size()));
    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(k), zs.end());
    return zs[k];
  }

  std::vector<Vec3f> samplePoints(std::size_t max_points) const override {
    std::vector<Vec3f> out;
    if (!cloud || cloud->size() == 0 || max_points == 0) return out;
    const std::size_t stride = std::max<std::size_t>(1, (cloud->size() + max_points - 1) / max_points);
    out.reserve(cloud->size() / stride + 1);
    for (std::size_t i = 0; i < cloud->size(); i += stride) out.push_back(cloud->points[i].head<3>().cast<float>());
    return out;
  }

 private:
  mutable std::once_flag voxel_once_;
  mutable std::shared_ptr<SgVoxelMap> voxelmap_;
  mutable std::once_flag fine_once_;
  mutable std::shared_ptr<SgVoxelMap> fine_voxelmap_;
};

class GicpSource : public SourceCloud {
 public:
  std::shared_ptr<SgCloud> cloud;
  std::vector<std::size_t> structure;  ///< 水平でない面（法線が鉛直に近くない点）の添字（overlap の計算に使う）
  std::size_t size() const override { return cloud ? cloud->size() : 0; }
};

const GicpTarget& asTarget(const MatchTarget& t) {
  const auto* p = dynamic_cast<const GicpTarget*>(&t);
  if (!p) throw std::invalid_argument("GicpMatcher: target was not built by GicpMatcher");
  return *p;
}

const GicpSource& asSource(const SourceCloud& s) {
  const auto* p = dynamic_cast<const GicpSource*>(&s);
  if (!p) throw std::invalid_argument("GicpMatcher: source was not prepared by GicpMatcher");
  return *p;
}

Eigen::Matrix4d unpackCov(const PackedCov& c) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
  m(0, 0) = c[0];
  m(0, 1) = m(1, 0) = c[1];
  m(0, 2) = m(2, 0) = c[2];
  m(1, 1) = c[3];
  m(1, 2) = m(2, 1) = c[4];
  m(2, 2) = c[5];
  return m;
}

double yawDiff(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return std::abs(wrapAngle(yawOf(a.linear()) - yawOf(b.linear())));
}

}  // namespace

GicpMatcher::GicpMatcher(const LidarConfig& lidar, const RelocalizeConfig& reloc) : cfg_(lidar), reloc_(reloc) {
  if (cfg_.registration != "gicp" && cfg_.registration != "vgicp")
    throw std::invalid_argument("lidar.registration must be gicp or vgicp: " + cfg_.registration);
  if (cfg_.registration == "vgicp" && !(cfg_.vgicp_voxel_size > 0.0))
    throw std::invalid_argument("lidar.vgicp_voxel_size must be positive");
}

std::shared_ptr<const MatchTarget> GicpMatcher::buildTarget(
    const std::string& group, const MapAnchor& anchor,
    const std::vector<std::shared_ptr<const TileData>>& tiles) const {
  auto t = std::make_shared<GicpTarget>();
  t->group = group;
  t->anchor = anchor;
  t->coarse_voxel = reloc_.coarse_voxel_size;
  t->fine_voxel = cfg_.vgicp_voxel_size;
  std::size_t n = 0;
  for (const auto& tile : tiles) n += tile->points.size();
  t->cloud = std::make_shared<SgCloud>();
  t->cloud->points.resize(n);
  t->cloud->covs.resize(n);
  bool all_covs = true;
  std::size_t k = 0;
  for (const auto& tile : tiles) {
    t->tiles.push_back(tile->id);
    const bool has_cov = tile->covs.size() == tile->points.size();
    all_covs = all_covs && has_cov;
    for (std::size_t i = 0; i < tile->points.size(); ++i, ++k) {
      t->cloud->points[k] << tile->points[i].cast<double>(), 1.0;
      t->cloud->covs[k] = has_cov ? unpackCov(tile->covs[i]) : Eigen::Matrix4d::Zero();
    }
  }
  t->num_points = n;
  if (n == 0) return t;
  t->tree = std::make_shared<SgTree>(t->cloud, small_gicp::KdTreeBuilderOMP(cfg_.num_threads));
  if (!all_covs) {
    // 共分散を持たないタイル（外部で作ったタイルなど）が混ざっていれば、ここで計算する
    small_gicp::estimate_covariances_omp(*t->cloud, *t->tree, 20, cfg_.num_threads);
    t->cloud->normals.clear();
  }
  return t;
}

std::shared_ptr<const SourceCloud> GicpMatcher::prepareSource(const std::vector<Vec3f>& points_base,
                                                              double voxel_size) const {
  auto src = std::make_shared<GicpSource>();
  const SgCloud raw(points_base);
  src->cloud = small_gicp::voxelgrid_sampling_omp(raw, voxel_size, cfg_.num_threads);
  if (src->cloud->size() < 5) return src;
  const SgTree tree(src->cloud, small_gicp::KdTreeBuilderOMP(cfg_.num_threads));
  small_gicp::estimate_normals_covariances_omp(*src->cloud, tree, cfg_.source_num_neighbors, cfg_.num_threads);
  for (std::size_t i = 0; i < src->cloud->size(); ++i) {
    const Eigen::Vector4d& nrm = src->cloud->normals[i];
    if (nrm.head<3>().squaredNorm() > 0.5 && std::abs(nrm.z()) < kHorizontalNormalZ) src->structure.push_back(i);
  }
  src->cloud->normals.clear();
  return src;
}

double GicpMatcher::overlap(const SourceCloud& source, const MatchTarget& target, const Eigen::Isometry3d& T,
                            double distance, int num_threads, bool structure_only) const {
  const int threads = num_threads > 0 ? num_threads : cfg_.num_threads;
  const GicpSource& src = asSource(source);
  const GicpTarget& tgt = asTarget(target);
  if (!tgt.tree || src.size() == 0) return 0.0;
  const bool use_structure = structure_only && src.structure.size() >= kMinStructurePoints;
  const std::int64_t n = static_cast<std::int64_t>(use_structure ? src.structure.size() : src.size());
  const double d2max = distance * distance;
  std::int64_t hits = 0;
#pragma omp parallel for num_threads(threads) reduction(+ : hits)
  for (std::int64_t i = 0; i < n; ++i) {
    const std::size_t idx = use_structure ? src.structure[static_cast<std::size_t>(i)] : static_cast<std::size_t>(i);
    const Eigen::Vector4d q = T * src.cloud->points[idx];
    std::size_t k = 0;
    double d2 = 0.0;
    if (tgt.tree->nearest_neighbor_search(q, &k, &d2) && d2 <= d2max) ++hits;
  }
  return static_cast<double>(hits) / static_cast<double>(n);
}

RegistrationResult GicpMatcher::align(const SourceCloud& source, const MatchTarget& target,
                                      const Eigen::Isometry3d& init) const {
  const GicpSource& src = asSource(source);
  const GicpTarget& tgt = asTarget(target);
  RegistrationResult out;
  out.T_map_base = init;
  out.num_source = src.size();
  if (!tgt.tree || src.size() < 10 || tgt.num_points < 10) return out;

  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg;
  reg.reduction.num_threads = cfg_.num_threads;
  reg.rejector.max_dist_sq = cfg_.max_correspondence_distance * cfg_.max_correspondence_distance;
  reg.criteria.translation_eps = cfg_.translation_eps;
  reg.criteria.rotation_eps = cfg_.rotation_eps;
  reg.optimizer.max_iterations = cfg_.max_iterations;
  const bool vgicp = cfg_.registration == "vgicp";
  small_gicp::RegistrationResult r;
  if (vgicp) {
    // VGICP: スキャンの点を、入ったボクセル（地図の点の平均と共分散）に合わせる。近傍の探索は要らない。
    // ボクセルの平均との距離はボクセルの大きさ程度になりうるので、対応の距離の上限もそれに合わせる
    const SgVoxelMap& voxels = tgt.fineVoxels();
    reg.rejector.max_dist_sq =
        std::pow(std::max(cfg_.max_correspondence_distance, 2.0 * cfg_.vgicp_voxel_size), 2);
    r = reg.align(voxels, *src.cloud, voxels, init);
  } else {
    r = reg.align(*tgt.cloud, *src.cloud, *tgt.tree, init);
  }

  out.converged = r.converged;
  out.iterations = static_cast<int>(r.iterations);
  out.T_map_base = r.T_target_source;
  out.H = r.H;
  out.error = r.error;
  out.num_inliers = r.num_inliers;
  out.inlier_ratio = static_cast<double>(r.num_inliers) / static_cast<double>(src.size());
  if (vgicp) {
    // VGICP の「対応がある点」はボクセルに入った点なので、GICP の意味（max_correspondence_distance 以内に地図の点が
    // ある点）とは違う。採用の判定（min_inlier_ratio）をそろえるため、地図の点との距離で数え直す
    out.inlier_ratio = overlap(source, target, out.T_map_base, cfg_.max_correspondence_distance, 0, false);
  }
  out.error_per_point = r.num_inliers > 0 ? r.error / static_cast<double>(r.num_inliers) : 0.0;
  out.overlap = overlap(source, target, out.T_map_base, cfg_.overlap_distance);
  return out;
}

RegistrationResult GicpMatcher::alignCoarse(const SourceCloud& source, const MatchTarget& target,
                                            const Eigen::Isometry3d& init, int num_threads) const {
  const GicpSource& src = asSource(source);
  const GicpTarget& tgt = asTarget(target);
  RegistrationResult out;
  out.T_map_base = init;
  out.num_source = src.size();
  if (!tgt.tree || src.size() < 10) return out;

  const SgVoxelMap& voxels = tgt.voxels();
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg;
  reg.reduction.num_threads = num_threads > 0 ? num_threads : cfg_.num_threads;
  // ボクセルの平均との距離は、ボクセルの大きさ程度になりうる
  reg.rejector.max_dist_sq = std::pow(2.0 * reloc_.coarse_voxel_size, 2);
  reg.criteria.translation_eps = 1e-2;
  reg.criteria.rotation_eps = 1e-2;
  reg.optimizer.max_iterations = reloc_.coarse_max_iterations;
  const auto r = reg.align(voxels, *src.cloud, voxels, init);
  out.converged = r.converged;
  out.iterations = static_cast<int>(r.iterations);
  out.T_map_base = r.T_target_source;
  out.H = r.H;
  out.error = r.error;
  out.num_inliers = r.num_inliers;
  out.inlier_ratio = static_cast<double>(r.num_inliers) / static_cast<double>(src.size());
  out.error_per_point = r.num_inliers > 0 ? r.error / static_cast<double>(r.num_inliers) : 0.0;
  return out;
}

PoseSearchResult GicpMatcher::search(const std::vector<Vec3f>& points_base, const MatchTarget& target,
                                     const PoseSearchRequest& req) const {
  PoseSearchResult res;
  const auto fine = prepareSource(points_base, cfg_.source_voxel_size);
  const auto coarse = prepareSource(points_base, reloc_.coarse_source_voxel);
  if (static_cast<int>(fine->size()) < cfg_.min_source_points || coarse->size() < 10) {
    res.reason = "too few scan points";
    return res;
  }
  if (target.num_points < 10) {
    res.reason = "empty target";
    return res;
  }

  // --- 候補（位置の格子 × yaw）を並べて、粗い位置合わせ ---
  struct Candidate {
    Eigen::Isometry3d T;
    double score;
  };
  std::vector<Candidate> coarse_results;
  const double step = std::max(0.1, reloc_.position_step);
  const int nr = static_cast<int>(std::ceil(req.radius / step - 1e-9));
  std::vector<double> yaws;
  if (req.yaw_range >= kPi - 1e-6) {
    const int n = std::max(1, static_cast<int>(std::ceil(2.0 * kPi / reloc_.yaw_step - 1e-9)));
    for (int k = 0; k < n; ++k) yaws.push_back(-kPi + 2.0 * kPi * k / n);
  } else {
    const int n = static_cast<int>(std::ceil(req.yaw_range / reloc_.yaw_step - 1e-9));
    for (int k = -n; k <= n; ++k) yaws.push_back(n == 0 ? 0.0 : req.yaw_range * k / n);
  }
  std::vector<Eigen::Isometry3d> hypotheses;
  for (int ix = -nr; ix <= nr; ++ix) {
    for (int iy = -nr; iy <= nr; ++iy) {
      const Vec2 d(ix * step, iy * step);
      if (d.norm() > req.radius + 1e-9) continue;
      for (const double dyaw : yaws) {
        Eigen::Isometry3d T = req.center;
        T.linear() = Eigen::AngleAxisd(dyaw, Vec3::UnitZ()).toRotationMatrix() * req.center.linear();
        T.translation() += Vec3(d.x(), d.y(), 0.0);
        hypotheses.push_back(T);
      }
    }
  }
  res.num_hypotheses = static_cast<int>(hypotheses.size());
  asTarget(target).voxels();  // ボクセル地図は並列に入る前に作っておく
  // 候補ごとに並列に処理する（1 つの候補の点数は少ないので、候補の中では並列にしない）
  const double coarse_score_dist = reloc_.coarse_source_voxel;
  coarse_results.resize(hypotheses.size());
  const std::int64_t nh = static_cast<std::int64_t>(hypotheses.size());
#pragma omp parallel for num_threads(cfg_.num_threads) schedule(dynamic)
  for (std::int64_t i = 0; i < nh; ++i) {
    const RegistrationResult r = alignCoarse(*coarse, target, hypotheses[static_cast<std::size_t>(i)], 1);
    coarse_results[static_cast<std::size_t>(i)] = {r.T_map_base,
                                                   overlap(*coarse, target, r.T_map_base, coarse_score_dist, 1)};
  }
  std::sort(coarse_results.begin(), coarse_results.end(),
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

  const auto distinct = [this](const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return (a.translation().head<2>() - b.translation().head<2>()).norm() > reloc_.distinct_xy ||
           yawDiff(a, b) > reloc_.distinct_yaw;
  };

  // --- 上位の、互いに離れた候補だけを GICP で詰める ---
  std::vector<RegistrationResult> refined;
  std::vector<Eigen::Isometry3d> picked;
  for (const auto& c : coarse_results) {
    if (static_cast<int>(picked.size()) >= reloc_.refine_candidates) break;
    bool far = true;
    for (const auto& p : picked) far = far && distinct(c.T, p);
    if (!far) continue;
    picked.push_back(c.T);
    refined.push_back(align(*fine, target, c.T));
  }
  if (refined.empty()) {
    res.reason = "no candidate";
    return res;
  }
  std::sort(refined.begin(), refined.end(),
            [](const RegistrationResult& a, const RegistrationResult& b) { return a.overlap > b.overlap; });
  res.best = refined.front();
  res.best_overlap = res.best.overlap;
  for (std::size_t i = 1; i < refined.size(); ++i) {
    if (distinct(refined[i].T_map_base, res.best.T_map_base)) {
      res.second_overlap = refined[i].overlap;
      break;
    }
  }

  if (res.best.inlier_ratio < cfg_.min_inlier_ratio) {
    res.reason = "low inlier ratio";
  } else if (res.best_overlap < reloc_.min_overlap) {
    res.reason = "low overlap";
  } else if (res.second_overlap > 0.0 && res.best_overlap < reloc_.uniqueness_ratio * res.second_overlap) {
    res.reason = "ambiguous";
  } else {
    res.found = true;
  }
  return res;
}

}  // namespace gll
