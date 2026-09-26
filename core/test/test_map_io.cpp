// 地図の基盤（アンカー・PCD・タイル・maps.yaml・タイル化）の単体テスト。
#include "gll/map/map_anchor.hpp"
#include "gll/map/map_config.hpp"
#include "gll/map/map_tiler.hpp"
#include "gll/map/pcd_io.hpp"
#include "gll/map/tile.hpp"

#include "sim_world.hpp"

#include <GeographicLib/Geodesic.hpp>
#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace gll;
namespace fs = std::filesystem;

namespace {

fs::path tempDir(const std::string& name) {
  const fs::path d = fs::temp_directory_path() / ("gll_test_" + name + "_" + std::to_string(::getpid()));
  fs::remove_all(d);
  fs::create_directories(d);
  return d;
}

AnchorConfig tokyoAnchor(double heading_deg) {
  AnchorConfig a;
  a.map_point = Vec3(10.0, -5.0, 1.0);
  a.latitude = 35.681236;
  a.longitude = 139.767125;
  a.ellipsoid_height = 40.0;
  a.heading = deg2rad(heading_deg);
  return a;
}

}  // namespace

TEST(MapAnchor, MatchesGeodesicDisplacement) {
  // 地図の x 軸の方向（真北から時計回り 30°）に 100 m、y 軸の方向（x から反時計回りに 90°）に 50 m 進んだ点を、
  // 測地線の計算で求めた緯度経度 → UTM と比べる
  const UtmProjector utm(54, true);
  const AnchorConfig cfg = tokyoAnchor(30.0);
  const MapAnchor a = MapAnchor::fromConfig(cfg, utm);
  const auto& g = GeographicLib::Geodesic::WGS84();
  for (const auto& [dx, dy] : {std::pair{100.0, 0.0}, std::pair{0.0, 50.0}, std::pair{-80.0, 60.0}}) {
    const double dist = std::hypot(dx, dy);
    // 地図の (dx, dy) の方向の真方位: x 軸の方位 30° から、反時計回りの角度 atan2(dy, dx) を引く
    const double azi = 30.0 - rad2deg(std::atan2(dy, dx));
    double lat2, lon2;
    g.Direct(cfg.latitude, cfg.longitude, azi, dist, lat2, lon2);
    const UtmPoint expected = utm.forward(lat2, lon2);
    const Vec3 got = a.mapToUtm(cfg.map_point + Vec3(dx, dy, 2.0));
    EXPECT_NEAR(got.x(), expected.easting, 0.01) << dx << "," << dy;
    EXPECT_NEAR(got.y(), expected.northing, 0.01) << dx << "," << dy;
    EXPECT_NEAR(got.z(), cfg.ellipsoid_height + 2.0, 1e-9);
  }
}

TEST(MapAnchor, RoundTrips) {
  const UtmProjector utm(54, true);
  const MapAnchor a = MapAnchor::fromConfig(tokyoAnchor(-120.0), utm);
  const Vec3 p(123.4, -56.7, 3.2);
  EXPECT_LT((a.utmToMap(a.mapToUtm(p)) - p).norm(), 1e-6);

  const Eigen::Isometry3d T = makePose(Vec3(5, 6, 1), 0.02, -0.03, 1.2);
  const Eigen::Isometry3d back = a.poseUtmToMap(a.poseMapToUtm(T));
  EXPECT_LT((back.matrix() - T.matrix()).norm(), 1e-6);
  // yaw は φ だけ回り、roll / pitch は変わらない
  EXPECT_NEAR(wrapAngle(yawOf(a.poseMapToUtm(T).linear()) - 1.2 - a.rotation()), 0.0, 1e-9);

  const SE2 X = SE2::fromPose(20.0, -3.0, 0.7);
  const SE2 Xb = a.toMap(a.toUtm(X));
  EXPECT_NEAR(Xb.t().x(), 20.0, 1e-6);
  EXPECT_NEAR(Xb.t().y(), -3.0, 1e-6);
  EXPECT_NEAR(Xb.yaw(), 0.7, 1e-9);
}

TEST(MapAnchor, UtmDirectAndLocal) {
  const UtmProjector utm(54, true);
  AnchorConfig c;
  c.use_utm = true;
  c.easting = 386000.0;
  c.northing = 3950000.0;
  c.grid_heading = deg2rad(90.0);  // 地図の x 軸 = グリッドの東
  c.use_scale_factor = false;
  const MapAnchor a = MapAnchor::fromConfig(c, utm);
  EXPECT_NEAR(a.rotation(), 0.0, 1e-12);
  const Vec3 q = a.mapToUtm(Vec3(10, 20, 0));
  EXPECT_NEAR(q.x(), 386010.0, 1e-9);
  EXPECT_NEAR(q.y(), 3950020.0, 1e-9);

  const MapAnchor id = MapAnchor::identity();
  EXPECT_LT((id.mapToUtm(Vec3(1, 2, 3)) - Vec3(1, 2, 3)).norm(), 1e-12);
}

TEST(Pcd, ReadWriteAllFormats) {
  const fs::path d = tempDir("pcd");
  std::vector<Vec3f> pts;
  for (int i = 0; i < 1000; ++i) pts.emplace_back(0.1f * i, -0.2f * i, 1.5f + 0.01f * i);
  for (auto fmt : {PcdFormat::ASCII, PcdFormat::BINARY, PcdFormat::BINARY_COMPRESSED}) {
    const std::string path = (d / "a.pcd").string();
    writePcd(path, pts, fmt);
    const auto back = readPcd(path);
    ASSERT_EQ(back.size(), pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) EXPECT_LT((back[i] - pts[i]).norm(), 1e-4f);
  }
  fs::remove_all(d);
}

TEST(Pcd, ExtraFieldsDoublesAndNan) {
  // intensity を含み、x / y / z が F8 の binary。NaN の点は捨てる
  const fs::path d = tempDir("pcd2");
  const std::string path = (d / "b.pcd").string();
  {
    std::ofstream os(path, std::ios::binary);
    os << "VERSION .7\nFIELDS intensity x y z\nSIZE 4 8 8 8\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\n"
          "POINTS 3\nDATA binary\n";
    const auto rec = [&os](float in, double x, double y, double z) {
      os.write(reinterpret_cast<const char*>(&in), 4);
      os.write(reinterpret_cast<const char*>(&x), 8);
      os.write(reinterpret_cast<const char*>(&y), 8);
      os.write(reinterpret_cast<const char*>(&z), 8);
    };
    rec(1.f, 1.0, 2.0, 3.0);
    rec(2.f, std::nan(""), 0.0, 0.0);
    rec(3.f, -4.0, 5.5, 6.25);
  }
  const auto pts = readPcd(path);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_FLOAT_EQ(pts[1].x(), -4.0f);
  EXPECT_FLOAT_EQ(pts[1].z(), 6.25f);
  fs::remove_all(d);
}

TEST(Pcd, LzfBackReference) {
  // "abc" のリテラルの後に、3 バイト前から 6 バイトを写す参照 → "abcabcabc"
  const char in[] = {2, 'a', 'b', 'c', static_cast<char>(0x80), 2};
  const auto out = lzfDecompress(in, sizeof(in), 9);
  EXPECT_EQ(std::string(out.begin(), out.end()), "abcabcabc");
  EXPECT_THROW(lzfDecompress(in, sizeof(in), 10), std::runtime_error);
}

TEST(Tiles, FileAndIndexRoundTrip) {
  const fs::path d = tempDir("tiles");
  TileData t;
  t.id = TileId{"g", -3, 7};
  t.points = {Vec3f(1, 2, 3), Vec3f(-1, 0.5f, 2)};
  t.covs = {PackedCov{1, 0, 0, 1, 0, 0.001f}, PackedCov{0.5f, 0.1f, 0, 1, 0, 0.002f}};
  fs::create_directories(d / "tiles");
  writeTileFile((d / "tiles" / "-3_7.bin").string(), t);
  TileIndex idx;
  idx.tile_size = 20.0;
  TileMeta m;
  m.id = t.id;
  m.num_points = 2;
  m.bounds_min = Vec3(-1, 0.5, 2);
  m.bounds_max = Vec3(1, 2, 3);
  idx.tiles.push_back(m);
  idx.save((d / "tile_index.yaml").string());

  const TileIndex back = TileIndex::load((d / "tile_index.yaml").string(), "g");
  ASSERT_EQ(back.tiles.size(), 1u);
  EXPECT_EQ(back.tiles[0].id, t.id);
  EXPECT_NEAR(back.tiles[0].bounds_max.y(), 2.0, 1e-9);
  const auto loaded = BinaryTileLoader().load(back.tiles[0]);
  ASSERT_EQ(loaded->points.size(), 2u);
  EXPECT_EQ(loaded->id, t.id);
  EXPECT_FLOAT_EQ(loaded->covs[1][1], 0.1f);
  EXPECT_FLOAT_EQ(loaded->points[1].y(), 0.5f);
  // 別のグループの同じ番号のタイルは別物
  EXPECT_NE(t.id, (TileId{"h", -3, 7}));
  fs::remove_all(d);
}

TEST(MapConfig, LoadsGroupsWithAnchors) {
  const fs::path d = tempDir("mapcfg");
  for (const char* g : {"a", "b", "c"}) {
    fs::create_directories(d / g);
    TileIndex idx;
    idx.save((d / g / "tile_index.yaml").string());
  }
  {
    std::ofstream os(d / "maps.yaml");
    os << "utm: {zone: 54, hemisphere: north}\n"
          "map_groups:\n"
          "  - id: a\n"
          "    tile_index: a/tile_index.yaml\n"
          "    anchor: {map_point: [0, 0, 0], latitude: 35.68, longitude: 139.77, ellipsoid_height: 40, "
          "heading_deg: 90, stddev_xy: 0.03, stddev_yaw_deg: 0.1}\n"
          "  - id: b\n"
          "    tile_index: b/tile_index.yaml\n"
          "    anchor: {easting: 386000, northing: 3950000, grid_heading_deg: 0, use_scale_factor: false}\n"
          "  - id: c\n"
          "    tile_index: c/tile_index.yaml\n"
          "    anchor: local\n";
  }
  const MapSetConfig cfg = loadMapSetConfig((d / "maps.yaml").string());
  ASSERT_EQ(cfg.groups.size(), 3u);
  EXPECT_EQ(*cfg.utm_zone, 54);
  EXPECT_TRUE(*cfg.utm_north);
  EXPECT_NEAR(cfg.groups[0].anchor.heading, deg2rad(90.0), 1e-12);
  EXPECT_NEAR(cfg.groups[0].anchor.stddev_xy, 0.03, 1e-12);
  EXPECT_TRUE(cfg.groups[1].anchor.use_utm);
  EXPECT_TRUE(cfg.groups[2].local_anchor);
  EXPECT_EQ(cfg.groups[1].tile_index, (d / "b" / "tile_index.yaml").lexically_normal().string());

  const auto groups = loadMapGroups(cfg, UtmProjector(54, true));
  ASSERT_EQ(groups.size(), 3u);
  EXPECT_NEAR(groups[1].anchor.rotation(), kPi / 2.0, 1e-12);  // グリッド北 = UTM の +y
  fs::remove_all(d);
}

TEST(MapConfig, RejectsDuplicateIds) {
  const fs::path d = tempDir("mapcfg2");
  {
    std::ofstream os(d / "maps.yaml");
    os << "map_groups:\n  - {id: a, tile_index: x.yaml, anchor: local}\n  - {id: a, tile_index: y.yaml, anchor: local}\n";
  }
  EXPECT_THROW(loadMapSetConfig((d / "maps.yaml").string()), std::runtime_error);
  fs::remove_all(d);
}

TEST(MapTiler, TilesWithPlaneCovariances) {
  const sim::World w = sim::campusWorld();
  const auto pts = w.samplePoints(0.25, 0.005);
  TilerOptions opt;
  opt.tile_size = 20.0;
  opt.voxel_size = 0.3;
  opt.num_threads = 2;
  const TilerResult r = tileMap(pts, opt, "campus");
  ASSERT_GT(r.tiles.size(), 50u);
  std::size_t total = 0;
  for (std::size_t i = 0; i < r.tiles.size(); ++i) {
    const auto& t = r.tiles[i];
    total += t.points.size();
    ASSERT_EQ(t.covs.size(), t.points.size());
    EXPECT_EQ(t.id.group, "campus");
    EXPECT_EQ(r.index.tiles[i].id, t.id);
    for (const auto& p : t.points) {
      EXPECT_EQ(tileCoord(p.x(), 20.0), t.id.ix);
      EXPECT_EQ(tileCoord(p.y(), 20.0), t.id.iy);
    }
  }
  EXPECT_EQ(total, r.output_points);
  EXPECT_LT(r.output_points, pts.size());
  // 地面の点の共分散は、法線（z）方向だけが小さい平面の形になる（固有値 1e-3, 1, 1）
  const auto& t0 = r.tiles.front();
  const PackedCov& c = t0.covs.front();
  Mat3 C;
  C << c[0], c[1], c[2], c[1], c[3], c[4], c[2], c[4], c[5];
  const Eigen::SelfAdjointEigenSolver<Mat3> es(C);
  EXPECT_NEAR(es.eigenvalues()(0), 1e-3, 1e-4);
  EXPECT_NEAR(es.eigenvalues()(2), 1.0, 1e-3);

  const fs::path d = tempDir("tiler");
  writeTiles(d.string(), r);
  const TileIndex idx = TileIndex::load((d / "tile_index.yaml").string(), "campus");
  ASSERT_EQ(idx.tiles.size(), r.tiles.size());
  const auto back = BinaryTileLoader().load(idx.tiles[3]);
  EXPECT_EQ(back->points.size(), r.tiles[3].points.size());
  fs::remove_all(d);
}
