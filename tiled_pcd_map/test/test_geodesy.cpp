#include "tiled_pcd_map/geodesy.hpp"
#include "tiled_pcd_map/math.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace tiled_pcd_map;

TEST(Geodesy, CentralMeridian) {
  // ゾーン 54 の中央子午線は東経 141°。中央子午線上では E = 500 km、収差 0、縮尺 0.9996。
  const UtmProjector p(54, true);
  const UtmPoint u = p.forward(35.0, 141.0);
  EXPECT_NEAR(u.easting, 500000.0, 1e-6);
  EXPECT_NEAR(u.convergence, 0.0, 1e-12);
  EXPECT_NEAR(u.scale, 0.9996, 1e-9);
}

TEST(Geodesy, RoundTrip) {
  const UtmProjector p(54, true);
  for (double lat : {34.0, 35.68, 36.5}) {
    for (double lon : {139.2, 139.77, 142.1}) {
      const UtmPoint u = p.forward(lat, lon);
      const LatLon ll = p.inverse(u.easting, u.northing);
      EXPECT_NEAR(ll.lat, lat, 1e-9);
      EXPECT_NEAR(ll.lon, lon, 1e-9);
    }
  }
}

TEST(Geodesy, ConvergenceSignWestOfCentralMeridian) {
  // 北半球で中央子午線より西ではグリッド北が真北の反時計回り側になる（γ < 0）
  const UtmProjector p(54, true);
  EXPECT_LT(p.forward(35.68, 139.77).convergence, 0.0);
  EXPECT_GT(p.forward(35.68, 142.0).convergence, 0.0);
}

TEST(Geodesy, FixedZoneOutsideItsBand) {
  // ゾーン境界（東経 138°）の西側でも指定ゾーンで計算できる（ゾーンは切り替えない）
  const UtmProjector p(54, true);
  const UtmPoint u = p.forward(35.0, 137.5);
  const LatLon ll = p.inverse(u.easting, u.northing);
  EXPECT_NEAR(ll.lon, 137.5, 1e-8);
}
