// 緯度経度 ⇔ UTM（ゾーン固定）。GeographicLib を使う。
#pragma once

namespace tiled_pcd_map {

struct UtmPoint {
  double easting = 0.0;
  double northing = 0.0;
  double convergence = 0.0;  ///< 子午線収差 γ [rad]（グリッド北の、真北からの時計回り角）
  double scale = 1.0;        ///< 点縮尺係数 k
};

struct LatLon {
  double lat = 0.0;  ///< [deg]
  double lon = 0.0;  ///< [deg]
};

class UtmProjector {
 public:
  /// ゾーンは固定する（ゾーン境界をまたいでも切り替えない。設計書 3.6 節）。
  UtmProjector(int zone, bool north);

  UtmPoint forward(double lat_deg, double lon_deg) const;
  LatLon inverse(double easting, double northing) const;

  int zone() const { return zone_; }
  bool north() const { return north_; }

 private:
  int zone_;
  bool north_;
};

}  // namespace tiled_pcd_map
