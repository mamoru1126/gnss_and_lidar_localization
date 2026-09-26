#include "gll/common/geodesy.hpp"

#include "gll/common/types.hpp"

#include <GeographicLib/UTMUPS.hpp>

#include <stdexcept>

namespace gll {

UtmProjector::UtmProjector(int zone, bool north) : zone_(zone), north_(north) {
  if (zone < 1 || zone > 60) throw std::invalid_argument("UTM zone must be in [1, 60]");
}

UtmPoint UtmProjector::forward(double lat_deg, double lon_deg) const {
  int zone_out = 0;
  bool north_out = true;
  double x = 0.0, y = 0.0, gamma_deg = 0.0, k = 1.0;
  GeographicLib::UTMUPS::Forward(lat_deg, lon_deg, zone_out, north_out, x, y, gamma_deg, k, zone_);
  // 北半球/南半球の指定と実際の位置が違う場合は、northing を指定側の基準に合わせる。
  if (north_out != north_) y += north_out ? -10000000.0 : 10000000.0;
  UtmPoint p;
  p.easting = x;
  p.northing = y;
  p.convergence = deg2rad(gamma_deg);
  p.scale = k;
  return p;
}

LatLon UtmProjector::inverse(double easting, double northing) const {
  double lat = 0.0, lon = 0.0, gamma = 0.0, k = 1.0;
  GeographicLib::UTMUPS::Reverse(zone_, north_, easting, northing, lat, lon, gamma, k);
  return LatLon{lat, lon};
}

}  // namespace gll
