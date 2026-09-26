#include "gll/common/pose_store.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace gll {

bool savePose(const std::string& path, const SavedPose& p) {
  const std::string tmp = path + ".tmp";
  {
    std::ofstream os(tmp);
    if (!os) return false;
    os << std::setprecision(17) << "# gll saved pose: t x y yaw (map = UTM)\n"
       << p.t << ' ' << p.pose.x << ' ' << p.pose.y << ' ' << p.pose.yaw << '\n';
    if (!os) return false;
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

std::optional<SavedPose> loadPose(const std::string& path) {
  std::ifstream is(path);
  if (!is) return std::nullopt;
  std::string line;
  while (std::getline(is, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    SavedPose p;
    if (ls >> p.t >> p.pose.x >> p.pose.y >> p.pose.yaw && std::isfinite(p.pose.x) && std::isfinite(p.pose.y) &&
        std::isfinite(p.pose.yaw))
      return p;
    return std::nullopt;
  }
  return std::nullopt;
}

}  // namespace gll
