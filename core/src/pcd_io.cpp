#include "gll/map/pcd_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace gll {
namespace {

struct PcdField {
  std::string name;
  int size = 4;
  char type = 'F';
  int count = 1;
};

struct PcdHeader {
  std::vector<PcdField> fields;
  std::size_t points = 0;
  std::string data;
};

std::vector<std::string> split(const std::string& line) {
  std::istringstream is(line);
  std::vector<std::string> out;
  std::string tok;
  while (is >> tok) out.push_back(tok);
  return out;
}

PcdHeader parseHeader(std::istream& is, const std::string& path) {
  PcdHeader h;
  std::vector<int> sizes, counts;
  std::vector<char> types;
  std::size_t width = 0, height = 1;
  bool has_points = false;
  std::string line;
  while (std::getline(is, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    const auto tok = split(line);
    if (tok.empty()) continue;
    const std::string& key = tok[0];
    if (key == "FIELDS" || key == "COLUMNS") {
      for (std::size_t i = 1; i < tok.size(); ++i) h.fields.push_back(PcdField{tok[i]});
    } else if (key == "SIZE") {
      for (std::size_t i = 1; i < tok.size(); ++i) sizes.push_back(std::stoi(tok[i]));
    } else if (key == "TYPE") {
      for (std::size_t i = 1; i < tok.size(); ++i) types.push_back(tok[i][0]);
    } else if (key == "COUNT") {
      for (std::size_t i = 1; i < tok.size(); ++i) counts.push_back(std::stoi(tok[i]));
    } else if (key == "WIDTH") {
      width = std::stoul(tok.at(1));
    } else if (key == "HEIGHT") {
      height = std::stoul(tok.at(1));
    } else if (key == "POINTS") {
      h.points = std::stoul(tok.at(1));
      has_points = true;
    } else if (key == "DATA") {
      h.data = tok.at(1);
      break;
    }
  }
  if (h.data.empty()) throw std::runtime_error("PCD '" + path + "': DATA line is missing");
  if (h.fields.empty()) throw std::runtime_error("PCD '" + path + "': FIELDS is missing");
  if (sizes.size() != h.fields.size() || types.size() != h.fields.size())
    throw std::runtime_error("PCD '" + path + "': SIZE / TYPE do not match FIELDS");
  if (!counts.empty() && counts.size() != h.fields.size())
    throw std::runtime_error("PCD '" + path + "': COUNT does not match FIELDS");
  for (std::size_t i = 0; i < h.fields.size(); ++i) {
    h.fields[i].size = sizes[i];
    h.fields[i].type = types[i];
    h.fields[i].count = counts.empty() ? 1 : counts[i];
  }
  if (!has_points) h.points = width * height;
  return h;
}

double readValue(const char* p, const PcdField& f) {
  if (f.type == 'F' && f.size == 4) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
  }
  if (f.type == 'F' && f.size == 8) {
    double v;
    std::memcpy(&v, p, 8);
    return v;
  }
  throw std::runtime_error("PCD: x / y / z must be float32 or float64");
}

}  // namespace

std::vector<char> lzfDecompress(const char* in, std::size_t in_size, std::size_t out_size) {
  std::vector<char> out(out_size);
  const auto* ip = reinterpret_cast<const std::uint8_t*>(in);
  const auto* const in_end = ip + in_size;
  std::size_t op = 0;
  while (ip < in_end) {
    unsigned int ctrl = *ip++;
    if (ctrl < (1u << 5)) {  // リテラル: ctrl + 1 バイトをそのまま写す
      ++ctrl;
      if (op + ctrl > out_size || ip + ctrl > in_end) throw std::runtime_error("LZF: corrupted data (literal)");
      std::memcpy(out.data() + op, ip, ctrl);
      op += ctrl;
      ip += ctrl;
    } else {  // 後方参照
      unsigned int len = ctrl >> 5;
      if (ip >= in_end) throw std::runtime_error("LZF: corrupted data (reference)");
      if (len == 7) len += *ip++;
      if (ip >= in_end) throw std::runtime_error("LZF: corrupted data (reference)");
      const std::size_t back = ((ctrl & 0x1fu) << 8) + *ip++ + 1;
      len += 2;
      if (back > op || op + len > out_size) throw std::runtime_error("LZF: corrupted data (reference range)");
      for (unsigned int i = 0; i < len; ++i, ++op) out[op] = out[op - back];  // 重なりを許してバイト単位で写す
    }
  }
  if (op != out_size) throw std::runtime_error("LZF: size mismatch");
  return out;
}

std::vector<Vec3f> readPcd(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) throw std::runtime_error("cannot open PCD '" + path + "'");
  const PcdHeader h = parseHeader(is, path);

  int ix = -1, iy = -1, iz = -1;
  std::vector<std::size_t> byte_off(h.fields.size()), tok_off(h.fields.size());
  std::size_t stride = 0, ntok = 0;
  for (std::size_t i = 0; i < h.fields.size(); ++i) {
    byte_off[i] = stride;
    tok_off[i] = ntok;
    stride += static_cast<std::size_t>(h.fields[i].size) * h.fields[i].count;
    ntok += h.fields[i].count;
    if (h.fields[i].name == "x") ix = static_cast<int>(i);
    if (h.fields[i].name == "y") iy = static_cast<int>(i);
    if (h.fields[i].name == "z") iz = static_cast<int>(i);
  }
  if (ix < 0 || iy < 0 || iz < 0) throw std::runtime_error("PCD '" + path + "': fields x, y, z are required");

  std::vector<Vec3f> pts;
  pts.reserve(h.points);
  const auto push = [&pts](double x, double y, double z) {
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      pts.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  };

  if (h.data == "ascii") {
    std::string line;
    std::size_t n = 0;
    while (n < h.points && std::getline(is, line)) {
      const auto tok = split(line);
      if (tok.empty()) continue;
      if (tok.size() < ntok) throw std::runtime_error("PCD '" + path + "': short line in ascii data");
      push(std::strtod(tok[tok_off[ix]].c_str(), nullptr), std::strtod(tok[tok_off[iy]].c_str(), nullptr),
           std::strtod(tok[tok_off[iz]].c_str(), nullptr));
      ++n;
    }
    if (n != h.points) throw std::runtime_error("PCD '" + path + "': ascii data is truncated");
  } else if (h.data == "binary") {
    std::vector<char> buf(stride * h.points);
    is.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    if (!is) throw std::runtime_error("PCD '" + path + "': binary data is truncated");
    for (std::size_t k = 0; k < h.points; ++k) {
      const char* rec = buf.data() + k * stride;
      push(readValue(rec + byte_off[ix], h.fields[ix]), readValue(rec + byte_off[iy], h.fields[iy]),
           readValue(rec + byte_off[iz], h.fields[iz]));
    }
  } else if (h.data == "binary_compressed") {
    std::uint32_t csize = 0, usize = 0;
    is.read(reinterpret_cast<char*>(&csize), 4);
    is.read(reinterpret_cast<char*>(&usize), 4);
    if (!is) throw std::runtime_error("PCD '" + path + "': compressed header is truncated");
    if (usize != stride * h.points) throw std::runtime_error("PCD '" + path + "': unexpected uncompressed size");
    std::vector<char> cbuf(csize);
    is.read(cbuf.data(), csize);
    if (!is) throw std::runtime_error("PCD '" + path + "': compressed data is truncated");
    const std::vector<char> buf = lzfDecompress(cbuf.data(), csize, usize);
    // 展開後はフィールドごとに全点が並ぶ（SoA）
    std::vector<std::size_t> soa(h.fields.size());
    std::size_t acc = 0;
    for (std::size_t i = 0; i < h.fields.size(); ++i) {
      soa[i] = acc;
      acc += static_cast<std::size_t>(h.fields[i].size) * h.fields[i].count * h.points;
    }
    const auto at = [&](int f, std::size_t k) {
      const std::size_t w = static_cast<std::size_t>(h.fields[f].size) * h.fields[f].count;
      return readValue(buf.data() + soa[f] + k * w, h.fields[f]);
    };
    for (std::size_t k = 0; k < h.points; ++k) push(at(ix, k), at(iy, k), at(iz, k));
  } else {
    throw std::runtime_error("PCD '" + path + "': unsupported DATA " + h.data);
  }
  return pts;
}

void writePcd(const std::string& path, const std::vector<Vec3f>& points, PcdFormat format) {
  std::ofstream os(path, std::ios::binary);
  if (!os) throw std::runtime_error("cannot write PCD '" + path + "'");
  const char* data = format == PcdFormat::ASCII ? "ascii" : format == PcdFormat::BINARY ? "binary" : "binary_compressed";
  os << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH "
     << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA " << data << "\n";
  if (format == PcdFormat::ASCII) {
    os.precision(9);
    for (const auto& p : points) os << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
  } else if (format == PcdFormat::BINARY) {
    for (const auto& p : points) os.write(reinterpret_cast<const char*>(p.data()), 12);
  } else {
    std::vector<char> raw(points.size() * 12);
    for (int f = 0; f < 3; ++f)
      for (std::size_t k = 0; k < points.size(); ++k)
        std::memcpy(raw.data() + (f * points.size() + k) * 4, &points[k][f], 4);
    std::vector<char> comp;
    comp.reserve(raw.size() + raw.size() / 32 + 1);
    for (std::size_t i = 0; i < raw.size(); i += 32) {
      const std::size_t len = std::min<std::size_t>(32, raw.size() - i);
      comp.push_back(static_cast<char>(len - 1));
      comp.insert(comp.end(), raw.begin() + static_cast<std::ptrdiff_t>(i),
                  raw.begin() + static_cast<std::ptrdiff_t>(i + len));
    }
    const std::uint32_t csize = static_cast<std::uint32_t>(comp.size());
    const std::uint32_t usize = static_cast<std::uint32_t>(raw.size());
    os.write(reinterpret_cast<const char*>(&csize), 4);
    os.write(reinterpret_cast<const char*>(&usize), 4);
    os.write(comp.data(), static_cast<std::streamsize>(comp.size()));
  }
  if (!os) throw std::runtime_error("failed to write PCD '" + path + "'");
}

}  // namespace gll
