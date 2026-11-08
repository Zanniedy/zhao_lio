#include "zhao_lio_backend/map_io.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace zhao_lio_backend {

bool read_pcd(const std::string& path, std::vector<Eigen::Vector3f>& out, std::string* error) {
  auto fail = [&](const std::string& m) { if (error) *error = path + ": " + m; return false; };
  std::ifstream f(path, std::ios::binary);
  if (!f) return fail("cannot open");
  std::vector<std::string> fields;
  std::vector<int> sizes, counts;
  std::vector<char> types;
  std::size_t points = 0;
  std::string data, line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "FIELDS") { std::string s; while (ss >> s) fields.push_back(s); }
    else if (key == "SIZE") { int v; while (ss >> v) sizes.push_back(v); }
    else if (key == "TYPE") { char c; while (ss >> c) types.push_back(c); }
    else if (key == "COUNT") { int v; while (ss >> v) counts.push_back(v); }
    else if (key == "POINTS") ss >> points;
    else if (key == "DATA") { ss >> data; break; }
  }
  if (counts.empty()) counts.assign(fields.size(), 1);
  if (fields.empty() || sizes.size() != fields.size() || types.size() != fields.size() || counts.size() != fields.size())
    return fail("malformed header");
  int ix = -1, iy = -1, iz = -1;
  std::vector<int> offset(fields.size());
  int stride = 0, col = 0;
  std::vector<int> column(fields.size());
  for (std::size_t k = 0; k < fields.size(); ++k) {
    offset[k] = stride;
    column[k] = col;
    stride += sizes[k] * counts[k];
    col += counts[k];
    if (fields[k] == "x") ix = static_cast<int>(k);
    if (fields[k] == "y") iy = static_cast<int>(k);
    if (fields[k] == "z") iz = static_cast<int>(k);
  }
  if (ix < 0 || iy < 0 || iz < 0) return fail("no x/y/z fields");
  for (int k : {ix, iy, iz})
    if (types[k] != 'F' || (sizes[k] != 4 && sizes[k] != 8)) return fail("x/y/z must be float32 or float64");
  out.clear();
  out.reserve(points);
  if (data == "ascii") {
    std::vector<double> vals(col);
    for (std::size_t n = 0; n < points && std::getline(f, line); ++n) {
      std::istringstream ss(line);
      for (int c = 0; c < col; ++c) ss >> vals[c];
      if (!ss) continue;
      const Eigen::Vector3f p(vals[column[ix]], vals[column[iy]], vals[column[iz]]);
      if (p.allFinite()) out.push_back(p);
    }
  } else if (data == "binary") {
    std::vector<char> buf(static_cast<std::size_t>(stride) * points);
    f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    if (static_cast<std::size_t>(f.gcount()) != buf.size()) return fail("truncated binary data");
    auto get = [&](const char* rec, int k) {
      if (sizes[k] == 4) { float v; std::memcpy(&v, rec + offset[k], 4); return static_cast<double>(v); }
      double v; std::memcpy(&v, rec + offset[k], 8); return v;
    };
    for (std::size_t n = 0; n < points; ++n) {
      const char* rec = buf.data() + n * stride;
      const Eigen::Vector3f p(get(rec, ix), get(rec, iy), get(rec, iz));
      if (p.allFinite()) out.push_back(p);
    }
  } else {
    return fail("unsupported DATA " + data);
  }
  return true;
}

std::vector<Eigen::Vector3f> voxel_filter(const std::vector<Eigen::Vector3f>& in, float leaf) {
  struct Acc { Eigen::Vector3f sum = Eigen::Vector3f::Zero(); int n = 0; };
  std::unordered_map<uint64_t, Acc> grid;
  grid.reserve(in.size());
  const float inv = 1.0f / leaf;
  constexpr int64_t kOff = int64_t(1) << 20;
  constexpr uint64_t kMask = (uint64_t(1) << 21) - 1;
  std::vector<uint64_t> order;
  for (const auto& p : in) {
    const int64_t x = static_cast<int64_t>(std::floor(p.x() * inv)) + kOff;
    const int64_t y = static_cast<int64_t>(std::floor(p.y() * inv)) + kOff;
    const int64_t z = static_cast<int64_t>(std::floor(p.z() * inv)) + kOff;
    const uint64_t key = ((static_cast<uint64_t>(x) & kMask) << 42) | ((static_cast<uint64_t>(y) & kMask) << 21) |
                         (static_cast<uint64_t>(z) & kMask);
    Acc& a = grid[key];
    if (a.n == 0) order.push_back(key);
    a.sum += p;
    ++a.n;
  }
  std::vector<Eigen::Vector3f> out;
  out.reserve(order.size());
  for (uint64_t k : order) { const Acc& a = grid[k]; out.push_back(a.sum / static_cast<float>(a.n)); }
  return out;
}

}  // namespace zhao_lio_backend
