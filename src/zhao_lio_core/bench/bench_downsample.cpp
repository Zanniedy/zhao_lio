#include <chrono>
#include <cstdio>
#include <random>

#include "zhao_lio/voxel_downsample.h"

using namespace zhao_lio;

int main(int argc, char** argv) {
  const int n = argc > 1 ? std::atoi(argv[1]) : 20000;  // Mid360 ~20k points per 0.1 s scan
  const float leaf = argc > 2 ? std::atof(argv[2]) : 0.5f;
  std::mt19937 rng(0);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f), r(0.5f, 40.0f);
  std::vector<LidarPoint> in(n);
  for (int i = 0; i < n; ++i) {
    in[i].xyz = Eigen::Vector3f(u(rng), u(rng), 0.3f * u(rng)).normalized() * r(rng);
    in[i].time = i * 5e-6;
  }
  VoxelDownsample ds(leaf);
  std::vector<LidarPoint> out;
  auto bench = [&](const char* name, auto&& fn) {
    for (int i = 0; i < 20; ++i) fn();  // warm-up
    const int iters = 200;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iters; ++i) fn();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / iters;
    std::printf("%-10s n=%d leaf=%.2f out=%zu  %.3f ms\n", name, n, leaf, out.size(), ms);
  };
  bench("serial", [&] { ds.filter_serial(in, out); });
  bench("parallel", [&] { ds.filter_parallel(in, out); });
  return 0;
}
