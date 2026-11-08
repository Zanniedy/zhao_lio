#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace zhao_lio_backend {

// Reads x/y/z of an ASCII or binary PCD (other fields are skipped). binary_compressed is not supported.
bool read_pcd(const std::string& path, std::vector<Eigen::Vector3f>& out, std::string* error);

// Centroid per voxel of side `leaf` (> 0).
std::vector<Eigen::Vector3f> voxel_filter(const std::vector<Eigen::Vector3f>& in, float leaf);

}  // namespace zhao_lio_backend
