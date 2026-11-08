#pragma once

#include <vector>

#include <Eigen/Core>

namespace zhao_lio_backend {

struct ScanContextParams {
  int rings = 20;
  int sectors = 60;
  double max_range = 40.0;     // m
  double height_offset = 2.0;  // m added to z so that occupied cells are > 0 (empty cells are 0)
  int candidates = 10;         // ring-key prefilter size
};

// Scan Context (Kim & Kim, IROS 2018): polar grid around the sensor, each cell keeps the maximum
// height. Points must be in a gravity-leveled frame centered on the sensor.
struct ScanContext {
  Eigen::MatrixXf desc;       // rings x sectors
  Eigen::VectorXf ring_key;   // mean of each ring

  static ScanContext make(const std::vector<Eigen::Vector3f>& leveled, const ScanContextParams& p);
};

struct ScMatch {
  int index = -1;      // into the database
  double distance = 1.0;
  double yaw = 0.0;    // rad; query frame = Rz(yaw) applied to candidate frame, see scan_context.cpp
};

// Column-shift cosine distance; returns the best shift in sectors.
double sc_distance(const ScanContext& a, const ScanContext& b, int* best_shift);

class ScanContextDb {
 public:
  explicit ScanContextDb(const ScanContextParams& p = ScanContextParams()) : p_(p) {}
  void add(const ScanContext& sc) { db_.push_back(sc); }
  std::size_t size() const { return db_.size(); }
  const ScanContext& at(std::size_t i) const { return db_[i]; }
  void clear() { db_.clear(); }
  // Best match among entries for which `allowed(i)` is true. Returns index -1 if none.
  template <class Pred>
  ScMatch query(const ScanContext& q, Pred allowed) const;
  // Top-k matches (ascending distance).
  template <class Pred>
  std::vector<ScMatch> query_k(const ScanContext& q, int k, Pred allowed) const;
  const ScanContextParams& params() const { return p_; }

 private:
  ScanContextParams p_;
  std::vector<ScanContext> db_;
};

}  // namespace zhao_lio_backend

#include "zhao_lio_backend/scan_context_impl.h"
