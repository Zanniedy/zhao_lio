#include "zhao_lio_backend/pose_graph.h"

#include <algorithm>
#include <cmath>
#include <set>

#include <Eigen/SparseCholesky>

namespace zhao_lio_backend {

namespace {

double robust_cost(double s2, double k) {
  if (k <= 0 || s2 <= k * k) return 0.5 * s2;
  return k * std::sqrt(s2) - 0.5 * k * k;
}

double robust_weight(double s2, double k) {
  if (k <= 0 || s2 <= k * k) return 1.0;
  return k / std::sqrt(s2);
}

void add_block(std::vector<Eigen::Triplet<double>>& t, int r0, int c0, const Eigen::MatrixXd& B) {
  for (int r = 0; r < B.rows(); ++r)
    for (int c = 0; c < B.cols(); ++c)
      if (B(r, c) != 0.0) t.emplace_back(r0 + r, c0 + c, B(r, c));
}

}  // namespace

void PoseGraph::clear() {
  nodes_.clear();
  factors_.clear();
  priors_.clear();
}

void PoseGraph::add_node(int id, const Pose& init) { nodes_[id] = init; }

std::vector<int> PoseGraph::ids() const {
  std::vector<int> v;
  v.reserve(nodes_.size());
  for (const auto& kv : nodes_) v.push_back(kv.first);
  return v;
}

void PoseGraph::add_prior(int id, const Pose& z, const SqrtInfo& L, double huber) {
  factors_.push_back({id, -1, z, L, huber});
}

void PoseGraph::add_between(int i, int j, const Pose& z, const SqrtInfo& L, double huber) {
  factors_.push_back({i, j, z, L, huber});
}

void PoseGraph::add_gravity(int id, const Eigen::Vector3d& up_body, double sigma) {
  Factor f;
  f.i = id;
  f.z = Pose::Identity();
  f.z.translation() = up_body.normalized();
  f.L = SqrtInfo::Zero(1, 6);
  f.L(0, 0) = 1.0 / sigma;
  f.gravity = true;
  factors_.push_back(f);
}

PoseGraph::Linearized PoseGraph::linearize(const Factor& f) const {
  Linearized out;
  const Pose& Ti = nodes_.at(f.i);
  if (f.gravity) {
    // r = R' e_z - up_body; with R -> R Exp(phi): dr/dphi = [R' e_z]x.
    const Eigen::Vector3d up = Ti.linear().transpose() * Eigen::Vector3d::UnitZ();
    const double w = f.L(0, 0);
    out.e = w * (up - f.z.translation());
    out.Ji = Eigen::MatrixXd::Zero(3, 6);
    out.Ji.rightCols<3>() = w * skew(up);
    return out;
  }
  if (f.j < 0) {
    const Vec6 r = se3_log(f.z.inverse() * Ti);
    out.e = f.L * r;
    out.Ji = f.L * se3_right_jacobian_inv(r);
    return out;
  }
  const Pose& Tj = nodes_.at(f.j);
  const Vec6 r = se3_log(f.z.inverse() * Ti.inverse() * Tj);
  const Mat6 Jr_inv = se3_right_jacobian_inv(r);
  out.e = f.L * r;
  out.Jj = f.L * Jr_inv;
  out.Ji = -f.L * Jr_inv * se3_adjoint(Tj.inverse() * Ti);
  return out;
}

double PoseGraph::factor_cost(const Factor& f, double* weight) const {
  const Pose& Ti = nodes_.at(f.i);
  if (f.gravity) {
    const double s2 = std::pow(f.L(0, 0), 2) *
                      (Ti.linear().transpose() * Eigen::Vector3d::UnitZ() - f.z.translation()).squaredNorm();
    if (weight) *weight = 1.0;
    return 0.5 * s2;
  }
  const Vec6 r = f.j < 0 ? se3_log(f.z.inverse() * Ti) : se3_log(f.z.inverse() * Ti.inverse() * nodes_.at(f.j));
  const double s2 = (f.L * r).squaredNorm();
  if (weight) *weight = robust_weight(s2, f.huber);
  return robust_cost(s2, f.huber);
}

double PoseGraph::prior_cost(const LinearPrior& p, Eigen::VectorXd* delta) const {
  Eigen::VectorXd d(6 * p.ids.size());
  for (std::size_t k = 0; k < p.ids.size(); ++k) d.segment<6>(6 * k) = se3_log(p.lin[k].inverse() * nodes_.at(p.ids[k]));
  if (delta) *delta = d;
  return p.g.dot(d) + 0.5 * d.dot(p.H * d);
}

double PoseGraph::total_cost() const {
  double c = 0.0;
  for (const auto& f : factors_) c += factor_cost(f, nullptr);
  for (const auto& p : priors_) c += prior_cost(p, nullptr);
  return c;
}

std::map<int, int> PoseGraph::make_index() const {
  std::map<int, int> index;
  int k = 0;
  for (const auto& kv : nodes_) index[kv.first] = k++;
  return index;
}

void PoseGraph::build(const std::map<int, int>& index, Eigen::SparseMatrix<double>* H, Eigen::VectorXd* g) const {
  const int n = 6 * static_cast<int>(index.size());
  std::vector<Eigen::Triplet<double>> trip;
  trip.reserve(factors_.size() * 144);
  g->setZero(n);
  for (const auto& f : factors_) {
    double w = 1.0;
    factor_cost(f, &w);
    const Linearized lin = linearize(f);
    const int bi = 6 * index.at(f.i);
    add_block(trip, bi, bi, w * lin.Ji.transpose() * lin.Ji);
    g->segment<6>(bi) += w * lin.Ji.transpose() * lin.e;
    if (f.j >= 0) {
      const int bj = 6 * index.at(f.j);
      add_block(trip, bj, bj, w * lin.Jj.transpose() * lin.Jj);
      add_block(trip, bi, bj, w * lin.Ji.transpose() * lin.Jj);
      add_block(trip, bj, bi, w * lin.Jj.transpose() * lin.Ji);
      g->segment<6>(bj) += w * lin.Jj.transpose() * lin.e;
    }
  }
  for (const auto& p : priors_) {
    Eigen::VectorXd d;
    prior_cost(p, &d);
    const Eigen::VectorXd gp = p.g + p.H * d;
    for (std::size_t a = 0; a < p.ids.size(); ++a) {
      const int ba = 6 * index.at(p.ids[a]);
      g->segment<6>(ba) += gp.segment<6>(6 * a);
      for (std::size_t b = 0; b < p.ids.size(); ++b) {
        add_block(trip, ba, 6 * index.at(p.ids[b]), p.H.block<6, 6>(6 * a, 6 * b));
      }
    }
  }
  H->resize(n, n);
  H->setFromTriplets(trip.begin(), trip.end());
}

double PoseGraph::optimize(int max_iters) {
  if (nodes_.empty()) return 0.0;
  const std::map<int, int> index = make_index();
  double cost = total_cost();
  double lambda = 1e-6;
  for (int it = 0; it < max_iters; ++it) {
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd g;
    build(index, &H, &g);
    bool improved = false;
    for (int tries = 0; tries < 8 && !improved; ++tries) {
      Eigen::SparseMatrix<double> A = H;
      for (int k = 0; k < A.rows(); ++k) A.coeffRef(k, k) += lambda * (1.0 + A.coeff(k, k));
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(A);
      if (solver.info() != Eigen::Success) { lambda *= 10; continue; }
      const Eigen::VectorXd dx = solver.solve(-g);
      const std::map<int, Pose> backup = nodes_;
      for (const auto& kv : index) nodes_[kv.first] = normalized(nodes_[kv.first] * se3_exp(dx.segment<6>(6 * kv.second)));
      const double new_cost = total_cost();
      if (new_cost <= cost) {
        improved = true;
        const double rel = (cost - new_cost) / std::max(cost, 1e-12);
        cost = new_cost;
        lambda = std::max(lambda / 10, 1e-9);
        if (dx.lpNorm<Eigen::Infinity>() < 1e-6 || rel < 1e-9) return cost;
      } else {
        nodes_ = backup;
        lambda *= 10;
      }
    }
    if (!improved) break;
  }
  return cost;
}

bool PoseGraph::marginal(const std::vector<int>& ids, Eigen::MatrixXd& cov) const {
  if (nodes_.empty()) return false;
  const std::map<int, int> index = make_index();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd g;
  build(index, &H, &g);
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(H);
  if (solver.info() != Eigen::Success) return false;
  const int n = static_cast<int>(H.rows());
  const int m = 6 * static_cast<int>(ids.size());
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(n, m);
  for (std::size_t a = 0; a < ids.size(); ++a) {
    const auto it = index.find(ids[a]);
    if (it == index.end()) return false;
    E.block<6, 6>(6 * it->second, 6 * a).setIdentity();
  }
  const Eigen::MatrixXd X = solver.solve(E);
  if (solver.info() != Eigen::Success) return false;
  cov = E.transpose() * X;
  cov = 0.5 * (cov + cov.transpose()).eval();
  return cov.allFinite();
}

Innovation PoseGraph::innovation(const std::vector<int>& ids, const Eigen::VectorXd& e, const Eigen::MatrixXd& J) const {
  Innovation out;
  Eigen::MatrixXd P;
  if (!marginal(ids, P)) return out;
  const Eigen::MatrixXd S = J * P * J.transpose() + Eigen::MatrixXd::Identity(e.size(), e.size());
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  if (ldlt.info() != Eigen::Success) return out;
  out.nis = e.dot(ldlt.solve(e));
  out.dof = static_cast<int>(e.size());
  out.valid = std::isfinite(out.nis) && out.dof > 0;
  return out;
}

Innovation PoseGraph::innovation_prior(int id, const Pose& z, const SqrtInfo& L) const {
  if (!has_node(id) || L.rows() == 0) return {};
  const Factor f{id, -1, z, L, 0.0};
  const Linearized lin = linearize(f);
  return innovation({id}, lin.e, lin.Ji);
}

Innovation PoseGraph::innovation_between(int i, int j, const Pose& z, const SqrtInfo& L) const {
  if (!has_node(i) || !has_node(j) || L.rows() == 0) return {};
  const Factor f{i, j, z, L, 0.0};
  const Linearized lin = linearize(f);
  Eigen::MatrixXd J(lin.e.size(), 12);
  J << lin.Ji, lin.Jj;
  return innovation({i, j}, lin.e, J);
}

void PoseGraph::marginalize_oldest() {
  if (nodes_.size() < 2) return;
  const int old = nodes_.begin()->first;

  // Variables touched by anything that involves `old`; `old` goes first.
  std::set<int> touched;
  for (const auto& f : factors_) {
    if (f.i == old && f.j >= 0) touched.insert(f.j);
    if (f.j == old) touched.insert(f.i);
  }
  for (const auto& p : priors_) {
    if (std::find(p.ids.begin(), p.ids.end(), old) == p.ids.end()) continue;
    for (int id : p.ids) if (id != old) touched.insert(id);
  }
  std::vector<int> keep(touched.begin(), touched.end());
  std::map<int, int> local;
  local[old] = 0;
  for (std::size_t k = 0; k < keep.size(); ++k) local[keep[k]] = static_cast<int>(k + 1);
  const int n = 6 * static_cast<int>(local.size());
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n);

  std::vector<Factor> rest;
  for (const auto& f : factors_) {
    if (f.i != old && f.j != old) { rest.push_back(f); continue; }
    double w = 1.0;
    factor_cost(f, &w);
    const Linearized lin = linearize(f);
    const int bi = 6 * local.at(f.i);
    H.block<6, 6>(bi, bi) += w * lin.Ji.transpose() * lin.Ji;
    g.segment<6>(bi) += w * lin.Ji.transpose() * lin.e;
    if (f.j >= 0) {
      const int bj = 6 * local.at(f.j);
      H.block<6, 6>(bj, bj) += w * lin.Jj.transpose() * lin.Jj;
      H.block<6, 6>(bi, bj) += w * lin.Ji.transpose() * lin.Jj;
      H.block<6, 6>(bj, bi) += w * lin.Jj.transpose() * lin.Ji;
      g.segment<6>(bj) += w * lin.Jj.transpose() * lin.e;
    }
  }
  std::vector<LinearPrior> rest_priors;
  for (const auto& p : priors_) {
    if (std::find(p.ids.begin(), p.ids.end(), old) == p.ids.end()) { rest_priors.push_back(p); continue; }
    // Re-linearize the old prior at the current estimate (first-order: Jacobian of delta ≈ I).
    Eigen::VectorXd d;
    prior_cost(p, &d);
    const Eigen::VectorXd gp = p.g + p.H * d;
    for (std::size_t a = 0; a < p.ids.size(); ++a) {
      const int ba = 6 * local.at(p.ids[a]);
      g.segment<6>(ba) += gp.segment<6>(6 * a);
      for (std::size_t b = 0; b < p.ids.size(); ++b) H.block<6, 6>(ba, 6 * local.at(p.ids[b])) += p.H.block<6, 6>(6 * a, 6 * b);
    }
  }
  factors_ = std::move(rest);
  priors_ = std::move(rest_priors);
  nodes_.erase(old);
  if (keep.empty()) return;

  // Schur complement eliminates `old` (block 0).
  const int m = n - 6;
  const Mat6 Hoo = H.block<6, 6>(0, 0) + 1e-9 * Mat6::Identity();
  const Eigen::MatrixXd Hko = H.block(6, 0, m, 6);
  const Eigen::LDLT<Mat6> ldlt(Hoo);
  LinearPrior lp;
  lp.ids = keep;
  for (int id : keep) lp.lin.push_back(nodes_.at(id));
  lp.H = H.block(6, 6, m, m) - Hko * ldlt.solve(Hko.transpose());
  lp.H = 0.5 * (lp.H + lp.H.transpose()).eval();
  lp.g = g.segment(6, m) - Hko * ldlt.solve(g.head<6>());
  priors_.push_back(std::move(lp));
}

}  // namespace zhao_lio_backend
