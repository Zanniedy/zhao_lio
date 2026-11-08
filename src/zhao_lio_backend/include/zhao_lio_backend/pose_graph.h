#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "zhao_lio_backend/se3.h"

namespace zhao_lio_backend {

// Factor information is given as a square-root information matrix L (r x 6, r <= 6), so a factor
// may constrain only some directions: whitened residual e = L * Log(...).
using SqrtInfo = Eigen::Matrix<double, Eigen::Dynamic, 6>;

inline SqrtInfo sqrt_info_from_sigmas(const Vec6& sigmas) {
  SqrtInfo L = SqrtInfo::Zero(6, 6);
  for (int k = 0; k < 6; ++k) L(k, k) = 1.0 / sigmas[k];
  return L;
}

struct Innovation {
  bool valid = false;
  double nis = 0.0;
  int dof = 0;
};

// SE(3) pose graph solved with Levenberg-Marquardt on a sparse LDLT. Nodes are keyed by id; the
// oldest node can be marginalized into a linear prior to bound the problem size.
class PoseGraph {
 public:
  void clear();
  void add_node(int id, const Pose& init);
  bool has_node(int id) const { return nodes_.count(id) > 0; }
  const Pose& pose(int id) const { return nodes_.at(id); }
  void set_pose(int id, const Pose& T) { nodes_.at(id) = T; }
  std::size_t size() const { return nodes_.size(); }
  std::vector<int> ids() const;

  // huber <= 0 disables the robust kernel; otherwise it is the threshold on |e|.
  void add_prior(int id, const Pose& z, const SqrtInfo& L, double huber = 0.0);
  void add_between(int i, int j, const Pose& z, const SqrtInfo& L, double huber = 0.0);
  // Gravity direction: world up (+z) expressed in the body frame should equal `up_body`.
  // Constrains roll and pitch only; sigma in rad.
  void add_gravity(int id, const Eigen::Vector3d& up_body, double sigma);

  // Returns the final cost; iterations stop on convergence or after max_iters.
  double optimize(int max_iters = 10);

  // Marginal covariance of the stacked tangents of `ids` (6k x 6k) at the current estimate.
  bool marginal(const std::vector<int>& ids, Eigen::MatrixXd& cov) const;

  // Normalized innovation squared of a factor that is not (yet) in the graph.
  Innovation innovation_prior(int id, const Pose& z, const SqrtInfo& L) const;
  Innovation innovation_between(int i, int j, const Pose& z, const SqrtInfo& L) const;

  // Removes the oldest node; its factors become a linear prior on the nodes it was connected to.
  void marginalize_oldest();

  std::size_t num_factors() const { return factors_.size(); }

 private:
  struct Factor {
    int i = -1, j = -1;  // j < 0: unary prior on i
    Pose z;
    SqrtInfo L;
    double huber = 0.0;
    bool gravity = false;  // unary gravity factor: z.translation() = up_body, L = 1/sigma (1 x 1)
  };
  // cost(delta) = g' delta + 0.5 delta' H delta, delta_k = Log(lin_k^-1 T_k)
  struct LinearPrior {
    std::vector<int> ids;
    std::vector<Pose> lin;
    Eigen::MatrixXd H;
    Eigen::VectorXd g;
  };

  struct Linearized {
    Eigen::VectorXd e;    // whitened residual
    Eigen::MatrixXd Ji, Jj;  // whitened Jacobians (r x 6)
  };

  Linearized linearize(const Factor& f) const;
  double factor_cost(const Factor& f, double* weight) const;
  double prior_cost(const LinearPrior& p, Eigen::VectorXd* delta) const;
  double total_cost() const;
  // Builds the Gauss-Newton system; `index` maps node id to block index.
  void build(const std::map<int, int>& index, Eigen::SparseMatrix<double>* H, Eigen::VectorXd* g) const;
  std::map<int, int> make_index() const;
  Innovation innovation(const std::vector<int>& ids, const Eigen::VectorXd& e, const Eigen::MatrixXd& J) const;

  std::map<int, Pose> nodes_;
  std::vector<Factor> factors_;
  std::vector<LinearPrior> priors_;
};

}  // namespace zhao_lio_backend
