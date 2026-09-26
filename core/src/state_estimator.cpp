#include "gll/estimation/state_estimator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace gll {

UpdateResult correct(const IStateEstimator& est, FilterState& st, const Linearization& lin,
                     GatePolicy policy, const MahalanobisGate& gate) {
  UpdateResult res;
  const Eigen::MatrixXd& H = lin.H;
  const Eigen::MatrixXd S = H * st.P * H.transpose() + lin.R;
  const Eigen::MatrixXd S_inv = S.inverse();
  res.dof = static_cast<int>(lin.r.size());
  res.d2 = lin.r.dot(S_inv * lin.r);
  res.residual = lin.r;
  res.gate_passed = (policy == GatePolicy::SKIP) || gate.pass(res.d2, res.dof);
  if (!res.gate_passed && policy != GatePolicy::SKIP) return res;

  const Eigen::MatrixXd K = st.P * H.transpose() * S_inv;
  const Vec5 dx = K * lin.r;
  const Pose2D before = st.X.toPose();
  est.inject(st, dx);
  const Mat5 IKH = Mat5::Identity() - K * H;
  st.P = IKH * st.P * IKH.transpose() + K * lin.R * K.transpose();
  st.P = 0.5 * (st.P + st.P.transpose());
  const Pose2D after = st.X.toPose();
  res.world_delta = Vec3(after.x - before.x, after.y - before.y, wrapAngle(after.yaw - before.yaw));
  res.accepted = true;
  return res;
}

}  // namespace gll
