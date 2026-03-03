#include "sim2sim/observation_builder.h"

#include <algorithm>

namespace sim2sim {

ObservationBuilder::ObservationBuilder(std::size_t obs_dim, bool include_last_action)
    : obs_dim_(obs_dim), include_last_action_(include_last_action) {}

Eigen::VectorXd ObservationBuilder::build(const MujocoState& state, const Eigen::VectorXd& last_action) const {
  Eigen::VectorXd obs = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(obs_dim_));
  std::size_t offset = 0;

  const std::size_t q_copy = std::min(obs_dim_ - offset, state.q.size());
  for (std::size_t i = 0; i < q_copy; ++i) {
    obs(static_cast<Eigen::Index>(offset + i)) = state.q[i];
  }
  offset += q_copy;

  if (offset < obs_dim_) {
    const std::size_t dq_copy = std::min(obs_dim_ - offset, state.dq.size());
    for (std::size_t i = 0; i < dq_copy; ++i) {
      obs(static_cast<Eigen::Index>(offset + i)) = state.dq[i];
    }
    offset += dq_copy;
  }

  if (include_last_action_ && offset < obs_dim_) {
    const std::size_t action_copy =
        std::min<std::size_t>(obs_dim_ - offset, static_cast<std::size_t>(last_action.size()));
    obs.segment(static_cast<Eigen::Index>(offset), static_cast<Eigen::Index>(action_copy)) =
        last_action.head(static_cast<Eigen::Index>(action_copy));
  }

  return obs;
}

}  // namespace sim2sim

