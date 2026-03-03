#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "sim2sim/sim2sim_types.h"

namespace sim2sim {

class ObservationBuilder {
 public:
  ObservationBuilder(std::size_t obs_dim, bool include_last_action);

  Eigen::VectorXd build(const MujocoState& state, const Eigen::VectorXd& last_action) const;
  std::size_t obs_dim() const { return obs_dim_; }

 private:
  std::size_t obs_dim_;
  bool include_last_action_;
};

}  // namespace sim2sim

