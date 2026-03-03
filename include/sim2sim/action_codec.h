#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace sim2sim {

class ActionCodec {
 public:
  ActionCodec(std::size_t action_dim, double scale, double clip);

  Eigen::VectorXd sanitize(const Eigen::VectorXd& action_raw) const;
  std_msgs::msg::Float64MultiArray encode(const Eigen::VectorXd& action) const;

 private:
  std::size_t action_dim_;
  double scale_;
  double clip_;
};

}  // namespace sim2sim

