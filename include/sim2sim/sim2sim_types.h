#pragma once

#include <vector>

#include <rclcpp/time.hpp>

namespace sim2sim {

struct MujocoState {
  std::vector<double> q;
  std::vector<double> dq;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

}  // namespace sim2sim

