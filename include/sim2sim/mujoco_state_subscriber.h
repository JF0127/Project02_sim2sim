#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "sim2sim/sim2sim_types.h"

namespace sim2sim {

class MujocoStateSubscriber {
 public:
  MujocoStateSubscriber(rclcpp::Node& node, const std::string& topic_name);

  bool has_state() const;
  MujocoState latest_state() const;

 private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  mutable std::mutex mutex_;
  bool has_state_{false};
  MujocoState state_;
};

}  // namespace sim2sim

