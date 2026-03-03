#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "sim2sim/action_codec.h"
#include "sim2sim/mujoco_state_subscriber.h"
#include "sim2sim/observation_builder.h"
#include "sim2sim/rl_onnx_policy.h"

namespace sim2sim {

class Sim2SimControllerNode : public rclcpp::Node {
 public:
  explicit Sim2SimControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void tick();

  MujocoStateSubscriber state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_pub_;
  ObservationBuilder obs_builder_;
  RlOnnxPolicy policy_;
  ActionCodec action_codec_;
  Eigen::VectorXd last_action_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool logged_dims_{false};
};

}  // namespace sim2sim

