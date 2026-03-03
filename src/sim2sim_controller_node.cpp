#include "sim2sim/sim2sim_controller_node.h"

#include <chrono>
#include <stdexcept>
#include <string>

namespace sim2sim {

Sim2SimControllerNode::Sim2SimControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("sim2sim_controller", options),
      state_sub_(*this, this->declare_parameter<std::string>("io.state_topic", "/mujoco/joint_states")),
      obs_builder_(
          static_cast<std::size_t>(this->declare_parameter<int64_t>("obs.dim", 36)),
          this->declare_parameter<bool>("obs.include_last_action", true)),
      policy_(
          this->declare_parameter<std::string>("policy.path", ""),
          static_cast<std::size_t>(this->get_parameter("obs.dim").as_int()),
          static_cast<std::size_t>(this->declare_parameter<int64_t>("action.dim", 12))),
      action_codec_(
          static_cast<std::size_t>(this->get_parameter("action.dim").as_int()),
          this->declare_parameter<double>("action.scale", 1.0),
          this->declare_parameter<double>("action.clip", 1.0)),
      last_action_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(static_cast<std::size_t>(this->get_parameter("action.dim").as_int())))) {
  const auto action_topic = this->declare_parameter<std::string>("io.action_topic", "/sim2sim/action");
  action_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(action_topic, rclcpp::SystemDefaultsQoS());

  policy_.init();
  policy_.reset();

  const auto hz = this->declare_parameter<double>("loop_hz", 50.0);
  if (hz <= 0.0) {
    throw std::runtime_error("loop_hz must be > 0");
  }
  const auto period = std::chrono::duration<double>(1.0 / hz);
  timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                   [this]() { tick(); });

  RCLCPP_INFO(get_logger(), "sim2sim_controller started. state_topic=%s action_topic=%s hz=%.2f",
              this->get_parameter("io.state_topic").as_string().c_str(), action_topic.c_str(), hz);
}

void Sim2SimControllerNode::tick() {
  if (!state_sub_.has_state()) {
    return;
  }

  const auto state = state_sub_.latest_state();
  const Eigen::VectorXd obs = obs_builder_.build(state, last_action_);
  const Eigen::VectorXd action_raw = policy_.forward(obs);
  last_action_ = action_codec_.sanitize(action_raw);
  action_pub_->publish(action_codec_.encode(last_action_));

  if (!logged_dims_) {
    logged_dims_ = true;
    RCLCPP_INFO(get_logger(), "first tick: q_dim=%zu dq_dim=%zu obs_dim=%ld action_dim=%ld", state.q.size(),
                state.dq.size(), obs.size(), last_action_.size());
  }
}

}  // namespace sim2sim
