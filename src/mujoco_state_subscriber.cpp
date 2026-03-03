#include "sim2sim/mujoco_state_subscriber.h"

namespace sim2sim {

MujocoStateSubscriber::MujocoStateSubscriber(rclcpp::Node& node, const std::string& topic_name) {
  sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
      topic_name, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });
}

bool MujocoStateSubscriber::has_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_state_;
}

MujocoState MujocoStateSubscriber::latest_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void MujocoStateSubscriber::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.q = msg->position;
  state_.dq = msg->velocity;
  state_.stamp = rclcpp::Time(msg->header.stamp);
  has_state_ = true;
}

}  // namespace sim2sim
