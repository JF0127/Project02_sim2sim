#include "sim2sim_platform/mujoco_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sim2sim_platform::MujocoNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco_node"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
