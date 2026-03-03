#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sim2sim/sim2sim_controller_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim2sim::Sim2SimControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

