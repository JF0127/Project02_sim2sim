#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#if SIM2SIM_HAS_GLFW
#include <GLFW/glfw3.h>
#endif

namespace sim2sim_platform {

struct ControlledJoint {
  std::string joint_name;
  int joint_id;
  int actuator_id;
  int qpos_adr;
  int qvel_adr;
  bool ctrl_limited;
  double ctrl_min;
  double ctrl_max;
};

class MujocoNode : public rclcpp::Node {
public:
  explicit MujocoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MujocoNode() override;

private:
  bool init_model();
  bool init_controlled_joints();
  void action_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void simulation_loop();
  void publish_joint_state();
  void viewer_loop();

  mjModel * model_{nullptr};
  mjData * data_{nullptr};

  std::vector<ControlledJoint> controlled_joints_;
  std::mutex sim_mutex_;

  std::mutex action_mutex_;
  std::vector<double> latest_action_;
  rclcpp::Time latest_action_time_{0, 0, RCL_ROS_TIME};
  bool has_received_action_{false};

  std::atomic<bool> running_{false};
  std::thread sim_thread_;
  std::thread viewer_thread_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr action_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::string xml_path_;
  std::string action_topic_;
  std::string joint_state_topic_;
  std::vector<std::string> controlled_joint_names_;

  double sim_hz_{500.0};
  double pub_hz_{100.0};
  int publish_decimation_{5};
  bool clamp_to_ctrl_range_{true};
  int hold_last_action_timeout_ms_{100};

  bool enable_viewer_{false};
  int viewer_width_{1280};
  int viewer_height_{720};
  double viewer_hz_{60.0};

#if SIM2SIM_HAS_GLFW
  static void mouse_button_callback(GLFWwindow * window, int button, int act, int mods);
  static void cursor_pos_callback(GLFWwindow * window, double xpos, double ypos);
  static void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);
  void handle_mouse_move(double xpos, double ypos);

  GLFWwindow * window_{nullptr};
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scene_;
  mjrContext context_;

  bool button_left_down_{false};
  bool button_middle_down_{false};
  bool button_right_down_{false};
  double last_cursor_x_{0.0};
  double last_cursor_y_{0.0};
#endif
};

}  // namespace sim2sim_platform
