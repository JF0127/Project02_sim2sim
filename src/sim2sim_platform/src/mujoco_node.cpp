#include "sim2sim_platform/mujoco_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_map>

#if SIM2SIM_HAS_GLFW
#include <GLFW/glfw3.h>
#endif

namespace sim2sim_platform {
namespace {
constexpr int kMaxMujocoErrorSize = 1024;
}  // namespace

MujocoNode::MujocoNode(const rclcpp::NodeOptions & options)
: Node("mujoco_node", options)
{
  xml_path_ = this->declare_parameter<std::string>("xml_path", "");
  action_topic_ = this->declare_parameter<std::string>("action_topic", "/robot/action_final");
  joint_state_topic_ =
    this->declare_parameter<std::string>("joint_state_topic", "/robot/joint_state");
  controlled_joint_names_ =
    this->declare_parameter<std::vector<std::string>>(
    "controlled_joint_names", std::vector<std::string>{});
  sim_hz_ = this->declare_parameter<double>("sim_hz", 500.0);
  pub_hz_ = this->declare_parameter<double>("pub_hz", 100.0);
  clamp_to_ctrl_range_ = this->declare_parameter<bool>("clamp_to_ctrl_range", true);
  hold_last_action_timeout_ms_ = this->declare_parameter<int>("hold_last_action_timeout_ms", 100);
  enable_viewer_ = this->declare_parameter<bool>("enable_viewer", false);
  viewer_width_ = this->declare_parameter<int>("viewer_width", 1280);
  viewer_height_ = this->declare_parameter<int>("viewer_height", 720);
  viewer_hz_ = this->declare_parameter<double>("viewer_hz", 60.0);

  if (xml_path_.empty()) {
    throw std::runtime_error("Parameter 'xml_path' is required.");
  }
  if (sim_hz_ <= 0.0 || pub_hz_ <= 0.0) {
    throw std::runtime_error("'sim_hz' and 'pub_hz' must be > 0.");
  }

  publish_decimation_ = std::max(1, static_cast<int>(std::round(sim_hz_ / pub_hz_)));

  if (!init_model()) {
    throw std::runtime_error("Failed to initialize MuJoCo model/data.");
  }
  if (!init_controlled_joints()) {
    throw std::runtime_error("Failed to initialize controlled joints.");
  }

  latest_action_.assign(controlled_joints_.size(), 0.0);
  for (size_t i = 0; i < controlled_joints_.size(); ++i) {
    const int aid = controlled_joints_[i].actuator_id;
    latest_action_[i] = data_->ctrl[aid];
  }

  action_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    action_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MujocoNode::action_callback, this, std::placeholders::_1));

  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::SensorDataQoS());

  running_.store(true);

#if SIM2SIM_HAS_GLFW
  if (enable_viewer_) {
    viewer_thread_ = std::thread(&MujocoNode::viewer_loop, this);
  }
#else
  if (enable_viewer_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Viewer requested but GLFW support is unavailable in this build. Rebuild with GLFW installed.");
  }
#endif

  sim_thread_ = std::thread(&MujocoNode::simulation_loop, this);

  RCLCPP_INFO(
    this->get_logger(),
    "mujoco_node started. sim_hz=%.1f pub_hz=%.1f controlled_joints=%zu viewer=%s",
    sim_hz_, pub_hz_, controlled_joints_.size(), enable_viewer_ ? "on" : "off");
}

MujocoNode::~MujocoNode()
{
  running_.store(false);
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }

  if (data_ != nullptr) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

bool MujocoNode::init_model()
{
  char error[kMaxMujocoErrorSize] = {0};
  model_ = mj_loadXML(xml_path_.c_str(), nullptr, error, kMaxMujocoErrorSize);
  if (model_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "mj_loadXML failed: %s", error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "mj_makeData failed.");
    return false;
  }

  return true;
}

bool MujocoNode::init_controlled_joints()
{
  std::unordered_map<int, int> first_actuator_by_joint;
  for (int aid = 0; aid < model_->nu; ++aid) {
    if (model_->actuator_trntype[aid] != mjTRN_JOINT) {
      continue;
    }
    const int joint_id = model_->actuator_trnid[2 * aid];
    if (joint_id < 0) {
      continue;
    }
    if (first_actuator_by_joint.find(joint_id) == first_actuator_by_joint.end()) {
      first_actuator_by_joint[joint_id] = aid;
    }
  }

  if (first_actuator_by_joint.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No joint transmission actuators found in model.");
    return false;
  }

  std::vector<int> selected_joint_ids;
  if (controlled_joint_names_.empty()) {
    selected_joint_ids.reserve(first_actuator_by_joint.size());
    for (const auto & kv : first_actuator_by_joint) {
      selected_joint_ids.push_back(kv.first);
    }
    std::sort(selected_joint_ids.begin(), selected_joint_ids.end());
  } else {
    selected_joint_ids.reserve(controlled_joint_names_.size());
    for (const auto & joint_name : controlled_joint_names_) {
      const int jid = mj_name2id(model_, mjOBJ_JOINT, joint_name.c_str());
      if (jid < 0) {
        RCLCPP_ERROR(this->get_logger(), "Configured joint '%s' not found in model.", joint_name.c_str());
        return false;
      }
      selected_joint_ids.push_back(jid);
    }
  }

  controlled_joints_.clear();
  controlled_joints_.reserve(selected_joint_ids.size());

  for (const int jid : selected_joint_ids) {
    auto it = first_actuator_by_joint.find(jid);
    if (it == first_actuator_by_joint.end()) {
      const char * joint_name = mj_id2name(model_, mjOBJ_JOINT, jid);
      RCLCPP_ERROR(
        this->get_logger(), "Joint '%s' has no joint-type actuator.",
        (joint_name != nullptr) ? joint_name : "<unnamed>");
      return false;
    }

    const int aid = it->second;
    const int jtype = model_->jnt_type[jid];
    if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE) {
      const char * joint_name = mj_id2name(model_, mjOBJ_JOINT, jid);
      RCLCPP_ERROR(
        this->get_logger(),
        "Joint '%s' is not 1-DoF hinge/slide; current node supports 1-DoF joints only.",
        (joint_name != nullptr) ? joint_name : "<unnamed>");
      return false;
    }

    ControlledJoint info;
    const char * joint_name = mj_id2name(model_, mjOBJ_JOINT, jid);
    info.joint_name = (joint_name != nullptr) ? joint_name : ("joint_" + std::to_string(jid));
    info.joint_id = jid;
    info.actuator_id = aid;
    info.qpos_adr = model_->jnt_qposadr[jid];
    info.qvel_adr = model_->jnt_dofadr[jid];
    info.ctrl_limited = (model_->actuator_ctrllimited[aid] != 0);
    info.ctrl_min = model_->actuator_ctrlrange[2 * aid];
    info.ctrl_max = model_->actuator_ctrlrange[2 * aid + 1];

    controlled_joints_.push_back(info);
  }

  return true;
}

void MujocoNode::action_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != controlled_joints_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Action size mismatch: got %zu expected %zu. Dropped.", msg->data.size(),
      controlled_joints_.size());
    return;
  }

  for (double v : msg->data) {
    if (!std::isfinite(v)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Action contains NaN/Inf. Dropped.");
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    latest_action_ = msg->data;
    latest_action_time_ = this->now();
    has_received_action_ = true;
  }
}

void MujocoNode::simulation_loop()
{
  const auto period = std::chrono::microseconds(static_cast<int64_t>(1e6 / sim_hz_));
  auto next_tick = std::chrono::steady_clock::now() + period;

  int step_count = 0;
  std::vector<double> action_local(controlled_joints_.size(), 0.0);

  while (rclcpp::ok() && running_.load()) {
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      action_local = latest_action_;

      if (has_received_action_) {
        const auto age = (this->now() - latest_action_time_).nanoseconds();
        const auto timeout_ns = static_cast<int64_t>(hold_last_action_timeout_ms_) * 1000 * 1000;
        if (age > timeout_ns) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Action timeout (%d ms), holding last action.", hold_last_action_timeout_ms_);
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      for (size_t i = 0; i < controlled_joints_.size(); ++i) {
        const ControlledJoint & cj = controlled_joints_[i];
        double v = action_local[i];
        if (clamp_to_ctrl_range_ && cj.ctrl_limited) {
          v = std::clamp(v, cj.ctrl_min, cj.ctrl_max);
        }
        data_->ctrl[cj.actuator_id] = v;
      }

      mj_step(model_, data_);
    }

    ++step_count;
    if (step_count % publish_decimation_ == 0) {
      publish_joint_state();
    }

    std::this_thread::sleep_until(next_tick);
    next_tick += period;
  }
}

void MujocoNode::publish_joint_state()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();

  msg.name.reserve(controlled_joints_.size());
  msg.position.reserve(controlled_joints_.size());
  msg.velocity.reserve(controlled_joints_.size());
  msg.effort.reserve(controlled_joints_.size());

  {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    for (const auto & cj : controlled_joints_) {
      msg.name.push_back(cj.joint_name);
      msg.position.push_back(data_->qpos[cj.qpos_adr]);
      msg.velocity.push_back(data_->qvel[cj.qvel_adr]);
      msg.effort.push_back(data_->qfrc_actuator[cj.qvel_adr]);
    }
  }

  joint_state_pub_->publish(std::move(msg));
}

void MujocoNode::viewer_loop()
{
#if SIM2SIM_HAS_GLFW
  if (!enable_viewer_) {
    return;
  }

  if (viewer_hz_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "viewer_hz <= 0, fallback to 60 Hz.");
    viewer_hz_ = 60.0;
  }

  if (!glfwInit()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW, viewer disabled.");
    return;
  }

  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  window_ = glfwCreateWindow(viewer_width_, viewer_height_, "MuJoCo Viewer", nullptr, nullptr);
  if (window_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window, viewer disabled.");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &MujocoNode::mouse_button_callback);
  glfwSetCursorPosCallback(window_, &MujocoNode::cursor_pos_callback);
  glfwSetScrollCallback(window_, &MujocoNode::scroll_callback);

  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);
  mjv_makeScene(model_, &scene_, 2000);
  mjr_makeContext(model_, &context_, mjFONTSCALE_150);
  cam_.type = mjCAMERA_FREE;

  const auto period = std::chrono::microseconds(static_cast<int64_t>(1e6 / viewer_hz_));
  auto next_tick = std::chrono::steady_clock::now() + period;

  while (running_.load() && rclcpp::ok() && !glfwWindowShouldClose(window_)) {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    const mjrRect viewport = {0, 0, width, height};

    {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
    }

    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();

    std::this_thread::sleep_until(next_tick);
    next_tick += period;
  }

  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);
  glfwDestroyWindow(window_);
  window_ = nullptr;
  glfwTerminate();

  if (running_.load()) {
    running_.store(false);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
#else
  (void)this;
#endif
}

#if SIM2SIM_HAS_GLFW
void MujocoNode::mouse_button_callback(GLFWwindow * window, int button, int act, int /*mods*/)
{
  auto * self = static_cast<MujocoNode *>(glfwGetWindowUserPointer(window));
  if (self == nullptr) {
    return;
  }

  self->button_left_down_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  self->button_middle_down_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  self->button_right_down_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

  if (act == GLFW_PRESS) {
    glfwGetCursorPos(window, &self->last_cursor_x_, &self->last_cursor_y_);
  }

  (void)button;
}

void MujocoNode::cursor_pos_callback(GLFWwindow * window, double xpos, double ypos)
{
  auto * self = static_cast<MujocoNode *>(glfwGetWindowUserPointer(window));
  if (self == nullptr) {
    return;
  }
  self->handle_mouse_move(xpos, ypos);
}

void MujocoNode::scroll_callback(GLFWwindow * window, double /*xoffset*/, double yoffset)
{
  auto * self = static_cast<MujocoNode *>(glfwGetWindowUserPointer(window));
  if (self == nullptr) {
    return;
  }
  mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &self->scene_, &self->cam_);
}

void MujocoNode::handle_mouse_move(double xpos, double ypos)
{
  if (!button_left_down_ && !button_middle_down_ && !button_right_down_) {
    last_cursor_x_ = xpos;
    last_cursor_y_ = ypos;
    return;
  }

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window_, &width, &height);
  if (height <= 0) {
    return;
  }

  const double dx = xpos - last_cursor_x_;
  const double dy = ypos - last_cursor_y_;
  last_cursor_x_ = xpos;
  last_cursor_y_ = ypos;

  const bool shift_pressed =
    glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
    glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

  mjtMouse action;
  if (button_right_down_) {
    action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left_down_) {
    action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(model_, action, dx / static_cast<double>(height), dy / static_cast<double>(height), &scene_, &cam_);
}
#endif

}  // namespace sim2sim_platform
