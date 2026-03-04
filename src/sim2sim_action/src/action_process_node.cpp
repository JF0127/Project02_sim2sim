#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace sim2sim_action {

class ActionProcessNode : public rclcpp::Node {
public:
  ActionProcessNode()
  : Node("action_process_node")
  {
    raw_action_topic_ = this->declare_parameter<std::string>("raw_action_topic", "/policy/raw_action");
    action_final_topic_ =
      this->declare_parameter<std::string>("action_final_topic", "/robot/action_final");
    controlled_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "controlled_joint_names",
      std::vector<std::string>{
        "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_hip_pitch_joint", "right_knee_joint", "right_ankle_pitch_joint"});
    action_dim_ = this->declare_parameter<int>("action_dim", 10);
    action_scale_ = this->declare_parameter<double>("action_scale", 1.0);
    enable_clip_ = this->declare_parameter<bool>("enable_clip", false);
    clip_min_ = this->declare_parameter<double>("clip_min", -10.0);
    clip_max_ = this->declare_parameter<double>("clip_max", 10.0);
    hold_last_on_invalid_ = this->declare_parameter<bool>("hold_last_on_invalid", true);

    if (!controlled_joint_names_.empty()) {
      if (action_dim_ != static_cast<int>(controlled_joint_names_.size())) {
        RCLCPP_WARN(
          this->get_logger(),
          "action_dim=%d differs from controlled_joint_names size=%zu, use controlled_joint_names size.",
          action_dim_, controlled_joint_names_.size());
      }
      action_dim_ = static_cast<int>(controlled_joint_names_.size());
    }
    if (action_dim_ <= 0) {
      throw std::runtime_error("action_dim must be > 0");
    }
    if (clip_min_ > clip_max_) {
      throw std::runtime_error("clip_min must be <= clip_max");
    }

    last_action_.assign(static_cast<size_t>(action_dim_), 0.0);

    raw_action_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      raw_action_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ActionProcessNode::raw_action_callback, this, std::placeholders::_1));

    action_final_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      action_final_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "action_process_node started. raw_action_topic=%s action_final_topic=%s action_dim=%d mode=passthrough",
      raw_action_topic_.c_str(), action_final_topic_.c_str(), action_dim_);
  }

private:
  void raw_action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != static_cast<size_t>(action_dim_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "raw_action size mismatch: got %zu expected %d.", msg->data.size(), action_dim_);
      if (hold_last_on_invalid_) {
        publish_action(last_action_);
      }
      return;
    }

    std::vector<double> action;
    action.reserve(static_cast<size_t>(action_dim_));

    for (float raw_v : msg->data) {
      const double v = static_cast<double>(raw_v);
      if (!std::isfinite(v)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "raw_action contains NaN/Inf.");
        if (hold_last_on_invalid_) {
          publish_action(last_action_);
        }
        return;
      }

      double out_v = action_scale_ * v;
      if (enable_clip_) {
        out_v = std::clamp(out_v, clip_min_, clip_max_);
      }
      action.push_back(out_v);
    }

    last_action_ = action;
    publish_action(action);
  }

  void publish_action(const std::vector<double> & action)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "action_final";
    msg.layout.dim[0].size = action.size();
    msg.layout.dim[0].stride = action.size();
    msg.layout.data_offset = 0;
    msg.data = action;
    action_final_pub_->publish(std::move(msg));
  }

  std::string raw_action_topic_;
  std::string action_final_topic_;
  std::vector<std::string> controlled_joint_names_;

  int action_dim_{10};
  double action_scale_{1.0};
  bool enable_clip_{false};
  double clip_min_{-10.0};
  double clip_max_{10.0};
  bool hold_last_on_invalid_{true};

  std::vector<double> last_action_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_action_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_final_pub_;
};

}  // namespace sim2sim_action

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sim2sim_action::ActionProcessNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("action_process_node"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
