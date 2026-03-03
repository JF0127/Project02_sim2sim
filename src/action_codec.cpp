#include "sim2sim/action_codec.h"

#include <algorithm>
#include <cmath>

namespace sim2sim {

ActionCodec::ActionCodec(std::size_t action_dim, double scale, double clip)
    : action_dim_(action_dim), scale_(scale), clip_(clip) {}

Eigen::VectorXd ActionCodec::sanitize(const Eigen::VectorXd& action_raw) const {
  Eigen::VectorXd out = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim_));
  const Eigen::Index copy_dim = std::min<Eigen::Index>(action_raw.size(), static_cast<Eigen::Index>(action_dim_));
  for (Eigen::Index i = 0; i < copy_dim; ++i) {
    const double v = static_cast<double>(action_raw(i));
    const double scaled = std::isfinite(v) ? v * scale_ : 0.0;
    out(i) = std::clamp(scaled, -clip_, clip_);
  }
  return out;
}

std_msgs::msg::Float64MultiArray ActionCodec::encode(const Eigen::VectorXd& action) const {
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(static_cast<std::size_t>(action.size()));
  for (Eigen::Index i = 0; i < action.size(); ++i) {
    msg.data[static_cast<std::size_t>(i)] = action(i);
  }
  return msg;
}

}  // namespace sim2sim

