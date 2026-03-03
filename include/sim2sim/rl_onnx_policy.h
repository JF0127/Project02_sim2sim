#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <Eigen/Core>

namespace sim2sim {

class RlOnnxPolicy {
 public:
  RlOnnxPolicy(std::string policy_path, std::size_t obs_dim, std::size_t action_dim);
  ~RlOnnxPolicy();

  RlOnnxPolicy(const RlOnnxPolicy&) = delete;
  RlOnnxPolicy& operator=(const RlOnnxPolicy&) = delete;
  RlOnnxPolicy(RlOnnxPolicy&&) noexcept;
  RlOnnxPolicy& operator=(RlOnnxPolicy&&) noexcept;

  void init();
  void reset();
  Eigen::VectorXd forward(const Eigen::VectorXd& obs);

  std::size_t obs_dim() const { return obs_dim_; }
  std::size_t action_dim() const { return action_dim_; }

 private:
  std::string policy_path_;
  std::size_t obs_dim_;
  std::size_t action_dim_;
  Eigen::VectorXd last_action_;
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sim2sim
