#include "sim2sim/rl_onnx_policy.h"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>
#include <vector>

#ifdef SIM2SIM_USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace sim2sim {

class RlOnnxPolicy::Impl {
 public:
#ifdef SIM2SIM_USE_ONNXRUNTIME
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "sim2sim"};
  Ort::SessionOptions session_options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
#endif
};

RlOnnxPolicy::RlOnnxPolicy(std::string policy_path, std::size_t obs_dim, std::size_t action_dim)
    : policy_path_(std::move(policy_path)), obs_dim_(obs_dim), action_dim_(action_dim), impl_(std::make_unique<Impl>()) {}

RlOnnxPolicy::~RlOnnxPolicy() = default;
RlOnnxPolicy::RlOnnxPolicy(RlOnnxPolicy&&) noexcept = default;
RlOnnxPolicy& RlOnnxPolicy::operator=(RlOnnxPolicy&&) noexcept = default;

void RlOnnxPolicy::init() {
  if (obs_dim_ == 0) {
    throw std::runtime_error("obs_dim must be > 0");
  }
  if (action_dim_ == 0) {
    throw std::runtime_error("action_dim must be > 0");
  }
  if (policy_path_.empty()) {
    throw std::runtime_error("policy.path is empty");
  }
  last_action_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim_));

#ifndef SIM2SIM_USE_ONNXRUNTIME
  throw std::runtime_error(
      "ONNX Runtime is not enabled at compile time. Install onnxruntime and rebuild.");
#else
  impl_->session_options.SetIntraOpNumThreads(1);
  impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  impl_->session = std::make_unique<Ort::Session>(impl_->env, policy_path_.c_str(), impl_->session_options);

  const std::size_t input_count = impl_->session->GetInputCount();
  const std::size_t output_count = impl_->session->GetOutputCount();
  if (input_count == 0 || output_count == 0) {
    throw std::runtime_error("onnx model has no inputs or outputs");
  }

  impl_->input_names.clear();
  impl_->output_names.clear();
  impl_->input_names.reserve(input_count);
  impl_->output_names.reserve(output_count);

  for (std::size_t i = 0; i < input_count; ++i) {
    auto name = impl_->session->GetInputNameAllocated(i, impl_->allocator);
    impl_->input_names.emplace_back(name.get());
  }
  for (std::size_t i = 0; i < output_count; ++i) {
    auto name = impl_->session->GetOutputNameAllocated(i, impl_->allocator);
    impl_->output_names.emplace_back(name.get());
  }

  const auto input_info = impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
  const auto input_shape = input_info.GetShape();
  if (!input_shape.empty()) {
    const auto last_dim = input_shape.back();
    if (last_dim > 0 && static_cast<std::size_t>(last_dim) != obs_dim_) {
      throw std::runtime_error("onnx input dim mismatch with configured obs.dim");
    }
  }

  const auto output_info = impl_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
  const auto output_elem_count = output_info.GetElementCount();
  if (output_elem_count < action_dim_) {
    throw std::runtime_error("onnx output dim is smaller than configured action.dim");
  }
#endif
}

void RlOnnxPolicy::reset() {
  last_action_.setZero(static_cast<Eigen::Index>(action_dim_));
}

Eigen::VectorXd RlOnnxPolicy::forward(const Eigen::VectorXd& obs) {
  if (obs.size() != static_cast<Eigen::Index>(obs_dim_)) {
    throw std::runtime_error("obs dim mismatch in RlOnnxPolicy::forward");
  }

#ifndef SIM2SIM_USE_ONNXRUNTIME
  throw std::runtime_error("RlOnnxPolicy::forward called but ONNX Runtime support is disabled.");
#else
  std::vector<float> input_data(obs_dim_);
  for (std::size_t i = 0; i < obs_dim_; ++i) {
    input_data[i] = static_cast<float>(obs(static_cast<Eigen::Index>(i)));
  }

  const std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(obs_dim_)};
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_data.size(),
                                                             input_shape.data(), input_shape.size());

  const char* input_name = impl_->input_names.front().c_str();
  const char* output_name = impl_->output_names.front().c_str();
  const std::array<const char*, 1> input_names{input_name};
  const std::array<const char*, 1> output_names{output_name};
  const std::array<Ort::Value, 1> input_tensors{std::move(input_tensor)};

  auto output_tensors = impl_->session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(),
                                            input_tensors.size(), output_names.data(), output_names.size());
  if (output_tensors.empty()) {
    throw std::runtime_error("onnx inference returned no outputs");
  }

  auto& out_tensor = output_tensors.front();
  if (!out_tensor.IsTensor()) {
    throw std::runtime_error("onnx output is not a tensor");
  }

  const auto out_info = out_tensor.GetTensorTypeAndShapeInfo();
  const auto elem_count = out_info.GetElementCount();
  if (elem_count < action_dim_) {
    throw std::runtime_error("onnx output elements smaller than action_dim");
  }

  const auto elem_type = out_info.GetElementType();
  if (elem_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    const float* data = out_tensor.GetTensorData<float>();
    for (std::size_t i = 0; i < action_dim_; ++i) {
      last_action_(static_cast<Eigen::Index>(i)) = static_cast<double>(data[i]);
    }
  } else if (elem_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE) {
    const double* data = out_tensor.GetTensorData<double>();
    for (std::size_t i = 0; i < action_dim_; ++i) {
      last_action_(static_cast<Eigen::Index>(i)) = data[i];
    }
  } else {
    throw std::runtime_error("onnx output tensor type is unsupported");
  }

  return last_action_;
#endif
}

}  // namespace sim2sim

