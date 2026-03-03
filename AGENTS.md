# Project02_sim2sim - AGENT Guide (LLM-Oriented)

## 1. Project Mission

This repository is a **pure RL sim2sim** control stack.

Closed-loop target:

1. Subscribe MuJoCo robot state.
2. Build observation vector `obs`.
3. Run ONNX policy forward pass.
4. Post-process and publish `action` to ROS2.
5. MuJoCo consumes action and returns next state.

Out of scope:

1. Real robot deployment path.
2. Motion-tracking/reference gait pipeline.
3. `start_step`, `ext_pos_corr`, and WandB model download flow.

---

## 2. Current Completion Status

Overall completion: **~70% (MVP level)**.

Completed:

1. Architecture migrated from original motion-tracking scaffold to pure RL sim2sim.
2. Core C++ modules implemented:
   - `MujocoStateSubscriber`
   - `ObservationBuilder`
   - `RlOnnxPolicy` (ONNX Runtime path)
   - `ActionCodec`
   - `Sim2SimControllerNode`
3. Launch + param flow implemented:
   - `launch/mujoco.launch.py`
   - `config/g1/sim2sim.yaml`
4. Build validated on Ubuntu 22.04 + ROS2 Humble.
5. ONNX Runtime discovered and linked with:
   - `ONNXRUNTIME_ROOT=/usr/local/onnxruntime`

Not completed:

1. `obs.names` schema and strict train/deploy feature mapping.
2. Action protocol alignment with MuJoCo side beyond raw `Float64MultiArray`.
3. Base-state features integration (currently mainly `q/dq`).
4. Advanced ONNX input/output mapping (multi-input/multi-output, recurrent states).
5. Runtime telemetry (latency/frequency watchdog) and robust error recovery.

---

## 3. Code Map

Entry:

1. `src/main.cpp`
2. `src/sim2sim_controller_node.cpp`

Core headers:

1. `include/sim2sim/sim2sim_types.h`
2. `include/sim2sim/mujoco_state_subscriber.h`
3. `include/sim2sim/observation_builder.h`
4. `include/sim2sim/rl_onnx_policy.h`
5. `include/sim2sim/action_codec.h`
6. `include/sim2sim/sim2sim_controller_node.h`

Build and launch:

1. `CMakeLists.txt`
2. `package.xml`
3. `launch/mujoco.launch.py`
4. `config/g1/sim2sim.yaml`

---

## 4. Runtime Interfaces

## 4.1 ROS Topics

1. MuJoCo -> Controller:
   - `sensor_msgs/msg/JointState`
   - default topic: `/mujoco/joint_states`
2. Controller -> MuJoCo:
   - `std_msgs/msg/Float64MultiArray`
   - default topic: `/sim2sim/action`

## 4.2 Launch Arguments (`launch/mujoco.launch.py`)

1. `policy_path` (string): ONNX model path.
2. `params_file` (string): parameter yaml path (default package config).
3. `state_topic` (string): state input topic.
4. `action_topic` (string): action output topic.

## 4.3 Node Parameters (`sim2sim_controller`)

1. `loop_hz` (double)
2. `obs.dim` (int)
3. `obs.include_last_action` (bool)
4. `action.dim` (int)
5. `action.scale` (double)
6. `action.clip` (double)
7. `io.state_topic` (string)
8. `io.action_topic` (string)
9. `policy.path` (string)

---

## 5. Class Contracts

## 5.1 `MujocoStateSubscriber`

Responsibilities:

1. Subscribe joint-state topic.
2. Cache latest state snapshot.

Main APIs:

1. `has_state()`
2. `latest_state()`

## 5.2 `ObservationBuilder`

Responsibilities:

1. Build fixed-dim observation vector.
2. Optionally include last action.

Main APIs:

1. `build(const MujocoState&, const Eigen::VectorXd&)`
2. `obs_dim()`

## 5.3 `RlOnnxPolicy`

Responsibilities:

1. Load ONNX model using ONNX Runtime.
2. Validate dimensions.
3. Run forward inference and return action vector.

Main APIs:

1. `init()`
2. `reset()`
3. `forward(const Eigen::VectorXd&)`

## 5.4 `ActionCodec`

Responsibilities:

1. Apply scale and clip to raw action.
2. Encode action into ROS message.

Main APIs:

1. `sanitize(const Eigen::VectorXd&)`
2. `encode(const Eigen::VectorXd&)`

## 5.5 `Sim2SimControllerNode`

Responsibilities:

1. Wire all modules into timer-based loop.
2. Execute one tick: `state -> obs -> policy -> action`.

Main API:

1. `tick()`

---

## 6. Build & Run

## 6.1 Build (recommended)

```bash
source /opt/ros/humble/setup.bash
ONNXRUNTIME_ROOT=/usr/local/onnxruntime cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
ONNXRUNTIME_ROOT=/usr/local/onnxruntime cmake --build build -j
```

## 6.2 Build (colcon)

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motion_tracking_controller
source install/setup.bash
```

## 6.3 Run

```bash
source /opt/ros/humble/setup.bash
ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  params_file:=/abs/path/to/sim2sim.yaml
```

---

## 7. Engineering Constraints

1. Keep sim2sim-only scope.
2. Do not re-introduce legacy motion-tracking SDK dependencies.
3. Enforce explicit runtime dimension checks for `obs` and `action`.
4. Keep feature ordering configurable and documented.
5. Update this file before introducing new interfaces.

---

## 8. Next Priorities

1. Introduce `obs.names` and strict feature-order contract with training pipeline.
2. Replace raw action message with explicit action protocol (joint_names, mode, timestamp).
3. Add base pose/angular velocity (and optional contacts) into observation.
4. Add inference and control-loop telemetry (latency, jitter, drop detection).
5. Extend ONNX mapping to multi-input/multi-output policies.
