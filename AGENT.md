# Sim2Sim/Sim2Real ROS2 项目说明（给大模型与开发者）

## 1. 项目目标

本项目是一个四层解耦的强化学习部署框架：

1. `sim2sim_platform`（C++）：仿真/真机平台接入层（当前实现 MuJoCo）
2. `sim2sim_obs`（Python）：状态 -> 观测封装层
3. `sim2sim_policy`（Python）：策略推理层（当前是 stub，不接真实模型）
4. `sim2sim_action`（C++）：动作后处理层（raw_action -> action_final）

核心原则：

1. 各层仅通过 ROS2 Topic 通讯，尽量不跨层耦合。
2. 更换仿真器、策略、机器人时，优先替换对应单层实现。

---

## 2. 当前真实运行链路

运行 launch：`sim2sim_mujoco_full.launch.py`。

启动节点：

1. `mujoco_node`
2. `obs_process_node`
3. `rl_policy_node`
4. `action_process_node`

主数据流：

1. `/robot/joint_state` (`sensor_msgs/msg/JointState`) 由 `mujoco_node` 发布
2. `/policy/obs` (`std_msgs/msg/Float32MultiArray`) 由 `obs_process_node` 发布
3. `/policy/raw_action` (`std_msgs/msg/Float32MultiArray`) 由 `rl_policy_node` 发布
4. `/robot/action_final` (`std_msgs/msg/Float64MultiArray`) 由 `action_process_node` 发布并被 `mujoco_node` 订阅

当前频率：

1. MuJoCo step: `500Hz`
2. JointState 发布: `100Hz`
3. Obs/Policy/Action 按消息触发（近似 100Hz）

---

## 3. 目录结构（当前有效）

```text
src/
  sim2sim_platform/
    src/mujoco_node.cpp
  sim2sim_obs/
    sim2sim_obs/obs_process_node.py
  sim2sim_policy/
    sim2sim_policy/rl_policy_node.py
  sim2sim_action/
    src/action_process_node.cpp
  sim2sim_bringup/
    launch/sim2sim_mujoco_full.launch.py
    config/platform_mujoco.yaml
    config/obs_processor.yaml
    config/policy_stub.yaml
    config/action_processor.yaml
```

---

## 4. 各层行为契约

### 4.1 Platform（MuJoCo）

文件：`src/sim2sim_platform/src/mujoco_node.cpp`

1. 读取 XML：`/home/jf/lab/3_my_projects/Project02_sim2sim/assets/Mvr_model/Mvr_10dof/Mvr_10dof.xml`
2. 接收 `/robot/action_final`，直接写入 `mjData->ctrl`
3. 发布 `/robot/joint_state`
4. 启用 viewer（配置控制）

注意：`controlled_joint_names` 不能写 `[]` 空数组；空数组在 ROS2 参数里会触发 `No parameter value set`。

### 4.2 Obs Processor

文件：`src/sim2sim_obs/sim2sim_obs/obs_process_node.py`

1. 输入：`joint_state` + `raw_action`
2. 输出：`obs`（float32）
3. Obs 定义：`[q(10), dq(10), last_raw_action(10)]`，共 `30` 维
4. 缺失关节/非法值：使用上次值填充
5. 不做归一化

### 4.3 Policy Stub

文件：`src/sim2sim_policy/sim2sim_policy/rl_policy_node.py`

1. 输入：`/policy/obs`
2. 输出：`/policy/raw_action`
3. `policy_mode` 支持：
   - `copy_last_action`（默认）
   - `zero`
   - `tanh_head`

### 4.4 Action Processor

文件：`src/sim2sim_action/src/action_process_node.cpp`

1. 输入：`/policy/raw_action`（float32）
2. 输出：`/robot/action_final`（float64）
3. 处理：维度检查、NaN/Inf 检查、缩放、可选限幅、无效输入可保持上一帧

---

## 5. 参数文件（真值）

1. `platform_mujoco.yaml`：MuJoCo 节点参数
2. `obs_processor.yaml`：Obs 节点参数
3. `policy_stub.yaml`：Policy stub 参数
4. `action_processor.yaml`：Action 后处理参数

路径：`src/sim2sim_bringup/config/`

---

## 6. 必备依赖

系统：`Ubuntu 22.04 + ROS2 Humble`

1. ROS2：`/opt/ros/humble`
2. MuJoCo：`/home/jf/lab/0_external_libs/mujoco`
3. 可视化依赖（可选但建议）：

```bash
sudo apt update
sudo apt install -y libglfw3-dev libgl1-mesa-dev
```

---

## 7. 构建命令

### 7.1 全量构建

```bash
cd /home/jf/lab/3_my_projects/Project02_sim2sim
source /opt/ros/humble/setup.bash
MUJOCO_ROOT=/home/jf/lab/0_external_libs/mujoco colcon build --symlink-install
```

### 7.2 清缓存重构建（推荐改 CMake 后使用）

```bash
cd /home/jf/lab/3_my_projects/Project02_sim2sim
source /opt/ros/humble/setup.bash
MUJOCO_ROOT=/home/jf/lab/0_external_libs/mujoco colcon build --symlink-install --cmake-clean-cache
```

### 7.3 仅构建单包

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sim2sim_platform
colcon build --symlink-install --packages-select sim2sim_obs
colcon build --symlink-install --packages-select sim2sim_policy
colcon build --symlink-install --packages-select sim2sim_action
colcon build --symlink-install --packages-select sim2sim_bringup
```

---

## 8. 运行命令

### 8.1 一键启动全流程（推荐）

```bash
cd /home/jf/lab/3_my_projects/Project02_sim2sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/jf/lab/0_external_libs/mujoco/lib:$LD_LIBRARY_PATH
export ROS_LOG_DIR=/tmp/ros_log
export DISPLAY=:1
ros2 launch sim2sim_bringup sim2sim_mujoco_full.launch.py
```

### 8.2 手动逐节点启动（调试用）

```bash
source /opt/ros/humble/setup.bash
source /home/jf/lab/3_my_projects/Project02_sim2sim/install/setup.bash
export LD_LIBRARY_PATH=/home/jf/lab/0_external_libs/mujoco/lib:$LD_LIBRARY_PATH

ros2 run sim2sim_platform mujoco_node --ros-args --params-file /home/jf/lab/3_my_projects/Project02_sim2sim/src/sim2sim_bringup/config/platform_mujoco.yaml
ros2 run sim2sim_obs obs_process_node --ros-args --params-file /home/jf/lab/3_my_projects/Project02_sim2sim/src/sim2sim_bringup/config/obs_processor.yaml
ros2 run sim2sim_policy rl_policy_node --ros-args --params-file /home/jf/lab/3_my_projects/Project02_sim2sim/src/sim2sim_bringup/config/policy_stub.yaml
ros2 run sim2sim_action action_process_node --ros-args --params-file /home/jf/lab/3_my_projects/Project02_sim2sim/src/sim2sim_bringup/config/action_processor.yaml
```

---

## 9. 测试命令（可直接复制）

### 9.1 基础观测

```bash
source /opt/ros/humble/setup.bash
source /home/jf/lab/3_my_projects/Project02_sim2sim/install/setup.bash
ros2 topic echo /robot/joint_state --once
ros2 topic echo /policy/obs --once
ros2 topic echo /policy/raw_action --once
ros2 topic echo /robot/action_final --once
```

### 9.2 频率检查

```bash
ros2 topic hz /robot/joint_state
ros2 topic hz /policy/obs
ros2 topic hz /policy/raw_action
ros2 topic hz /robot/action_final
```

### 9.3 拓扑检查

```bash
ros2 topic info /policy/raw_action -v
ros2 topic info /robot/action_final -v
```

### 9.4 手工注入 raw_action（测试 action_process）

```bash
ros2 topic pub --once /policy/raw_action std_msgs/msg/Float32MultiArray "{data: [0.1, 0.0, -0.1, 0.2, -0.2, 0.1, 0.0, -0.1, 0.2, -0.2]}"
ros2 topic echo /robot/action_final --once
```

### 9.5 直接注入 action_final（仅测试 platform；建议停掉 action_process）

```bash
ros2 topic pub --once /robot/action_final std_msgs/msg/Float64MultiArray "{data: [2.0, -2.0, 1.5, -1.5, 1.0, -1.0, 2.5, -2.5, 1.2, -1.2]}"
```

### 9.6 一条命令做冒烟验证（启动+抓消息）

```bash
bash -lc 'source /opt/ros/humble/setup.bash; source /home/jf/lab/3_my_projects/Project02_sim2sim/install/setup.bash; export LD_LIBRARY_PATH=/home/jf/lab/0_external_libs/mujoco/lib:$LD_LIBRARY_PATH; export ROS_LOG_DIR=/tmp/ros_log; export DISPLAY=:1; ros2 launch sim2sim_bringup sim2sim_mujoco_full.launch.py > /tmp/sim2sim_smoke.log 2>&1 & pid=$!; sleep 4; timeout 6 ros2 topic echo /robot/action_final --once; rc=$?; kill -INT $pid; wait $pid 2>/dev/null; exit $rc'
```

---

## 10. Viewer 操作

MuJoCo 可视化窗口交互：

1. 左键拖动：旋转
2. 右键拖动：平移
3. `Shift + 左键`：水平旋转
4. `Shift + 右键`：水平平移
5. 滚轮：缩放

SSH 场景说明：

1. 仅 SSH 且无图形会话时，窗口可能无法显示。
2. 需要正确设置 `DISPLAY`（例如 `:1`）并具备可用 X/桌面环境。

---

## 11. 常见问题

1. `No parameter value set`（`controlled_joint_names`）
   - 原因：YAML 写了空数组 `[]`
   - 解决：删除该项或填写完整关节名列表

2. `Viewer requested but GLFW support is unavailable`
   - 原因：编译时未找到 GLFW/OpenGL
   - 解决：安装依赖后 `--cmake-clean-cache` 重编译

3. MuJoCo 未链接
   - 解决：构建时传 `MUJOCO_ROOT=/home/jf/lab/0_external_libs/mujoco`

4. ROS 日志权限问题
   - 解决：`export ROS_LOG_DIR=/tmp/ros_log`

---

## 12. 给大模型的开发约束

1. 先读本文件，再改代码。
2. 默认保持 Topic 契约不变：
   - `/robot/joint_state` -> `/policy/obs` -> `/policy/raw_action` -> `/robot/action_final`
3. 若修改维度或关节顺序，必须同步更新：
   - `obs_processor.yaml`
   - `policy_stub.yaml`
   - `action_processor.yaml`
   - 本文档的契约说明
4. `sim2sim_obs/resource/sim2sim_obs` 与 `sim2sim_policy/resource/sim2sim_policy` 必须保留（ament_python 包发现需要）。

