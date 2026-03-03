from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    policy_path = LaunchConfiguration("policy_path")
    params_file = LaunchConfiguration("params_file")
    state_topic = LaunchConfiguration("state_topic")
    action_topic = LaunchConfiguration("action_topic")

    sim2sim_node = Node(
        package="motion_tracking_controller",
        executable="sim2sim_node",
        output="screen",
        parameters=[
            params_file,
            {
                "policy.path": policy_path,
                "io.state_topic": state_topic,
                "io.action_topic": action_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_path",
                default_value="",
                description="Absolute path to RL ONNX model.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("motion_tracking_controller"), "config", "g1", "sim2sim.yaml"]
                ),
                description="Parameter file path (absolute path recommended).",
            ),
            DeclareLaunchArgument(
                "state_topic",
                default_value="/mujoco/joint_states",
                description="Mujoco state topic.",
            ),
            DeclareLaunchArgument(
                "action_topic",
                default_value="/sim2sim/action",
                description="Action output topic to Mujoco.",
            ),
            sim2sim_node,
        ]
    )
