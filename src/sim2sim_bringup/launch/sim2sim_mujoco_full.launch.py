from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('sim2sim_bringup')
    platform_config_path = share_dir + '/config/platform_mujoco.yaml'
    obs_config_path = share_dir + '/config/obs_processor.yaml'
    policy_config_path = share_dir + '/config/policy_stub.yaml'
    action_config_path = share_dir + '/config/action_processor.yaml'
    return LaunchDescription([
        Node(
            package='sim2sim_platform',
            executable='mujoco_node',
            name='mujoco_node',
            output='screen',
            parameters=[platform_config_path],
        ),
        Node(
            package='sim2sim_obs',
            executable='obs_process_node',
            name='obs_process_node',
            output='screen',
            parameters=[obs_config_path],
        ),
        Node(
            package='sim2sim_policy',
            executable='rl_policy_node',
            name='rl_policy_node',
            output='screen',
            parameters=[policy_config_path],
        ),
        Node(
            package='sim2sim_action',
            executable='action_process_node',
            name='action_process_node',
            output='screen',
            parameters=[action_config_path],
        ),
    ])
