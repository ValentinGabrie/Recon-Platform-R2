"""full_system.launch.py — Launch all roomba nodes in correct order.

Launch order (per Section 5):
  1. joy_node
  2. esp32_sensor_node
  3. motor_controller
  4. joy_control_node
  5. slam_toolbox (online async)
  6. nav2 stack
  7. slam_bridge_node
  8. recon_node
  9. db_node
 10. roomba_webui

All nodes have respawn=True and respawn_delay=2.0.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Config file paths
    # TODO: Resolve config path dynamically based on workspace location
    config_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', '..', 'config'
    )
    hardware_config = os.path.join(config_dir, 'hardware.yaml')
    controller_config = os.path.join(config_dir, 'controller.yaml')
    slam_config = os.path.join(config_dir, 'slam_params.yaml')
    nav2_config = os.path.join(config_dir, 'nav2_params.yaml')

    return LaunchDescription([
        # 1. joy_node (from joy package)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # 2. esp32_sensor_node
        Node(
            package='roomba_hardware',
            executable='esp32_sensor_node',
            name='esp32_sensor_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[hardware_config],
        ),

        # 3. motor_controller
        Node(
            package='roomba_hardware',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[hardware_config],
        ),

        # 4. joy_control_node
        Node(
            package='roomba_control',
            executable='joy_control_node',
            name='joy_control_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[controller_config],
        ),

        # 5. slam_toolbox
        # TODO: Include slam_toolbox launch file with proper config
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory('slam_toolbox'),
        #             'launch', 'online_async_launch.py'
        #         )
        #     ]),
        #     launch_arguments={'params_file': slam_config}.items(),
        # ),

        # 7. slam_bridge_node
        Node(
            package='roomba_navigation',
            executable='slam_bridge_node',
            name='slam_bridge_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # 8. recon_node
        Node(
            package='roomba_navigation',
            executable='recon_node',
            name='recon_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # 9. db_node
        Node(
            package='roomba_db',
            executable='db_node',
            name='db_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # 10. roomba_webui
        Node(
            package='roomba_webui',
            executable='roomba_webui',
            name='roomba_webui',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
