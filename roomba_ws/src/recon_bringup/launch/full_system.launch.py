"""full_system.launch.py — Launch the Recon-Platform-R2 handheld scanner stack.

Launch order (Stage H1 — pre-IMU/ESP32):
    1. ldlidar_stl_ros2 (LD14P driver) — published externally; included by setup.sh
    2. slam_toolbox online_async (TBD: included once IMU pipeline lands in H3)
    3. db_node
    4. recon_webui

Future stages (H2–H4) will add:
    - esp32_uart_bridge (Pi-side UART → /imu/data_raw + /buttons/*)
    - imu_filter_madgwick → /imu/data
    - robot_localization ekf_node → /odom

All nodes have respawn=True and respawn_delay=2.0.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='recon_db',
            executable='db_node',
            name='db_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='recon_webui',
            executable='recon_webui',
            name='recon_webui',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
