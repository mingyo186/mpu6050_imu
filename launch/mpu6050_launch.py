# Copyright 2025 The mpu6050_imu Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch file for mpu6050_imu package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the MPU6050 IMU node."""
    pkg_dir = get_package_share_directory('mpu6050_imu')
    default_params = os.path.join(pkg_dir, 'config', 'mpu6050_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),

        Node(
            package='mpu6050_imu',
            executable='mpu6050_node.py',
            name='mpu6050_imu_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
            ],
        ),
    ])
