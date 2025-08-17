# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('h1_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'h1_with_hand.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.95', '0.0', '0.0', '0.0', 'base_footprint', 'pelvis']
        ),

    ])