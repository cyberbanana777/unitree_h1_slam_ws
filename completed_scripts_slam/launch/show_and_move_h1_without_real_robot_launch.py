# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the paths to the launch files of other packages
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('h1_description'), 'launch'
    )

    # Enabling the first launch file
    launch_file1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg1_launch_dir, 'display_without_control_launch.py')
        )
    )

    move_node = Node(
        package='h1_move_joint_rviz',
        executable='move_joint_rviz_without_real_robot_node'
    )

    return LaunchDescription([
        launch_file1,
        move_node,
    ])