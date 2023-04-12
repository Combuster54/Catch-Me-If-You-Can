#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_pkg = get_package_share_directory('barista_robot_description')

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch',
                         'start_world.launch.py'),
        )
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch', 'spawn_robot_description.launch.py'),
        )
    )

    visualize_meshes_collisions_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch', 'urdf_visualize_meshes_collisions_inertias.launch.py'),
        )
    )

    return LaunchDescription([
        start_world,
        spawn_robot,
        visualize_meshes_collisions_and_rviz,

    ])