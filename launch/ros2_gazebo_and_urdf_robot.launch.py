"""Launch Gazebo server and client with command line arguments."""
"""Spawn robot from URDF file."""


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = 'barista_robot_description'
    world_file_name = 'empty.world'
    urdf_file_name = 'barista_robot_model.urdf'
    mesh_file_name = 'rplidar_a2.dae'
    world = os.path.join(get_package_share_directory(
        robot_name), 'worlds', world_file_name)

    urdf = os.path.join(get_package_share_directory(
        robot_name), 'urdf', urdf_file_name)

    meshes = os.path.join(get_package_share_directory(
        robot_name), 'meshes', mesh_file_name)
    
    print("MESHEHEHEHEHS: " )
    print(meshes)
    xml = open(urdf, 'r').read()

    xml = xml.replace('"', '\\"')

    swpan_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo',
                 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen'),
    ])