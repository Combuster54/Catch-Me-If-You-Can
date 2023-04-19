from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rick = Node(
        package='barista_robot_description',
        executable='world_to_rick',
        output='screen')
        
    morty = Node(
        package='barista_robot_description',
        executable='world_to_morty',
        output='screen')

    return LaunchDescription([
        rick,
        morty
    ])