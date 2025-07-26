from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_drive',
            executable='keyboard_node', 
            name='keyboard_nodeS',
            output='screen'
        ),
        Node(
            package='mecanum_drive',
            executable='mecanum_node',
            name='mecanum_node',
            output='screen'
        )
    ])
