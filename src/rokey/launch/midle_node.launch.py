from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rokey',
            executable='cook_node',
            name='weather_node',
            output='screen'
        ),
        Node(
            package='rokey',
            executable='breakfast_node',
            name='breakfast_node',
            output='screen'
        ),
        Node(
            package='rokey',
            executable='cleanup_node',
            name='cleanup_node',
            output='screen'
        ),
    ])
