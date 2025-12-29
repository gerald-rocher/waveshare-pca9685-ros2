from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_controller',
            executable='servo_node',
            name='servo_controller_node',
            output='screen'
        )
    ])

