from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='services_quiz',
            executable='turn_s_client',  # <- client only
            name='turn_s_client',
            output='screen'
        )
    ])
