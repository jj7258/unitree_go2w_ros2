from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2w_mock',
            executable='mock_node',
            name='mock_node',
            output='screen'
        )
    ])
