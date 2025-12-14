import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch RViz'
        )
    )

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('go2w_description'), 'launch', 'robot.launch.py'])
        ])
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('go2_rviz'), 'config', 'go2_rviz.rviz'])],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription(declared_arguments + [
        robot_description_cmd,
        joint_state_publisher_gui_node,
        rviz_node
    ])
