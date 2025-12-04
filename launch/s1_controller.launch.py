import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('s1_controller'),
        'config',
        's1_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='s1_controller',
            executable='pid_node',
            name='s1_feedback_controller',
            parameters=[config],
            output='screen',
            # 必要に応じてリマップもここに記述可能
            # remappings=[('/cmd_vel_out', '/s1_driver/cmd_vel')]
        )
    ])
