from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='user_input',
            executable='run_inputNode',
            name='inputNode',
            output='screen',
        ),
    ])
