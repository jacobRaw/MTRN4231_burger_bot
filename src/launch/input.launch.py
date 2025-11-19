import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='user_input',
            executable='run_inputNode',
            name='inputNode',
            output='screen',
        ),
        Node(
            package='brain',
            executable='run_brainNode',
            name='brainNode',
            output='screen',
        )
    ])
