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
        # DeclareLaunchArgument('static_broadcaster', default_value='shape1', description='Namespace for shape1 marker node'),
        # DeclareLaunchArgument('dynamic_broadcaster', default_value='shape2', description='Namespace for shape2 marker node'),
        
        Node(
            package='user_input',
            executable='run_inputNode',
            name='inputNode',
            emulate_tty=True,
            output='screen',
        ),
    ])
