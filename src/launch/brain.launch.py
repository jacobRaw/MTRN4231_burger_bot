from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Declare them so user can override
    declared_args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
    ]

    return LaunchDescription([
        Node(
            package='brain',
            executable='run_brainNode',
            name='brainNode',
            output='screen',
            parameters=[
                {'use_fake_hardware': use_fake_hardware}
            ]
        )
    ])
