import os
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():    

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake = LaunchConfiguration("use_fake_hardware")

    declared_args = [
        # 192.168.0.100 real ip
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
    ]

    this_dir = os.path.dirname(__file__)
    #include the arm_server_launch file
    arm_launch_path = os.path.join(this_dir, "arm_server_launch.py")
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_launch_path),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake,
        }.items()
    )

    #include the brain node launch file
    brain_launch_path = os.path.join(this_dir, "brain.launch.py")
    brain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(brain_launch_path),
        launch_arguments={
            "use_fake_hardware": use_fake,
        }.items()
    )

    #include the user input launch file
    user_input_launch_path = os.path.join(this_dir, "input.launch.py")
    user_input_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(user_input_launch_path),
    )

    #include perception launch file
    perception_launch_path = os.path.join(this_dir, "perception.launch.py")
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch_path),
    )

    #include perception launch file
    return launch.LaunchDescription(
        declared_args + [
            arm_launch,
            brain_launch,
            user_input_launch,
            perception_launch,
        ]
    )