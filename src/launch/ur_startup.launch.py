from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # launch arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    # --- Paths to the launch files ---
    ur_driver_pkg = FindPackageShare('ur_robot_driver').find('ur_robot_driver')
    ur_moveit_pkg = FindPackageShare('ur_moveit_config').find('ur_moveit_config')

    ur_driver_launch = PathJoinSubstitution(
        [ur_driver_pkg, "launch", "ur_control.launch.py"]
    )

    ur_moveit_launch = PathJoinSubstitution(
        [ur_moveit_pkg, "launch", "ur_moveit.launch.py"]
    )

    # --- Include the UR driver launch ---
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',
        }.items()
    )

    # --- Include the MoveIt launch (after delay) ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': 'true',
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
        }.items()
    )

    # # --- Delay MoveIt launch by 10 seconds ---
    delayed_moveit = TimerAction(
        period=5.0,
        actions=[moveit_launch]
    )

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),

        # Actual launch actions
        driver_launch,
        delayed_moveit
    ])
