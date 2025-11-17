from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # --- Paths to the launch files ---
    ur_driver_pkg = FindPackageShare('ur_robot_driver').find('ur_robot_driver')
    # ur_moveit_pkg = FindPackageShare('ur_moveit_config').find('ur_moveit_config')
    burger_bot_urdf = FindPackageShare('burger_bot_controller').find('burger_bot_controller')

    my_description_file = PathJoinSubstitution(
        [burger_bot_urdf, 'urdf', 'burger_bot_controlled.urdf.xacro']
    )


    ur_driver_launch = PythonLaunchDescriptionSource(
        [ur_driver_pkg, '/launch/ur_control.launch.py']
    )

    # ur_moveit_launch = PythonLaunchDescriptionSource(
    #     [ur_moveit_pkg, '/launch/ur_moveit.launch.py']
    # )

    # --- Include the UR driver launch ---
    driver_launch = IncludeLaunchDescription(
        ur_driver_launch,
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.0.100',
            'use_fake_hardware': 'false',
            'launch_rviz': 'true',
            # 'description_file': my_description_file,
            # 'launch_io': 'false'
        }.items()
    )

    # --- Include the MoveIt launch (after delay) ---
    # moveit_launch = IncludeLaunchDescription(
    #     ur_moveit_launch,
    #     launch_arguments={
    #         'ur_type': 'ur5e',
    #         'robot_ip': '192.168.0.100',
    #         'launch_rviz': 'true'
    #     }.items()
    # )

    # # --- Delay MoveIt launch by 10 seconds ---
    # delayed_moveit = TimerAction(
    #     period=10.0,
    #     actions=[moveit_launch]
    # )

    return LaunchDescription([
        driver_launch,
        # delayed_moveit
    ])
