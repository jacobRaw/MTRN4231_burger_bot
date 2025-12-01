import launch
from launch.substitutions import PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    realsense_pkg = FindPackageShare('realsense2_camera').find('realsense2_camera')
    perception_marker_pkg = FindPackageShare('perception_markers').find('perception_markers')
    realsense_launch = PathJoinSubstitution(
        [realsense_pkg, 'launch', 'rs_launch.py']
    )

    perception_marker_launch = PathJoinSubstitution(
        [perception_marker_pkg, 'launch', 'display_markers.launch.py']
    )

    realsense_activate = IncludeLaunchDescription(
        realsense_launch,
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_depth': 'true',
            'enable_color': 'true',
            'pointcloud.enable': 'true',
            'enable_sync': 'true',
            'enable_rgbd': 'true',
            'pointcloud.ordered_pc': 'true',
        }.items()
    )

    perception_marker_activate = IncludeLaunchDescription(
        perception_marker_launch
    )

    delayed_transform = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub',
                arguments=[
                    '--x', '0.61', '--y', '0.2305', '--z', '0.915',
                    '--yaw', '0', '--pitch', '0', '--roll', #roll = axis, pitch = y, yaw = z
                    '3.14', '--frame-id', 'base_link', '--child-frame-id', 'camera_link'
                ]
            )
        ]
    )
    

    delayed_perception = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='perception',
                executable='yolo_vision',
                name='perceptionNode',
                emulate_tty=True,
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        # DeclareLaunchArgument('static_broadcaster', default_value='shape1', description='Namespace for shape1 marker node'),
        # DeclareLaunchArgument('dynamic_broadcaster', default_value='shape2', description='Namespace for shape2 marker node'),
        realsense_activate,
        delayed_transform,
        delayed_perception,
        perception_marker_activate
    ])

## Installing Package https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md