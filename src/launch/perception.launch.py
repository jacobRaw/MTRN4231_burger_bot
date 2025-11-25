import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense_pkg = FindPackageShare('realsense2_camera').find('realsense2_camera')
    realsense_launch = PythonLaunchDescriptionSource(
        [realsense_pkg, '/launch/rs_launch.py']
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

    return LaunchDescription([
        # DeclareLaunchArgument('static_broadcaster', default_value='shape1', description='Namespace for shape1 marker node'),
        # DeclareLaunchArgument('dynamic_broadcaster', default_value='shape2', description='Namespace for shape2 marker node'),
        realsense_activate,
        Node(
            package='perception',
            executable='yolo_vision',
            name='perceptionNode',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=[
                '0.61', '0.2305', '0.915',           # x y z
                '1.57', '1.57', '0',           # roll pitch yaw
                'base_link', 'camera_link'
            ]
        )
    ])
