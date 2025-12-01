from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_markers',
            executable='ingredient_marker_node',
            name='ingredient_marker_node',
            output='screen'
        )
    ])
