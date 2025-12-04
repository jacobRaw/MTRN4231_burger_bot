from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_controller',
            executable='gripper_serial_node',
            name='gripper_serial_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'}
            ]
        )
    ])