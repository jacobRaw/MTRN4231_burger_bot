from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages
    ur_description = FindPackageShare("ur_description")
    ur_moveit_config = FindPackageShare("ur_moveit_config")

    # URDF xacro path for UR5e
    urdf_xacro_path = PathJoinSubstitution([
        ur_description,
        "urdf",
        "ur5e.urdf.xacro"
    ])

    # SRDF path for UR5e MoveIt config
    srdf_path = PathJoinSubstitution([
        ur_moveit_config,
        "config",
        "ur5e.srdf"
    ])

    # Build robot_description (URDF) at launch using xacro
    robot_description = {
        "robot_description": Command([
            TextSubstitution(text="xacro "),
            urdf_xacro_path,
            TextSubstitution(text=" "),
            TextSubstitution(text="ur_type:=ur5e "),
            TextSubstitution(text="prefix:= "),
            TextSubstitution(text="use_fake_hardware:=true "),
            TextSubstitution(text="initial_joint_controller:=joint_trajectory_controller "),
        ])
    }

    # Load SRDF file contents into robot_description_semantic
    robot_description_semantic = {
        "robot_description_semantic": Command([
            TextSubstitution(text="cat "),
            srdf_path
        ])
    }

    planning_server = Node(
        package="moveit_path_planner",
        executable="moveit_path_planning_server",
        name="moveit_path_planning_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return LaunchDescription([
        planning_server
    ])

import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
    
def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    planning_server = Node(
        package="moveit_path_planner",
        executable="moveit_path_planning_server",
        name="moveit_path_planning_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([planning_server])


# Calibrate the URDF from the real robot using which generates a calibration.yaml file
# Then run the ur_robot_driver to load the calibration to the robot