# MTRN4231 Group Project - Burger Bot

## Table of Contents
- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Technical Components](#technical-components)
- [Installation and Setup](#installation-and-setup)
- [Running the System](#running-the-system)
- [Results and Demonstration](#results-and-demonstration)
- [Discussion and Future Work](#discussion-and-future-work)
- [Contributors and Roles](#contributors-and-roles)
- [Repository Structure](#repository-structure)
- [References and Acknowledgements](#references-and-acknowledgements)

## Project Overview

A short description of the task or problem your system solves, including the intended
“customer” or end-user

A summary of the robot’s functionality.

A short video (e.g. 10-30s) of the robot completing one full cycle/operation
demonstrating closed-loop behaviour and visualisation. (This could be embedded or an
external link to a YouTube/OneDrive/Google Drive video.)


## System Architecture
 - A diagram of your ROS2 nodes, topics, services and actions (e.g. from rqt_graph or a
custom schematic).
- A package-level architecture diagram showing node interactions and topics.
- A behaviour-tree or state-machine diagram showing closed-loop system behaviour.
- A brief description of the function of each node.
- Any custom message types or interfaces should be listed and explained.

## Technical Components
- Computer Vision: describe your vision pipeline and how it contributes to the task.
- Custom End-Effector: provide photos/renders, assembly details, engineering drawings,
control overview and integration details.

List of technical images and drawings for end effector:

CAD:
<img width="1229" height="959" alt="image" src="https://github.com/user-attachments/assets/3d45d045-4e3d-4c83-b05e-1836b60c4578" />

Renders:
<img width="1191" height="839" alt="image" src="https://github.com/user-attachments/assets/76359ad7-43cc-491c-8dfb-574db24e3302" />
<img width="1317" height="761" alt="image" src="https://github.com/user-attachments/assets/d00f4ed7-9ddd-49fa-b6d7-1e94fd16da4d" />
<img width="1123" height="815" alt="image" src="https://github.com/user-attachments/assets/9abb99a5-1c40-49eb-b46d-8253dd56e5e4" />
<img width="1061" height="1018" alt="image" src="https://github.com/user-attachments/assets/c7eae45d-c591-483d-873c-556fe3ee5537" />


Engineering Drawings:
<img width="1596" height="1125" alt="image" src="https://github.com/user-attachments/assets/9e936828-e793-4619-a5e1-f73c662aab24" />

Circuit Diagram:
<img width="1217" height="924" alt="image" src="https://github.com/user-attachments/assets/07222e7e-da9f-46ed-8312-8f5fafd0dfda" />




- System Visualisation: explain how your system is visualised (e.g. RViz) and what it
demonstrates.
- Closed-Loop Operation: describe the feedback method and how it adapts system behaviour in real time.

## Installation and Setup
- Step-by-step installation instructions for dependencies and workspace setup.
- Hardware setup information (UR5e connection, camera, Teensy, etc.).
- Any environment variables, configuration files, or calibration procedures required to run
the system. (You can take as assumed that there is some sort of hand-eye calibration
already present in the system.)
- Troubleshooting (common errors/mistakes)

## Running the System
- Clear instructions for launching and running the complete system.
- Example commands (e.g. ros2 launch project_name bringup.launch.py).
- Expected behaviour and example outputs.
- Optional troubleshooting notes.
- The system should be launched by a single command (e.g. via a launch file, shell script
or Docker image), without manual sequencing.

Computer vision (perception):
ros2 launch perception perception.launch.py 

This launch file will also launch the perception_markers package to publish object markers in RVIZ

Brain Node and user input:
ros2 launch brain input.launch.py

This launches both the brainNode and the inputNode

Moveit (arm control):
ros2 launch moveit_path_planner arm_server_launch.py

This launches the moveit action server

End Effector controller:
ros2 run 

## Results and Demonstration
- Describe how your system performs against its design goals.
- Include quantitative results where possible (e.g. accuracy, repeatability).
    - confidence of the YOLO
    - Record total number of attempts for a full cycle against successful attempts for repeatability
    - record of the speed to complete task
- Provide photos, figures, or videos showing the system in operation. (See Project
Overview, above.)
- Highlight robustness, adaptability, and innovation.

## Discussion and Future Work
- Briefly discuss major engineering challenges faced and how they were addressed.
- Outline opportunities for improvement or extensions – what would you do better for
“Version 2.0”.
    - Can talk about adding the mould to the fake food
- Summarise what makes your approach novel, creative, or particularly effective

## Contributors and Roles
Jacob Rawung (zID) - Jacobs primary areas of responsibility include computer vision using YOLO and developing overarching closed loop system control, with slight contribution to system visualisation.
Reynold Chu (zID) - Reynold is responsible for developing the moveit path planner as well as being the primary developer for system visualisation.
Riley Hackett (z5417561) - Riley is responsible for CAD + hardware development as well as developing the software to control hardware with ROS, with slight contribution to system visualisation.

- Briefly list team members and describe their primary areas of responsibility (e.g. vision,
planning, hardware).

## Repository Structure
MTRN4231_burger_bot/
├── README.md
├── requirements.txt
└── src
    ├── arduino_controller
    │   ├── CMakeLists.txt
    │   ├── NemaCont2
    │   │   └── NemaCont2.ino
    │   ├── package.xml
    │   └── src
    │       └── gripper_serial_node.cpp
    ├── brain
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    │       └── brainNode.cpp
    ├── burger_bot_cell
    │   ├── burger_bot_config
    │   │   ├── CMakeLists.txt
    │   │   ├── config
    │   │   │   ├── burger_bot_cell.ros2_control.xacro
    │   │   │   ├── burger_bot_cell.srdf
    │   │   │   ├── burger_bot_cell.urdf.xacro
    │   │   │   ├── initial_positions.yaml
    │   │   │   ├── joint_limits.yaml
    │   │   │   ├── kinematics.yaml
    │   │   │   ├── moveit_controllers.yaml
    │   │   │   ├── moveit.rviz
    │   │   │   ├── pilz_cartesian_limits.yaml
    │   │   │   └── sensors_3d.yaml
    │   │   ├── launch
    │   │   │   ├── demo.launch.py
    │   │   │   ├── move_group.launch.py
    │   │   │   ├── moveit_rviz.launch.py
    │   │   │   ├── rsp.launch.py
    │   │   │   ├── setup_assistant.launch.py
    │   │   │   ├── spawn_controllers.launch.py
    │   │   │   ├── static_virtual_joint_tfs.launch.py
    │   │   │   └── warehouse_db.launch.py
    │   │   └── package.xml
    │   ├── burger_bot_controller
    │   │   ├── CMakeLists.txt
    │   │   ├── config
    │   │   │   ├── burger_bot_calibration.yaml
    │   │   │   ├── my_robot_calibration.yaml
    │   │   │   ├── ur20
    │   │   │   │   ├── default_kinematics.yaml
    │   │   │   │   ├── joint_limits.yaml
    │   │   │   │   ├── physical_parameters.yaml
    │   │   │   │   └── visual_parameters.yaml
    │   │   │   └── ur5e
    │   │   │       ├── default_kinematics.yaml
    │   │   │       ├── joint_limits.yaml
    │   │   │       ├── physical_parameters.yaml
    │   │   │       └── visual_parameters.yaml
    │   │   ├── launch
    │   │   │   └── start_robot_launch.py
    │   │   ├── package.xml
    │   │   ├── rviz
    │   │   │   └── view_robot.rviz
    │   │   └── urdf
    │   │       └── burger_bot_controlled.urdf.xacro
    │   └── burger_bot_description
    │       ├── CMakeLists.txt
    │       ├── launch
    │       │   └── view_robot_launch.py
    │       ├── meshes
    │       │   ├── 171005_Motek_Monitor_L.dae
    │       │   ├── fzi_skyline_v1.png
    │       │   ├── fzi_table.dae
    │       │   ├── Gripper_fixed.stl
    │       │   ├── notexture.png
    │       │   ├── Tablev2_v13.stl
    │       │   ├── ur_background.jpg
    │       │   └── wall.dae
    │       ├── package.xml
    │       ├── rviz
    │       │   └── urdf.rviz
    │       └── urdf
    │           ├── burger_bot_macro.xacro
    │           └── burger_bot.urdf.xacro
    ├── custom_interfaces
    │   ├── action
    │   │   ├── Movement.action
    │   │   └── OrderRequest.action
    │   ├── CMakeLists.txt
    │   ├── msg
    │   │   ├── IngredientPos.msg
    │   │   └── Ingredients.msg
    │   ├── package.xml
    │   └── srv
    │       ├── GripperServer.srv
    │       ├── InputServer.srv
    │       └── MovementRequest.srv
    ├── launch
    │   ├── arm_server_launch.py
    │   ├── input.launch.py
    │   ├── perception.launch.py
    │   ├── __pycache__
    │   │   ├── ur_launch.cpython-310.pyc
    │   │   └── ur_startup.launch.cpython-310.pyc
    │   ├── system.launch.py
    │   └── ur_startup.launch.py
    ├── moveit_path_planner
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── planning_server.launch.py
    │   ├── package.xml
    │   └── src
    │       ├── moveit_path_planning_action.cpp
    │       └── moveit_path_planning_server.cpp
    ├── moveit_test
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── test.launch.py
    │   └── package.xml
    ├── perception
    │   ├── package.xml
    │   ├── perception
    │   │   ├── black_seed.pt
    │   │   ├── burger_model.pt
    │   │   ├── __init__.py
    │   │   └── yolo_vision.py
    │   ├── resource
    │   │   └── perception
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── test
    │       ├── test_copyright.py
    │       ├── test_flake8.py
    │       └── test_pep257.py
    ├── perception_markers
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── display_markers.launch.py
    │   ├── package.xml
    │   └── src
    │       └── ingredient_markers_node.cpp
    ├── robot_description
    │   ├── CMakeLists.txt
    │   ├── meshes
    │   │   ├── EndEffector.mtl
    │   │   ├── EndEffector.obj
    │   │   ├── Table.mtl
    │   │   └── Table.obj
    │   ├── package.xml
    │   └── urdf
    │       ├── burger_bot.xacro
    │       ├── end_effector.urdf
    │       └── table.urdf
    ├── Segmented Image with Centroid_screenshot_02.12.2025.png
    ├── user_input
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    │       └── inputNode.cpp
    └── visualisation
        ├── CMakeLists.txt
        └── package.xml

- A short section outlining the folder structure of your repository.
- Explain briefly what each main directory contains.

## References and Acknowledgements
- Credit any external libraries, tutorials, or prior codebases.
- Acknowledge external assistance (e.g. demonstrators, other groups).
