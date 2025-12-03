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

The burger bot project aims to address the task of automating the fast-food industry by providing a burger assembly robotic manipulator which is more cost effective than current industry solutions. Employees are expensive, unreliable and take time to train. These are costs that can be mitigated through automation because a robotic solution to burger assembly is mostly a onetime cost bar electricity and periodic maintenance. These recurring expenses, however, are much lower than a team of employees’ wages. A robotic solution is consistent, accurate, efficient and can work 24/7. These benefits aim to aid fast food restaurants such as McDonalds, Hungry Jack’s, Wendy’s and any other burger restaurant which would like to enter business negotiations.

Burger and sandwich assembly is a significant technical issue faced by industry with minimal solutions currently available. Many require assembly lines with multiple robotic manipulators with end effectors specialised to one or two ingredients at a time. This inflates cost for restaurants who are fed up with their unreliable employees and looking for a robotic solution. This project proposes an end effector which can manipulate any foreseeable burger ingredient, reducing the assembly line to a singular robot per burger.

The solution integrates computer vision, path planning, a custom-built end effector and closed-loop control to solve this problem. As a general overview of functionality the below operation pipeline explains the solution.

- The functionality starts from the end-user who inputs their desired menu item. This has information about the correct ingredients and the order for stacking and is fed into the brain. 
- Within the restaurant, ingredients can either be fed into the robot’s range via conveyor belt in any configuration but in this simplified case they are situated randomly on a table within range of an RGB-D camera. 
- The solution uses a YOLO model for its computer vision trained on all possible ingredients + false ingredients to improve robustness in detection. By running the YOLO model on the camera feed the pixel locations of each ingredient’s centroid can be found as well as what ingredient is being detected. This is then transformed into coordinates relative to the UR5e’s base-link by comparing the locations of the camera and the UR5e base.
- The brain uses this information to determine the location of the first item in the stack for the end-users desired menu item and calls moveit utilising cartesian path planning to go to that location.
- Once the gripper is lowered over the ingredient, the brain calls the Arduino control node to send a close command over serial to the Teensy 4.1 which controls the end-effector.
- Moveit is then called again to send the TCP back to the defined home location and releases the gripper to add the ingredient to the stack.
- This is repeated for every ingredient in the menu item, with the locations of food items being dynamically updated during operation.
- During the entire operation every movement is visualised within Rviz, along with safety planes, environment objects and collision boxes.
The solution includes extensive error checking, but this is covered more in-depth within the system architecture and technical components sections.

### Solution Video:

Please see below a video of the manipulator completing a full control loop. This video includes a representation of many of the possible edge cases and displays how this solutions brain node is robust enough to handle them.

- 0:00:04 - Functionality for ensuring the end effector grippers do not crush other “non-target” ingredients on the table as it lowers by rotating the TCP.
- 0:00:18 - Safety plane integration with path planning to ensure ingredients are in a pickup able location without colliding with safety planes (Front).
- 0:00:23 - Singularity prevention for when ingredients are out of reach, integrated into closed loop control to update target location when ingredient is moved within suitable range.
- 0:00:31 - Recipe adherence, the robot will only pick up the necessary ingredients on the table to follow an inputted recipe.

**High quality video (YouTube):** https://youtu.be/2kIR_RqGSEc

https://github.com/user-attachments/assets/b9df1b45-9737-490c-970d-db6d626eccda


## System Architecture

### Diagram of ROS2 nodes, topics, services and actions:

### Package-level architecture diagram showing node interactions and topics:


### State-machine diagram which portrays the closed loop operation of the system:

<img width="1573" height="664" alt="State Diagram" src="https://github.com/user-attachments/assets/4d32728c-f26e-4c2d-9887-24d53b5c5966" />


### Description of the function of each node:

**1. Arduino_controller Package**
&rarr; gripper_serial_node: 

- Communicates with the teensy 4.1 over a serial port to control the end effector using open and close commands.

**2. Brain Package**
&rarr; brainNode: 

- The central node for the system coordinating everything. It:
    - Receives user input and converts it into burger stack orders
    - Reads perception outputs for available ingredient positions
    - Chooses UR5e motions based on vision outputs
    - Requests moveit to generate path planning
    - Sends open/close commands to the gripper
    - Handles edge-cases and overall closed loop operation

**3. moveit_path_planner Package**
&rarr; moveit_path_planning_action: 

- Implements a ROS action server that provides asynchronous motion-planning to handle when the brain node requests a motion plan to a target pose.

**4. perception Package**
&rarr; yolo_vision: 

- Runs object detection using YOLO and publishes the detected ingredient names and positions.

**5. perception_markers Package**
&rarr; ingredient_markers_node: 

- Subscribes to vision outputs and publishes the ingredients as markers for visualisation in Rviz.

**6. user_input Package**
&rarr; inputNode: 

- Captures user inputs for menu items and sends them to the brain node


### Custom message types and interfaces:

**1. Custom Messages:**

&rarr; IngredientPos.msg: 

- Contains a particular ingredients ID (i.e. lettuce, tomato etc.) and its position in space (x, y, z).

&rarr; Ingredients.msg:

- An array of IngredientPos messages for all ingredients on the table.

**2. Custom Interfaces:**

&rarr; GripperServer.srv:

- Is used for communication with the end effector. Requests a command for the end effector “o”=open or “c”=close and responds with success state and a message.

&rarr; Movement.action:

- Is used for sending move commands to moveit. Requests a command, an array for the pose and a constraints identifier “0” = unconstrained, “1” = orientation constrained. Responds with a success state and has status feedback.




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
```
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
    └── user_input
        ├── CMakeLists.txt
        ├── package.xml
        └── src
            └── inputNode.cpp
```

- A short section outlining the folder structure of your repository.
- Explain briefly what each main directory contains.

## References and Acknowledgements
- Credit any external libraries, tutorials, or prior codebases.
- Acknowledge external assistance (e.g. demonstrators, other groups).
