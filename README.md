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

### Diagram of ROS2 nodes, topics, services and actions (RQT Graph):
> These RQT graphs were taken at different points of operation for the solution and show the interaction between nodes as the solution runs.
<img width="1787" height="895" alt="RQT graph1" src="https://github.com/user-attachments/assets/290e8d6d-d17d-406d-88f6-189461412992" />
<img width="2078" height="501" alt="RQT graph2" src="https://github.com/user-attachments/assets/4a918ee2-831c-42b0-a4e1-b621dc801931" />


### Package-level architecture diagram showing node interactions and topics:
<img width="5789" height="6339" alt="Package Graph" src="https://github.com/user-attachments/assets/e112b9c2-bb3f-441b-a8db-46b1761d1cf8" />


### State-machine diagram which portrays the closed loop operation of the system:

<img width="1573" height="664" alt="State Diagram" src="https://github.com/user-attachments/assets/4d32728c-f26e-4c2d-9887-24d53b5c5966" />


### Description of the function of each node:

**1. Arduino_controller Package**
&rarr; gripper_serial_node: 

- Communicates with the teensy 4.1 over a serial port to control the end effector using open and close commands.

**2. Brain Package**
&rarr; brainNode: 

- Central node for the entire system coordinating everything. It:
    - Receives user input as a list of ingredients in the correct stack order
    - Reads perception outputs for available ingredient positions
    - Commands the UR5e arm to various positions to a follow strict trajectory
    - Sends open/close commands to the gripper
    - Handles edge-cases and overall closed loop operation with a state machine
    - Publishes location of collision objects to RVIZ for dynamic collision detection

**3. moveit_path_planner Package**
&rarr; moveit_path_planning_action: 

- Implements a ROS action server that provides asynchronous motion-planning to handle when the brain node requests a motion plan to a target pose
- Creates collision objects recieved from the brain node
- Interfaces with moveIt to generate collision free path planning

**4. perception Package**
&rarr; yolo_vision: 

- Runs object detection using YOLO and publishes the detected ingredient names and positions.

**5. perception_markers Package**
&rarr; ingredient_markers_node: 

- Subscribes to vision outputs and publishes the ingredients as markers for visualisation in Rviz.

**6. user_input Package**
&rarr; inputNode: 

- Captures user inputs for menu items and identifies the correct recipe.
- Sends the recipe as an ingredient list to the brain node.


### Custom message types and interfaces:

**1. Custom Messages:**

- IngredientPos.msg: Contains a particular ingredients ID (i.e. lettuce, tomato etc.) and its position in space (x, y, z).

- Ingredients.msg: An array of IngredientPos messages for all ingredients on the table.

**2. Custom Interfaces:**

- GripperServer.srv: Is used for communication with the end effector. Requests a command for the end effector “o”=open or “c”=close and responds with success state and a message.

- Movement.action: Is used for sending move commands to moveit. Requests a command, an array for the pose and a constraints identifier “0” = unconstrained, “1” = orientation constrained. Responds with a success state and has status feedback periodically.

- OrderRequest.action: Is used for inputting a desired menu item. Requests a string array of ingredient names and responds with a success state and has status feedback periodically.


## Technical Components

### Computer Vision 
- describe your vision pipeline and how it contributes to the task.
  
### Custom End-Effector
**Photos, Renders and Engineering Drawings**

- Photos:
<img width="625" height="391" alt="PhotoEndEffect" src="https://github.com/user-attachments/assets/e2d3ad23-8853-42a2-8bbb-b33f0fa38293" />


- Renders:
<img width="1317" height="761" alt="image" src="https://github.com/user-attachments/assets/d00f4ed7-9ddd-49fa-b6d7-1e94fd16da4d" />
<img width="1123" height="815" alt="image" src="https://github.com/user-attachments/assets/9abb99a5-1c40-49eb-b46d-8253dd56e5e4" />
<img width="1578" height="1017" alt="EF Render" src="https://github.com/user-attachments/assets/e2d6a0d9-fc88-4493-a6f4-96c419c2b4c3" />

- Engineering Drawings:
<img width="1574" height="1111" alt="EngDrawingVisible" src="https://github.com/user-attachments/assets/0b79e5d0-7679-4ff9-8ef9-bf5df09cbe85" />
<img width="1574" height="1107" alt="EngDrawingInvi" src="https://github.com/user-attachments/assets/f2b7b74f-2503-45e7-8336-60093db950dc" />


**Assembly Details** 

- Instructions:
  
      1. Before assembly you will need:
          - 5 M3x20, 6 M3x10, 4 M3x4, 1 M3x50 screws.
          - 1 M3 Wingnut.
          - 3 Bearings w/ 19OD & 6ID.
          - 2-part epoxy resin.
          - 600mm x 6.3mm solid aluminium or titanium round bar.
          - 5mm width timing belt replacement for 3D printer.
          - 0.6mm thick stainless steel sheet.
          - 4 9mm x 40mm low stiffness springs
          - 1 NEMA17 0.56Nm holding torque 1.7A 12V Stepper Motor
          - 1 DRV8825 or A4988 stepper motor driver
          - 1 100uF electrolytic capacitor
          - 1 Teensy 4.1 Microcontroller
  
      2. 3D print all parts which are not listed above.
  
      3. Cut aluminium rods into 2 293mm lengths and slide them into the two slots on the belt tensioner bracket.
  
      4. Slot the head of the M3x50 screw into the mount point on the coupling for the tensioner mechanism and slide onto the two rods from the unobstructed end.
  
      5. Slip the screw side of the M3x50 screw through the central hole on the belt tensioner bracket and fasten loosely with the M3 wingnut.
  
      6. Look for circular slots which fit the bearings on the 3D printed components and press fit them in place.
  
      7. With the bearings in place grab the timing belt drive gear with two male shafts and slot into the bearing within the tensioner coupling.
  
      8. Slide the large NEMA17 drive bracket onto the unobstructed side of the 2 guiderails, this should include a bearing.
  
      9. Slot the timing belt drive gear with one male shaft and one female d-profile into the bearing hole.
  
      10. Use the 4 M3x4 screws to fix the faceplate of the NEMA17 to its mounting bracket part.
  
      11. Measure a length of timing belt such that it loops around the two drive gears on opposite ends of the guiderails and meets itself without overlap or gap.
  
      12. Fix the mounting bracket holding the NEMA17 onto the drive bracket and slot the drive shaft through the d-profile in the belt drive gear. Fix this down with 2 M3x20 screws
  
      13. Similarly, slot the lid for the tensioner coupling in place and fasten with 2 M3x10 screws.
  
      14. Flip the entire structure upside-down and grab one of the top parts for the gripper jaws which should have a jagged profile matching the timing belt.
  
      15. Slot the free ends of the timing belt into this jagged profile and fix in place with a suspension sheath and 2 M3x10 screws.
  
      16. Slide this along the guide-rails until its flush with the NEMA drive bracket and tighten the wing nut to tension the timing belt.
  
      17. With the other suspension sheath and head, fix it in place such that it is flush with the tensioner coupling instead. Ensure the jagged profile is on the opposite side of the timing belt.
  
      18. Place the central UR5e connector on the guiderails and using both of the gripper heads, slide them into the center of the guiderails.
  
      19. Mix the 2 part epoxy resin and apply to the guiderails & inner surface of the central column, do not apply glue to the sliding gripper jaw heads.

      20. Press the suspension sheath for the holding pick into the epoxied central UR5e connector to fasten this column perfectly in the center of the guiderails.

      21. Once dry, place 2 springs into the central sheath and 1 spring into each of the jaw sheaths.

      22. Slide the holding pick in place and screw through the hole with 1 M3x20 screw to provide a blocker for the spring.

      23. Do a similar process for the two side jaws ensuring that the gripper cages face with the mouths inwards.

      24. Cut the stainless steel sheet with tin snips into 2 spatula shapes using a paper template to ensure consistency.

      25. Mix some more epoxy and apply to both the gripper cages and the spatulas and fix together while aligning the profiles.

      26. Once dry your end effector is complete.

      Next Steps:
      Follow the breadboard circuit diagram below and documentation for the particular motor driver you own to connect the NEMA17 to the Teensy 4.1.



      
- Exploded Render:
<img width="1529" height="1116" alt="ExplodedRender" src="https://github.com/user-attachments/assets/144b7d39-be0a-4d3e-87c2-f121c1c3149f" />


**Control Overview and ROS Integration Details**
- Control Overview:
  - The end effector is controlled by rotating the timing belt with a NEMA17 stepper motor. The gripper jaws are attached to opposite sides of the timing belt and guided by guide rails so that they can open and close by changing the direction of the stepper motor. The stepper motor is controlled by a teensy 4.1 microcontroller along with a DRV8825 stepper motor driver which handles heat dissipation and current limiting. The circuitry for this interaction can be seen below. The teensy is programmed to receive commands over a serial port with "o" referring to open and "c" referring to close. An open command will step the stepper motor 1 and 1/3rd rotations clockwise while a close command will do the opposite. This step value correlates to the amount of travel needed for the jaws to fully open and close. There is also functionality for handling unknown commands in which case nothing will occur. Importantly, when not receiving commands the stepper motor will hold its current state.
  
- ROS Integration:
  - The brain node sends service requests for opening and closing, the arduino_controller package is then used to handle these requests with service responses to the brain and sends the string "o" or "c" over the serial port /dev/ttyACM0 which represents the teensy serial port. The teensy then handles the serial input.

      
- Breadboard Circuit Diagram:
<img width="1217" height="924" alt="image" src="https://github.com/user-attachments/assets/07222e7e-da9f-46ed-8312-8f5fafd0dfda" />







### System Visualisation 
- explain how your system is visualised (e.g. RViz) and what it demonstrates.

- Table render for visualisation
<img width="1161" height="1015" alt="UR5e Table" src="https://github.com/user-attachments/assets/a4ca3fb6-6cdc-4276-8b8a-c7ffc7c1075e" />


### Closed-Loop Operation 
- describe the feedback method and how it adapts system behaviour in real time.

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

### Video Documentation
> Please see below a few video's of the system being tested for robustness in particular areas. The video in the project overview shows the fully working solution.

End Effector Test: https://youtu.be/JZK5y5WVghE

Computer Vision & Closed Loop Test: https://youtu.be/MXHHu9Mg7A8

Ingredient Collision Robustness Test: https://youtu.be/D7P5NxirLQY

### Evaluation of System Robustness, Adaptability, and Innovation

**Robustness**
- The solution exhibits many areas of robust operation. Namely, the ability for the moveit implementation to repeatedly move the UR5e to accurate coordinates and pose without unexpected behaviours is a major requirement and something that this solution does extremely well. 
Upon implementing the cartestian path planning method for moveit, any innacuracies or failures were completely mitigated. This greatly benefits robot safety with the safety planes now acting as backup precautions instead of the main constraint upon path planning. The current moveit
 implementation could very easily integrate into the desired fast-food industry and support accurate and safe path planning.

- Another strong area for this solution is the end effector itself, it provides significant repeatability, although it has been observed to fail on particularly rough parts of the UR5e table due to the spatula-grippers sliding along the table and getting caught on uneven surfaces or bolt holes. 
However, this is a relatively rare occurance and could be mitigated through further development and better manufacturing processes. The end effector is capable of manipulating any of the proposed ingredients, including extremly thin ingredient (~1mm thick) which was one of the major design requirements.

- The computer vision solution using YOLO works very well, by training the model on both the desired ingredients and false objects, it can now accurately identify ingredient types and positions as well as eliminate any non-ingredient from the selection. It updates extremely fast and exhibits high confidence levels. 
An area in which this solution can struggle is when the UR5e table is not made from darker wooden panels as this was the environment on which the YOLO model was trained. This yields lower confidence levels and some flickering behaviours, especially for smaller ingredients. The closed loop solution is capable of handling
this to an adequate extent, however, and will wait until the ingredient becomes available upon which the production will continue.

**Adaptability**
- The end effector is extremely adapatable and offers an easily scalable solution towards larger ingredients and a more food safe material selection (simply swap out the spatula grippers by unscrewing them). It also has the capability to adapt to errors within the solution through its suspension system which protects it from
crushing if it were to be forced into the UR5e table.

- The closed loop implementation offers a high level of adaptability to environmental changes, however, it can be slow to do so due to solution constraints involving the camera feed being blocked by the UR5e during the pick and place state. This means that updates to ingredient positions can only occur at the home state which
has been specifically chosen to provide an unobstructed view for the camera. Overall though, the major drawback is speed and the closed loop solution works for many foreseeable circumstances. This issue would likely need to be addressed and improved upon for industry integration. The closed loop implementation addresses a large number of edge cases which would otherwise cause failures or breaks to the system, some of which are clearly outlined in the demonstration video.

**Innovation**
- There are currently very few industry solutions for sandwich building robots due to the complexities of manipulating these sorts of objects. Burger ingredients have a high level of variability and so current methods often employ multi-stage, assembly-line type solutions with end-effectors only suitable to one or two ingredient
types per-robot on the line. This is an inneffective solution for fast food chains which often do not have a large amount of space to install such a system at high cost. The proposed end effector is designed to handle any forseeable burger ingredient, with scalability and ingredient care in mind. The end effector solution has proven to successfully contribute to this design area by fully assembling burgers with ingredients of varying diameter, thickness, rigidity and softness. However, wet ingredients are still yet to be tested even though its designed for it.

- Another area of innovation was the dynamically updating ingredient collision box implementation. This successfully solved a unique problem with the end effector design, in which, the spatulas can drop onto other ingredients during pick operation, thereby jamming the system or causing unpredictability. By applying tall collision boxes to all ingredients except for the target, the moveit path planning package can be used to return failed path plans if the end effector would crush an ingredient. The TCP can be rotated until a valid path plan is found (the end effector is clear below) and operation can continue. The solution to utilise innate path planning methods in moveit to solve an otherwise complex mathematical task adds to the innovation of this project.


## Discussion and Future Work
**Major Engineering Challenges and Solutions** 
- One of the largest challenges with the design of the end effector was ensuring that ingredients could be reliably released by the jaws at a repeatable location. Because the spatulas slide under the ingredients, when they re-open slight differences of friction and stickiness causes the ingredient to remain stuck on one side of the two gripper spatulas. This prevents reliable drop operation. To mitigate this, a centralised spring loaded holding pick featuring a studded base can be used to apply pressure from above while preventing lateral ingredient movement upon opening. This also applies force onto the burger stack to ensure ingredients are placed accurately and reliably. In addition, this implementation allows the end effector to be posed at any orientation, including fully upside down without ingredients slipping out.

- Moveit integration into the ROS solution posed a significant challenge with unrelaible path planning and unpredictable behaviours. The solution requires extremely accurate UR5e arm positioning to reliably stack burgers. The moveit issues were completely mitigated through using cartesian path planning which provides a reliable solution to this engineering problem.

- As discussed in the innovation section, there was an issue of ingredients being below the end effector as it lowered, if the jaws collided with these ingredients unintentionally it would cause significant failures to the solution. This was mitigated through adding collision boxes to all ingredients except the target to generate an accurate ingredient avoidance path planning solution.

- The resolution of the RGB-D camera was too minimal to allow for the side mounted computer vision implmentation. Because some ingredients are exceedingly small (pickles) the YOLO model could not accurately identify these ingredients from a far distance. To remedy this issue the camera was placed from above, giving a clearer view of the UR5e table. The camera mount was developed such that the camera was aligned with the centroid of the work area defined by the 10 table bolt holes.

**Future Improvements**
- Adding mold detection to YOLO and the ability to drop bad ingredients into a bin location. 
- The ability to change the stack location using aruco markers. 
- Adding a graphical user interface instead of just using the CLI. 
- Better smoothing of end effector spatulas to reduce the chance of them getting stuck.
- More efficient stacking by stacking from top to bottom and keeping the stack in the end effector. This removes having to go back to the stack location each time and instead the gripper can just jump between ingredients.

**Solution Novelty**
- The solutions novelty is mainly covered in the innovation section, however as an overview, the ability to delicately pick up soft objects is an interesting problem in robotics which is often solved through using pneumatic gripper solutions. A more classic take on gripper design with 2 jaws, integrated with the functionality to lift soft ingredients allows for an innovative approach to this design problem. The necessary collision planning that is needed to support this solution is not often faced by other pick and place systems and these unique qualities make the relatively aggressive Burger Master a novel and effective solution within the delicate robotics space.

## Contributors and Roles
**Jacob Rawung (z5406297) - Software/Control Engineer**
- Trained the YOLO model across two iterations
- Developed the brain node, user input and perception, key highlights:
    - Dynamic collision object generation
    - State machine with control feedback from perception
- Assisted in the development and integration with wider system for visualisation markers, arduino controller node and moveit path planner
- Maintained meeting minutes and managed task deadlines

**Reynold Chu (zID) - Mechatronics Engineer** - Reynold is responsible for developing the moveit path planner as well as being the primary developer for system visualisation.

**Riley Hackett (z5417561) - Mechanical/Electrical Engineer & Investor** 
- Responsible for all CAD and end-effector development/assembly, Key areas:
    - Developed timing belt solution, belt tensioning mechanism and gripper suspension.
    - Fully modelled UR5e table for visualisation.
    - Produced a work-space centred RGB-D camera mount.
- Developed the arduino software for end-effector control as well as breadboard wiring.
- Sourced and funded all components and manufacturing for final end-effector solution and prototypes.
- Produced the pre-integration versions of the perception_markers and arduino_controller packages.


## Repository Structure
All the source code is kept inside the src directory whilst the readme and requirements file is left at the root to for initial setup. Inside the src directory each directory is a ROS package that contains at most one ROS node. Some directories are unique such as custom_interfaces, launch and robot_description directories which do not contain a ROS node but contain crucial components that ROS nodes depend on.
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
    │   ├── system.launch.py
    │   └── ur_startup.launch.py
    ├── moveit_path_planner
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── planning_server.launch.py
    │   ├── package.xml
    │   └── src
    │       └── moveit_path_planning_action.cpp
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
    │   └── setup.py
    │   
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

### Arduino Controller
```
arduino_controller
├── CMakeLists.txt
├── NemaCont2
│   └── NemaCont2.ino
├── package.xml
└── src
└── gripper_serial_node.cpp
```
This package is reponsible for communicating to the teensy microcontroller over the serial port. It also contains the program that is run on the teensy to control the end effector. The node is interacted with by the brain via a service call which sends either 'o' or 'c' corresponding to either open or close.

### Brain
Contains the brain node source code that is responsible for interacting with the entire system, coordinating different componets.

### Custom Interfaces
```
custom_interfaces
├── action
│   ├── Movement.action
│   └── OrderRequest.action
├── CMakeLists.txt
├── msg
│   ├── IngredientPos.msg
│   └── Ingredients.msg
├── package.xml
└── srv
    ├── GripperServer.srv
    ├── InputServer.srv
    └── MovementRequest.srv
```
Custom interfaces contains all of the custom service, action and message types to allow for internode communication in ROS2.

### Launch
```
launch
├── arm_server_launch.py
├── input.launch.py
├── perception.launch.py
├── system.launch.py
└── ur_startup.launch.py
```
Launch directory contains all the necesssary launch files that are required to run the system.

### Moveit Path Planner
```
moveit_path_planner
├── CMakeLists.txt
├── launch
│   └── planning_server.launch.py
├── package.xml
└── src
    └── moveit_path_planning_action.cpp
```
Contains the source code for interacting with moveit and establishing an action server for the brain node to communicate with.

### Perception
```
perception
├── package.xml
├── perception
│   ├── black_seed.pt
│   ├── burger_model.pt
│   ├── __init__.py
│   └── yolo_vision.py
├── resource
│   └── perception
├── setup.cfg
└── setup.py
```
Contains the ROS node that implements YOLO and the two models that were trained. The most up to date model is black_seed.pt and whilst the other is the first iteration trained model.

### Perception Markers
```
perception_markers
├── CMakeLists.txt
├── launch
│   └── display_markers.launch.py
├── package.xml
└── src
    └── ingredient_markers_node.cpp
```
Contains the source code to publish the ingredients as RVIZ markers so that they are visible in the scene with the robot.

### Robot Description
```
robot_description
├── CMakeLists.txt
├── meshes
│   ├── EndEffector.mtl
│   ├── EndEffector.obj
│   ├── Table.mtl
│   └── Table.obj
├── package.xml
└── urdf
    ├── burger_bot.xacro
    ├── end_effector.urdf
    └── table.urdf
```
Contains all the meshes, URDF and XACRO files for the table and end effector for visualisation in RVIZ. The meshes were generated from the CAD models.

### User Input
```
user_input
├── CMakeLists.txt
├── package.xml
└── src
    └── inputNode.cpp
```
Contains source code for the user input node responsible for interacting with the user.

## References and Acknowledgements
- Credit any external libraries, tutorials, or prior codebases.
- Acknowledge external assistance (e.g. demonstrators, other groups).

We would like to acknowledge the other teams that undertook the course and was able to assist us
with the issues experienced when programming the arm with MoveIt.
Demonstrators (Alex Cronin and David Nie) and the lecturer (Dr Will Midgley) have all provided their personal experience, tips and and technical knowledge for debugging and learning how to use ROS2. 
