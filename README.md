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

Render:
<img width="1191" height="839" alt="image" src="https://github.com/user-attachments/assets/76359ad7-43cc-491c-8dfb-574db24e3302" />
<img width="1317" height="761" alt="image" src="https://github.com/user-attachments/assets/d00f4ed7-9ddd-49fa-b6d7-1e94fd16da4d" />
<img width="1123" height="815" alt="image" src="https://github.com/user-attachments/assets/9abb99a5-1c40-49eb-b46d-8253dd56e5e4" />

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
- A short section outlining the folder structure of your repository.
- Explain briefly what each main directory contains.

## References and Acknowledgements
- Credit any external libraries, tutorials, or prior codebases.
- Acknowledge external assistance (e.g. demonstrators, other groups).
