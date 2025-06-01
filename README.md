# Five DOF Robotic Arm - CAD, URDF files and MoveIt! Configuration

This project provides a complete CAD models, MoveIt! configuration and control setup for a custom 5 Degrees-of-Freedom (DOF) robotic arm using ROS Noetic. It includes URDF modeling, MoveIt! motion planning, RViz visualization, and a basic pick-and-place demo script.

---

##  Features

- 5-DOF articulated robotic arm modeled in URDF
- Integrated with MoveIt! for motion planning
- RViz visualization of planning scenes
- Python script to perform basic pick-and-place operations
- Launchable demo environment for testing configurations

---

## Prerequisites

Make sure your system is set up with:

- Ubuntu 20.04
- ROS Noetic
- MoveIt! (`ros-noetic-moveit`)
- Python 3
- A working catkin workspace

Install MoveIt! if needed: sudo apt install ros-noetic-moveit

---

## Installation

  -cd ~/catkin_ws/src  
  git clone https://github.com/YOUR_USERNAME/five_dof_arm_moveit.git  
  cd ~/catkin_ws  
  catkin_make  
  source devel/setup.bash  

  ---
  ## Folder Structure
  five_dof_arm_moveit/  
├── config/                     # MoveIt! config files  
├── launch/                     # Launch files  
├── urdf/                       # Robot URDF file  
├── scripts/                    # Python control scripts  
│   └── pick_place.py  
├── CMakeLists.txt  
├── package.xml  
└── README.md  

  
  ---
  ## Usage

  --roslaunch five_dof_arm_moveit_config demo.launch  
  --rosrun five_dof_arm_control pick_and_place.py  

  

Author
Tatsat  
Machinist | Mechatronics and Mechanical Engineer
tatsat2409dave@gmail.com


```bash
