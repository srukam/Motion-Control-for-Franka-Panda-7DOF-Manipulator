# Motion-Control-for-Franka-Panda-7DOF-Manipulator
# Autonomous Franka Panda Robot Control

Multi-phase robotics project implementing autonomous manipulation using ROS, MoveIt, and computer vision.

[](https://github.com/srukam/Motion-Control-for-Franka-Panda-7DOF-Manipulator/Videos/Panda_arm_controller_phase1.gif)

## Phase 1: Cartesian Motion Control  Complete

Behavior-based motion controller with state machine autonomy.

**Features:**
- Cartesian trajectory planning with MoveIt
- Parametric behavior primitives (reach, lift, retreat)
- Finite state machine for sequencing
- Automatic trajectory retiming and validation
- Gazebo simulation integration

**Demo:**
```bash
roslaunch Motion-Control-for-Franka-Panda-7DOF-Manipulator panda_arm_controller_g.launch
roslaunch Motion-Control-for-Franka-Panda-7DOF-Manipulator moveit_real_execution.launch
rosrun Motion-Control-for-Franka-Panda-7DOF-Manipulator panda_arm_controller.py
```

## Technologies

ROS Noetic | MoveIt | Python 3 | Gazebo | OpenCV | Franka Emika Panda

# TODO

## Phase 2A: Vision Integration - In Progress

RGB camera integration with OpenCV object detection.

## Phase 2B: Pick-and-Place  Planned

Vision-guided autonomous manipulation.

