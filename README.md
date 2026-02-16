# Motion-Control-for-Franka-Panda-7DOF-Manipulator
# Autonomous Franka Panda Robot Control

Multi-phase robotics project implementing autonomous manipulation using ROS, MoveIt, and computer vision.

## Phase 1: Cartesian Motion Control ✅ Complete

Behavior-based motion controller with state machine autonomy.

**Features:**
- Cartesian trajectory planning with MoveIt
- Parametric behavior primitives (reach, lift, retreat)
- Finite state machine for sequencing
- Automatic trajectory retiming and validation
- Gazebo simulation integration

**Demo:**
```bash
roslaunch franka_panda phase1/panda_arm_controller_g.launch
roslaunch franka_panda phase1/moveit_real_execution.launch
python3 phase1/panda_init.py
```

## Phase 2A: Vision Integration 🚧 In Progress

RGB camera integration with OpenCV object detection.

## Phase 2B: Pick-and-Place 📋 Planned

Vision-guided autonomous manipulation.

## Technologies

ROS Noetic | MoveIt | Python 3 | Gazebo | OpenCV | Franka Emika Panda


