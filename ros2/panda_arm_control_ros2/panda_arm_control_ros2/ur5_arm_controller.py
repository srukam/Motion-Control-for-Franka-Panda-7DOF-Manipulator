"""
ROS 2 port of src/ur5_arm_controller.py.

Run against either:
  - ros2 launch ur_robot_driver ur5.launch.py robot_ip:=192.168.1.15
    + ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5  (real)
  - ros2 launch ur_robot_driver ur5.launch.py use_fake_hardware:=true
    + ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5  (offline)

Critical UR5-specific differences vs. Panda:
  - Move group is "ur_manipulator" in Jazzy (not "manipulator" like ROS 1).
  - 6-DOF; UR5_READY_POSE is a singularity-free joint pose to escape the
    zero-pose shoulder singularity BEFORE any Cartesian motion.
  - Pendant must be in REMOTE control mode and the External Control program
    running, otherwise the driver silently drops trajectories.
"""
import copy
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor

from panda_arm_control_ros2.move_group_client import MoveGroupClient, offset_pose


UR5_GROUP = "ur_manipulator"  # NOTE: differs from ROS 1 ("manipulator")
UR5_BASE = "base_link"
UR5_EEF = "tool0"
UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Universal starter pose for UR3/UR5/UR10/UR5e/UR10e — matches the
# `up` named group_state defined in upstream ur_moveit_config's SRDF, so
# MoveIt will always know how to plan to/from it.
#
# Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
#   Pose: arm pointing up vertically, tool0 facing forward.
#
# Why this and not [0, -pi/2, pi/2, -pi/2, -pi/2, 0] (the previous):
#   The previous was the original Phase-1 Panda-style ready pose ported
#   over. This one is what the upstream UR maintainers ship as the
#   canonical starter — works on every UR variant identically.
UR5_READY_POSE = [
    0.0,
    -math.pi / 2.0,
    0.0,
    -math.pi / 2.0,
    0.0,
    0.0,
]


class UR5Controller(MoveGroupClient):
    def __init__(self):
        super().__init__(
            node_name="ur5_controller_node",
            group_name=UR5_GROUP,
            base_link=UR5_BASE,
            eef_link=UR5_EEF,
            joint_names=UR5_JOINTS,
        )

    def print_robot_info(self) -> None:
        joints = self.get_current_joint_values()
        pose = self.get_current_pose()
        log = self.get_logger()
        log.info(f"Group: {self.group_name}")
        log.info(f"Joints ({len(joints)}): " +
                 ", ".join(f"{j:.3f}" for j in joints))
        log.info(f"EEF link: {self.eef_link}")
        log.info(
            f"EEF pose: pos=({pose.position.x:.3f}, {pose.position.y:.3f}, "
            f"{pose.position.z:.3f})")

    def move_to_ready_pose(self, vel_scale: float = 0.2, acc_scale: float = 0.2) -> bool:
        self.get_logger().info("Moving to UR5_READY_POSE (joint space)")
        return self.move_to_joint_goal(UR5_READY_POSE, vel_scale, acc_scale)

    # ------------------------------------------------------------------
    # Behavior primitives
    # ------------------------------------------------------------------

    def behavior_reach(self, dx: float, dy: float, dz: float):
        self.get_logger().info(f"REACH dx={dx:.3f} dy={dy:.3f} dz={dz:.3f}")
        start = self.get_current_pose()
        target = offset_pose(start, dx, dy, dz)
        return self.cartesian_to([copy.deepcopy(start), target])

    def behavior_lift(self, height: float):
        self.get_logger().info(f"LIFT dz={height:.3f}")
        start = self.get_current_pose()
        target = offset_pose(start, dz=height)
        return self.cartesian_to([copy.deepcopy(start), target])

    def behavior_retreat(self, distance: float):
        self.get_logger().info(f"RETREAT dx=-{distance:.3f}")
        start = self.get_current_pose()
        target = offset_pose(start, dx=-distance)
        return self.cartesian_to([copy.deepcopy(start), target])

    def behavior_return(self, target_pose: Pose):
        self.get_logger().info("RETURN to stored pose")
        start = self.get_current_pose()
        return self.cartesian_to(
            [copy.deepcopy(start), copy.deepcopy(target_pose)])

    def run_behavior_sequence(self, behaviors) -> None:
        for i, b in enumerate(behaviors, start=1):
            kind = b[0]
            self.get_logger().info(f"--- behavior {i}/{len(behaviors)}: {b}")
            if kind == "reach":
                ok, _ = self.behavior_reach(b[1], b[2], b[3])
            elif kind == "lift":
                ok, _ = self.behavior_lift(b[1])
            elif kind == "retreat":
                ok, _ = self.behavior_retreat(b[1])
            elif kind == "return":
                ok, _ = self.behavior_return(b[1])
            elif kind == "joint":
                ok = self.move_to_joint_goal(b[1])
            elif kind == "hold":
                self.get_logger().info(f"HOLD {b[1]}s")
                time.sleep(float(b[1]))
                ok = True
            else:
                self.get_logger().error(f"Unknown behavior {kind}")
                ok = False

            if not ok:
                self.get_logger().error(
                    f"behavior {i} failed; retreating 3cm + skipping")
                self.behavior_retreat(0.03)


def main(args=None):
    rclpy.init(args=args)
    node: Optional[UR5Controller] = None
    executor: Optional[MultiThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None

    try:
        node = UR5Controller()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        node.get_logger().info(
            "Pendant must be in REMOTE mode with External Control program "
            "running. If not, trajectories will be silently dropped.")

        if not node.wait_for_servers(timeout_sec=30.0):
            node.get_logger().error("MoveIt didn't come up; aborting")
            return 1

        node.print_robot_info()

        # In headless mode the upstream UR driver's URScript can die between
        # launch and now (the trajectory controller goes "lifecycle ACTIVE
        # but internally not running"). Kick the URScript awake before any
        # motion attempt; wait a beat for the controller to come back.
        node.get_logger().info("Pre-flight: resend_robot_program + warm-up wait")
        node._ensure_controller_active()
        time.sleep(2.0)

        # Move to a UR-canonical starter pose (SRDF "up" group_state).
        if not node.move_to_ready_pose():
            node.get_logger().error(
                "Failed to reach ready pose. Likely the headless URScript "
                "is being rejected by the robot (look for 'C210A0' errors "
                "in the driver log). Recommended fix: set up the External "
                "Control URCap on the pendant (see launch file header).")
            return 1

        time.sleep(1.0)
        home_pose = node.get_current_pose()

        behaviors = [
            ("reach",   0.15,  0.0, -0.05),
            ("hold",    1.0),
            ("lift",    0.10),
            ("reach",   0.0,   0.15, 0.0),
            ("lift",   -0.08),
            ("hold",    1.0),
            ("retreat", 0.15),
            ("return",  home_pose),
        ]
        node.get_logger().info(f"Running {len(behaviors)} behaviors")
        node.run_behavior_sequence(behaviors)
        node.get_logger().info("Sequence complete")
        return 0

    except KeyboardInterrupt:
        return 130
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Fatal: {e}")
        else:
            print(f"Fatal before node init: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
