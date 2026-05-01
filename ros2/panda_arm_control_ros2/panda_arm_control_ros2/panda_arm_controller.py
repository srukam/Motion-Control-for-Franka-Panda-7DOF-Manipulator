"""
ROS 2 port of src/panda_arm_controller.py.

Run against either:
  - moveit_resources_panda_moveit_config/launch/demo.launch.py  (offline)
  - the franka_ros2 driver bringup                              (real robot)

Either way, ros2 run panda_arm_control_ros2 panda_arm_controller will pick
up the running /move_action server and execute the same behavior sequence
the ROS 1 version did.
"""
import copy
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor

from panda_arm_control_ros2.move_group_client import MoveGroupClient, offset_pose


PANDA_GROUP = "panda_arm"
PANDA_BASE = "panda_link0"
PANDA_EEF = "panda_link8"
PANDA_JOINTS = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
]


# Behavior tuple kinds: ("reach", dx, dy, dz), ("lift", dz), ("retreat", d),
# ("return", Pose), ("hold", seconds), ("joint", [j1..j7])
class PandaController(MoveGroupClient):
    def __init__(self):
        super().__init__(
            node_name="panda_controller_node",
            group_name=PANDA_GROUP,
            base_link=PANDA_BASE,
            eef_link=PANDA_EEF,
            joint_names=PANDA_JOINTS,
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
            f"{pose.position.z:.3f}) "
            f"quat=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, "
            f"{pose.orientation.z:.3f}, {pose.orientation.w:.3f})")

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
    node: Optional[PandaController] = None
    executor: Optional[MultiThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None

    try:
        node = PandaController()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if not node.wait_for_servers(timeout_sec=30.0):
            node.get_logger().error("MoveIt didn't come up; aborting")
            return 1

        node.print_robot_info()
        time.sleep(1.0)
        start_pose = node.get_current_pose()

        behaviors = [
            ("reach",   0.20,  0.0, -0.05),  # approach
            ("hold",    1.0),                 # grasp
            ("lift",    0.12),                # lift
            ("reach",   0.0,   0.15, 0.0),    # side
            ("lift",   -0.08),                # lower
            ("hold",    1.0),                 # release
            ("retreat", 0.20),                # clear
            ("return",  start_pose),          # home
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
