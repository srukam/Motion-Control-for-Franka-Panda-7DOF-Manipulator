"""
Shared base node for the Panda + UR5 ROS 2 controllers.

Wraps pymoveit2.MoveIt2 — handles the MoveGroup action, GetCartesianPath
service, ExecuteTrajectory action, FK service, and joint_state cache for us.
The behaviour surface (get_current_pose, move_to_joint_goal,
cartesian_to, etc.) is unchanged from the previous direct-action
implementation; only the internals are simpler.
"""
import copy
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

UR_RESEND_SERVICE = "/io_and_status_controller/resend_robot_program"


class MoveGroupClient(Node):
    """rclpy Node + pymoveit2.MoveIt2 wrapper."""

    def __init__(
        self,
        node_name: str,
        group_name: str,
        base_link: str,
        eef_link: str,
        joint_names: List[str],
    ):
        super().__init__(node_name)

        self.group_name = group_name
        self.base_link = base_link
        self.eef_link = eef_link
        self.joint_names = list(joint_names)

        self._cb_group = ReentrantCallbackGroup()
        self.moveit = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.eef_link,
            group_name=self.group_name,
            callback_group=self._cb_group,
        )

        # Self-heal client for UR headless mode. The upstream UR driver
        # auto-deactivates scaled_joint_trajectory_controller after RTDE
        # blips; calling /io_and_status_controller/resend_robot_program
        # re-uploads the URScript and re-activates the controller. The
        # service won't exist on Panda — calls become no-ops.
        self._resend_client = self.create_client(
            Trigger, UR_RESEND_SERVICE, callback_group=self._cb_group)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def wait_for_servers(self, timeout_sec: float = 30.0) -> bool:
        """Block until /joint_states is flowing (proxy for MoveIt being up)."""
        self.get_logger().info("Waiting for first /joint_states...")
        deadline = time.time() + timeout_sec
        while self.moveit.joint_state is None:
            if time.time() > deadline:
                self.get_logger().error("No /joint_states received")
                return False
            time.sleep(0.1)
        self.get_logger().info("MoveIt is up.")
        return True

    # ------------------------------------------------------------------
    # State accessors
    # ------------------------------------------------------------------

    def get_current_joint_values(self) -> List[float]:
        js = self.moveit.joint_state
        if js is None:
            raise RuntimeError("No /joint_states received yet")
        name_to_pos = dict(zip(js.name, js.position))
        try:
            return [float(name_to_pos[n]) for n in self.joint_names]
        except KeyError as e:
            raise RuntimeError(
                f"Joint {e} missing from /joint_states (got {list(js.name)})")

    # ------------------------------------------------------------------
    # UR headless self-heal
    # ------------------------------------------------------------------

    def _ensure_controller_active(self, timeout_sec: float = 10.0) -> None:
        """If UR's resend_robot_program service is up, call it. No-op otherwise.

        Timeout is generous (10 s) because the service handler does the
        actual URScript upload to the robot; on a stale stream that's been
        dropping packets, this can take several seconds before the driver
        either succeeds or gives up.
        """
        if not self._resend_client.service_is_ready():
            return  # service doesn't exist on this stack (Panda etc.)
        future = self._resend_client.call_async(Trigger.Request())
        deadline = time.time() + timeout_sec
        while not future.done():
            if time.time() > deadline:
                self.get_logger().warning(
                    f"resend_robot_program did not return within {timeout_sec}s; "
                    "continuing without confirmation")
                return
            time.sleep(0.02)
        try:
            resp = future.result()
            if resp is not None and not resp.success:
                self.get_logger().warning(
                    f"resend_robot_program reported failure: {resp.message!r}")
        except Exception:
            pass

    def get_current_pose(self) -> Pose:
        ps = self.moveit.compute_fk()
        if ps is None:
            raise RuntimeError("FK failed")
        if isinstance(ps, list):
            ps = ps[0]
        return ps.pose

    # ------------------------------------------------------------------
    # Motion: joint-space goal
    # ------------------------------------------------------------------

    def move_to_joint_goal(
        self,
        joint_values: List[float],
        vel_scale: float = 0.2,
        acc_scale: float = 0.2,
    ) -> bool:
        if len(joint_values) != len(self.joint_names):
            raise ValueError(
                f"Expected {len(self.joint_names)} joints, got {len(joint_values)}")

        self.moveit.max_velocity = float(vel_scale)
        self.moveit.max_acceleration = float(acc_scale)
        joints = [float(j) for j in joint_values]

        def _attempt() -> bool:
            self.moveit.move_to_configuration(joints)
            return bool(self.moveit.wait_until_executed())

        self._ensure_controller_active()
        ok = _attempt()
        if not ok:
            self.get_logger().warning(
                "Joint move aborted; resending URScript and retrying once")
            self._ensure_controller_active()
            time.sleep(2.0)
            ok = _attempt()
        return ok

    # ------------------------------------------------------------------
    # Motion: pose goal (single waypoint, free-space plan via OMPL)
    # ------------------------------------------------------------------

    def move_to_pose(
        self,
        target_pose: Pose,
        vel_scale: float = 0.1,
        acc_scale: float = 0.1,
        position_tolerance: float = 1.0e-3,
        orientation_tolerance: float = 1.0e-3,
    ) -> bool:
        self.moveit.max_velocity = float(vel_scale)
        self.moveit.max_acceleration = float(acc_scale)

        def _attempt() -> bool:
            self.moveit.move_to_pose(
                pose=target_pose,
                tolerance_position=position_tolerance,
                tolerance_orientation=orientation_tolerance,
                cartesian=False,
            )
            return bool(self.moveit.wait_until_executed())

        self._ensure_controller_active()
        ok = _attempt()
        if not ok:
            self.get_logger().warning(
                "Pose move aborted; resending URScript and retrying once")
            self._ensure_controller_active()
            time.sleep(2.0)
            ok = _attempt()
        return ok

    # ------------------------------------------------------------------
    # Motion: Cartesian path (linear EEF interpolation to target waypoint)
    # ------------------------------------------------------------------

    def cartesian_to(
        self,
        waypoints: List[Pose],
        eef_step: float = 0.01,
        jump_threshold: float = 0.0,
        vel_scale: float = 0.1,
        acc_scale: float = 0.1,
        avoid_collisions: bool = True,
        min_fraction: float = 0.9,
    ) -> Tuple[bool, float]:
        """
        Run /compute_cartesian_path then execute. pymoveit2 plans from the
        current state to the FINAL waypoint; intermediate waypoints in the
        list are ignored, which is fine for our 2-element [start, target]
        callers but worth noting.

        Returns (success, fraction). When pymoveit2 hides the fraction, we
        report 1.0 on success and 0.0 on failure.
        """
        if not waypoints:
            return (False, 0.0)
        target = waypoints[-1]

        self.moveit.max_velocity = float(vel_scale)
        self.moveit.max_acceleration = float(acc_scale)
        self.moveit.cartesian_avoid_collisions = bool(avoid_collisions)
        self.moveit.cartesian_jump_threshold = float(jump_threshold)

        def _attempt() -> bool:
            self.moveit.move_to_pose(
                pose=target,
                cartesian=True,
                cartesian_max_step=float(eef_step),
                cartesian_fraction_threshold=float(min_fraction),
            )
            return bool(self.moveit.wait_until_executed())

        self._ensure_controller_active()
        ok = _attempt()
        if not ok:
            self.get_logger().warning(
                "Cartesian move aborted; resending URScript and retrying once")
            self._ensure_controller_active()
            time.sleep(2.0)
            ok = _attempt()
        return (ok, 1.0 if ok else 0.0)


def offset_pose(base: Pose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
    """Return a copy of `base` with its position offset; orientation preserved."""
    p = copy.deepcopy(base)
    p.position.x = base.position.x + float(dx)
    p.position.y = base.position.y + float(dy)
    p.position.z = base.position.z + float(dz)
    return p
