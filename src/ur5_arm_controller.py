#!/usr/bin/python3
"""
UR5 motion controller, mirroring the structure of panda_arm_controller.py.

Differences from the Panda version that matter:
  - Move group is "manipulator" (UR convention), 6-DOF.
  - No gripper actions.
  - The UR5 zero-joint pose is at the shoulder singularity, so the controller
    moves to a safe joint-space "ready" pose BEFORE any Cartesian motion.
  - Default execution uses scaled_pos_joint_traj_controller; the pendant speed
    slider further scales whatever vel_scale we send. Keep slider at 100%.
"""
import math
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

# State definitions
IDLE = 0
MOVING = 1
HOLDING = 2
ERROR = 3

# Safe joint-space "ready" pose for the UR5. Elbow bent, EEF roughly forward,
# clear of the shoulder/elbow singularities so Cartesian planning has room.
# Order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
UR5_READY_POSE = [
    0.0,
    -math.pi / 2.0,
     math.pi / 2.0,
    -math.pi / 2.0,
    -math.pi / 2.0,
    0.0,
]


class UR5Controller:
    """
    Reusable controller for a real UR5 in remote mode (External Control URCap).
    Wraps MoveIt's MoveGroupCommander for the "manipulator" group.
    """

    def __init__(self, move_group_name="manipulator"):
        print("\n" + "=" * 60)
        print("INITIALIZING UR5 CONTROLLER")
        print("=" * 60)
        print("[INIT] Pendant must be in REMOTE mode with the External")
        print("       Control program running, otherwise trajectories")
        print("       will be silently dropped by the driver.")

        moveit_commander.roscpp_initialize([])
        rospy.init_node("ur5_controller_node", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(move_group_name)

        # Allow joint_states subscriber to catch up before we read state.
        rospy.sleep(2.0)

        self.eef_link = self.group.get_end_effector_link()

        print("\n[INIT] UR5 Controller initialized.")
        self.print_robot_info()

    def print_robot_info(self):
        print("\n--- Robot Information ---")
        print("Move group:  ", self.group.get_name())
        print("Joint names: ", self.group.get_active_joints())
        print("Current joints:", self.group.get_current_joint_values())
        print("EEF link:    ", self.eef_link)
        cur = self.group.get_current_pose(self.eef_link).pose
        print("Current EEF pose:")
        print("  Position:    x={:.3f}, y={:.3f}, z={:.3f}".format(
            cur.position.x, cur.position.y, cur.position.z))
        print("  Orientation: x={:.3f}, y={:.3f}, z={:.3f}, w={:.3f}".format(
            cur.orientation.x, cur.orientation.y,
            cur.orientation.z, cur.orientation.w))
        print("-" * 60)

    # ------------------------------------------------------------------
    # Trajectory helpers
    # ------------------------------------------------------------------

    def fix_time_monotonic(self, traj):
        """Ensure trajectory points have strictly increasing time_from_start."""
        last_time = 0.0
        dt = 0.03  # 30 ms minimum step
        for pt in traj.joint_trajectory.points:
            t = pt.time_from_start.to_sec()
            if t <= last_time:
                t = last_time + dt
            pt.time_from_start = rospy.Duration(t)
            last_time = t
        return traj

    def get_current_pose(self):
        return self.group.get_current_pose(self.eef_link).pose

    def get_current_joints(self):
        return self.group.get_current_joint_values()

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def move_to_joint_pose(self, joint_values, vel_scale=0.2, acc_scale=0.2):
        """
        Joint-space move. Use this before Cartesian motion to escape the
        UR5 zero-pose singularity. joint_values must be 6 floats (radians).
        """
        if len(joint_values) != 6:
            raise ValueError("UR5 expects 6 joint values, got {}".format(
                len(joint_values)))

        print("\n" + "=" * 60)
        print("BEHAVIOR: MOVE TO JOINT POSE")
        print("=" * 60)
        print("Target joints:", ["{:.3f}".format(j) for j in joint_values])

        self.group.set_max_velocity_scaling_factor(vel_scale)
        self.group.set_max_acceleration_scaling_factor(acc_scale)
        self.group.set_joint_value_target(list(joint_values))

        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if success:
            print("[JOINT MOVE] Reached target.")
        else:
            print("[JOINT MOVE] FAILED.")
        return success

    def move_to_ready_pose(self, vel_scale=0.2, acc_scale=0.2):
        """Joint-space move to UR5_READY_POSE — safe pre-Cartesian start."""
        return self.move_to_joint_pose(UR5_READY_POSE, vel_scale, acc_scale)

    def execute_cartesian_path(self, waypoints, vel_scale=0.1, acc_scale=0.1,
                               step_size=0.01, jump_threshold=0.0):
        """
        Plan and execute a Cartesian path. jump_threshold=0.0 disables the
        wrist-flip guard, matching the Panda controller. If you see the
        UR5 take long swings between waypoints in joint space, set this to
        ~5.0 to abort plans that contain a wrist flip.
        """
        print("\n[EXECUTE] Planning Cartesian path with {} waypoints...".format(
            len(waypoints)))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, step_size, jump_threshold)
        print("[EXECUTE] Planned fraction: {:.2%}".format(fraction))

        if fraction < 0.90:
            print("[EXECUTE] WARNING: incomplete path (fraction < 90%)")
            return (False, fraction)

        start_state = self.group.get_current_state()
        plan = self.group.retime_trajectory(
            start_state, plan,
            velocity_scaling_factor=vel_scale,
            acceleration_scaling_factor=acc_scale)
        plan = self.fix_time_monotonic(plan)

        self.group.stop()
        self.group.clear_pose_targets()
        plan.joint_trajectory.header.stamp = rospy.Time.now()

        if len(plan.joint_trajectory.points) == 0:
            print("[EXECUTE] Plan has no points!")
            return (False, 0.0)

        print("[EXECUTE] Executing {} points...".format(
            len(plan.joint_trajectory.points)))
        success = self.group.execute(plan, wait=True)
        print("[EXECUTE] {} ".format("OK" if success else "FAIL"))
        return (success, fraction)

    def behavior_reach(self, x_offset, y_offset, z_offset):
        print("\n" + "=" * 60)
        print("BEHAVIOR: REACH  dx={:.3f} dy={:.3f} dz={:.3f}".format(
            x_offset, y_offset, z_offset))
        print("=" * 60)

        start_pose = self.get_current_pose()
        target_pose = Pose()
        target_pose.position.x = start_pose.position.x + x_offset
        target_pose.position.y = start_pose.position.y + y_offset
        target_pose.position.z = start_pose.position.z + z_offset
        target_pose.orientation = start_pose.orientation
        return self.execute_cartesian_path(
            [copy.deepcopy(start_pose), target_pose])

    def behavior_lift(self, height):
        print("\n" + "=" * 60)
        print("BEHAVIOR: LIFT  dz={:.3f}".format(height))
        print("=" * 60)

        start_pose = self.get_current_pose()
        lift_pose = Pose()
        lift_pose.position.x = start_pose.position.x
        lift_pose.position.y = start_pose.position.y
        lift_pose.position.z = start_pose.position.z + height
        lift_pose.orientation = start_pose.orientation
        return self.execute_cartesian_path(
            [copy.deepcopy(start_pose), lift_pose])

    def behavior_retreat(self, distance):
        print("\n" + "=" * 60)
        print("BEHAVIOR: RETREAT  dx=-{:.3f}".format(distance))
        print("=" * 60)

        start_pose = self.get_current_pose()
        retreat_pose = Pose()
        retreat_pose.position.x = start_pose.position.x - distance
        retreat_pose.position.y = start_pose.position.y
        retreat_pose.position.z = start_pose.position.z
        retreat_pose.orientation = start_pose.orientation
        return self.execute_cartesian_path(
            [copy.deepcopy(start_pose), retreat_pose])

    def behavior_return_to_pose(self, target_pose):
        print("\n" + "=" * 60)
        print("BEHAVIOR: RETURN TO POSE")
        print("=" * 60)
        start_pose = self.get_current_pose()
        return self.execute_cartesian_path(
            [copy.deepcopy(start_pose), copy.deepcopy(target_pose)])

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def run_behavior_sequence(self, behavior_list):
        """
        Behavior tuples:
          ("reach", dx, dy, dz)
          ("lift", dz)
          ("retreat", d)
          ("return", Pose)
          ("hold", seconds)
          ("joint", [j1..j6])
        """
        state = IDLE
        idx = 0
        while not rospy.is_shutdown() and idx < len(behavior_list):
            if state == IDLE:
                print("\n[STATE: IDLE] behavior {}/{}".format(
                    idx + 1, len(behavior_list)))
                state = MOVING

            elif state == MOVING:
                b = behavior_list[idx]
                kind = b[0]
                print("[STATE: MOVING] {}".format(b))

                if kind == "reach":
                    success, _ = self.behavior_reach(b[1], b[2], b[3])
                elif kind == "lift":
                    success, _ = self.behavior_lift(b[1])
                elif kind == "retreat":
                    success, _ = self.behavior_retreat(b[1])
                elif kind == "return":
                    success, _ = self.behavior_return_to_pose(b[1])
                elif kind == "joint":
                    success = self.move_to_joint_pose(b[1])
                elif kind == "hold":
                    print("[HOLDING] {}s".format(b[1]))
                    rospy.sleep(b[1])
                    success = True
                else:
                    print("[ERROR] Unknown behavior: {}".format(kind))
                    success = False

                state = HOLDING if success else ERROR

            elif state == HOLDING:
                rospy.sleep(0.5)
                idx += 1
                state = IDLE

            elif state == ERROR:
                print("[STATE: ERROR] retreating 3cm and skipping")
                self.behavior_retreat(0.03)
                idx += 1
                state = IDLE

        print("\n[COMPLETE] sequence finished")


if __name__ == "__main__":
    try:
        controller = UR5Controller()

        # 1) Always start by moving to a known, singularity-free pose.
        if not controller.move_to_ready_pose():
            raise RuntimeError("Failed to reach ready pose; aborting")

        # 2) Capture the post-ready pose as our home for return.
        home_pose = controller.get_current_pose()
        rospy.sleep(1.0)

        # 3) Run a small Cartesian sequence — distances tuned for UR5 reach.
        behaviors = [
            ("reach",   0.15, 0.0, -0.05),  # approach
            ("hold",    1.0),
            ("lift",    0.10),               # lift
            ("reach",   0.0, 0.15, 0.0),    # side
            ("lift",   -0.08),               # lower
            ("hold",    1.0),
            ("retreat", 0.15),               # clear
            ("return",  home_pose),          # back to ready
        ]

        print("\nStarting sequence with {} behaviors".format(len(behaviors)))
        controller.run_behavior_sequence(behaviors)
        print("\nSequence complete")

    except rospy.ROSInterruptException:
        print("\n[ERROR] ROS interrupted")
    except Exception as e:
        print("\n[ERROR] {}".format(e))
        import traceback
        traceback.print_exc()
