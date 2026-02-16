#!/usr/bin/python3
import rospy
import moveit_commander
import copy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

# State definitions
IDLE = 0
MOVING = 1
HOLDING = 2
ERROR = 3

class PandaController:
    """
    A reusable controller class for the panda robot arm.
    Handles Moveit initialization and provides methods for motion planning
    """

    def __init__(self):
        """
        Initialize MoveIt commander, ROS node, and robot interfaces.
        This runs once when you create a PandaController object.
        
        :param self: Description
        :param robot : RobotCommander object
        :param group : MoveGroupCommander object for panda_arm
        """
        print("\n"+"="*60)
        print("INITIALIZING PANDA CONTROLLER")
        print ("="*60)

        #Initialize Moveit commander and ROS node
        moveit_commander.roscpp_initialize([])
        rospy.init_node("panda_controller_node", anonymous=True)

        #Initialize RobotCommander and MoveGroupCommander
        self.robot = moveit_commander.RobotCommander
        self.group = moveit_commander.MoveGroupCommander("panda_arm")

        #Sleep briefly to allow current joint states to be updated
        rospy.sleep(2.0)

        #Store end-effector link name
        self.eef_link = self.group.get_end_effector_link()

        print("\n [INIT] Panda Controller Initialized Successfully")
        self.print_robot_info()
    

    def print_robot_info(self):
        """
        Private method to print current robot state information.
        Called during initialization.
        
        :param self: Description
        """
        # Print joint names
        print("\n--- Robot Information ---")
        print("Joint names:", self.group.get_active_joints())
        
        # Read current joint values
        current_joints = self.group.get_current_joint_values()
        print("Current joint values:", current_joints)
        
        # Get current pose
        print("EEF Link:", self.eef_link)
        current_pose = self.group.get_current_pose(self.eef_link).pose
        print("Current end-effector pose:")
        print("  Position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ))
        print("  Orientation: x={:.3f}, y={:.3f}, z={:.3f}, w={:.3f}".format(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ))
        print("-" * 60)

    
    def fix_time_monotonic(self,traj):
        """
        Fix trajectory timestamps to be strictly increasing.
        
        Args:
            traj: RobotTrajectory message
            
        Returns:
            Modified trajectory with monotonic timestamps
        """
        last_time = 0.0
        dt = 0.03   # 30 ms minimum step
        
        for pt in traj.joint_trajectory.points:
            t = pt.time_from_start.to_sec()
            
            if t <= last_time:
                t = last_time + dt
            
            pt.time_from_start = rospy.Duration(t)
            last_time = t
        
        return traj
    

    def get_current_pose(self):
        """
        Get the current pose of the end-effector.
        
        Returns:
            Pose object with current position and orientation
        """
        return self.group.get_current_pose(self.eef_link).pose
    
    
    def get_current_joints(self):
        """
        Get current joint values.
        
        Returns:
            List of joint angles in radians
        """
        return self.group.get_current_joint_values()
    

    def execute_cartesian_path(self, waypoints, vel_scale=0.1, acc_scale=0.1, 
                               step_size=0.01, jump_threshold=False):
        """
        Plan and execute a Cartesian path through given waypoints.
        
        Args:
            waypoints: List of Pose objects defining the path
            vel_scale: Velocity scaling factor (0.0 to 1.0)
            acc_scale: Acceleration scaling factor (0.0 to 1.0)
            step_size: Resolution for Cartesian interpolation (meters)
            jump_threshold: Max allowed joint jump (0.0 = disabled)
            
        Returns:
            Tuple of (success: bool, fraction: float)
        """
        print("\n[EXECUTE] Planning Cartesian path with {} waypoints...".format(len(waypoints)))
        
        # Compute cartesian path
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, 
            step_size, 
            jump_threshold
        )
        print("[EXECUTE] Planned fraction: {:.2%}".format(fraction))
        
        if fraction < 0.90:
            print("[EXECUTE] WARNING: Could not plan complete path (fraction < 90%)")
            return (False, fraction)
        
        # Get current state for retiming
        start_state = self.group.get_current_state()
        print("[EXECUTE] Start state retrieved")
        
        # Retime trajectory with velocity/acceleration scaling
        plan = self.group.retime_trajectory(
            start_state,
            plan,
            velocity_scaling_factor=vel_scale,
            acceleration_scaling_factor=acc_scale
        )
        print("[EXECUTE] Trajectory retimed (vel={}, acc={})".format(vel_scale, acc_scale))
        
        # Fix timing to ensure monotonic timestamps
        plan = self.fix_time_monotonic(plan)
        print("[EXECUTE] Timestamps fixed")
        
        # Prepare for execution
        self.group.stop()
        self.group.clear_pose_targets()
        plan.joint_trajectory.header.stamp = rospy.Time.now()

        # Execute the plan
        if len(plan.joint_trajectory.points) > 0:
            print("[EXECUTE] Executing trajectory with {} points...".format(
                len(plan.joint_trajectory.points)
            ))
            success = self.group.execute(plan, wait=True)
            
            if success:
                print("[EXECUTE] ✓ Execution successful!")
            else:
                print("[EXECUTE] ✗ Execution failed!")
                
            return (success, fraction)
        else:
            print("[EXECUTE] ✗ Plan has no points!")
            return (False, 0.0)
        
    def behavior_reach(self, x_offset, y_offset, z_offset):
        """
        Reach to a position relative to current end-effector pose.
        
        Args:
            x_offset: Distance to move in X direction (meters, forward/backward)
            y_offset: Distance to move in Y direction (meters, left/right)
            z_offset: Distance to move in Z direction (meters, up/down)
            
        Returns:
            Tuple of (success: bool, fraction: float)
        """
        print("\n" + "="*60)
        print("BEHAVIOR: REACH")
        print("="*60)
        print("Target offset: x={:.3f}, y={:.3f}, z={:.3f}".format(
            x_offset, y_offset, z_offset
        ))
        
        waypoints = []
        
        # Start from current pose
        start_pose = self.get_current_pose()
        waypoints.append(copy.deepcopy(start_pose))
        
        print("Current position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            start_pose.position.x,
            start_pose.position.y,
            start_pose.position.z
        ))
        
        # Create target pose with offsets
        target_pose = Pose()
        target_pose.position.x = start_pose.position.x + x_offset
        target_pose.position.y = start_pose.position.y + y_offset
        target_pose.position.z = start_pose.position.z + z_offset
        target_pose.orientation = start_pose.orientation  # Keep same orientation
        waypoints.append(target_pose)
        
        print("Target position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ))
        
        # Execute the reach motion
        success, fraction = self.execute_cartesian_path(waypoints)
        
        if success:
            print("[REACH] ✓ Reach behavior completed successfully")
        else:
            print("[REACH] ✗ Reach behavior failed")
        
        return (success, fraction)
    
    
    def behavior_lift(self, height):
        """
        Lift straight up from current position.
        
        Args:
            height: Distance to lift in Z direction (meters, positive = up)
            
        Returns:
            Tuple of (success: bool, fraction: float)
        """
        print("\n" + "="*60)
        print("BEHAVIOR: LIFT")
        print("="*60)
        print("Lift height: {:.3f}m".format(height))
        
        waypoints = []
        
        # Start from current pose
        start_pose = self.get_current_pose()
        waypoints.append(copy.deepcopy(start_pose))
        
        print("Current Z position: {:.3f}".format(start_pose.position.z))
        
        # Create lift pose (only Z changes)
        lift_pose = Pose()
        lift_pose.position.x = start_pose.position.x
        lift_pose.position.y = start_pose.position.y
        lift_pose.position.z = start_pose.position.z + height
        lift_pose.orientation = start_pose.orientation
        waypoints.append(lift_pose)
        
        print("Target Z position: {:.3f}".format(lift_pose.position.z))
        
        # Execute the lift motion
        success, fraction = self.execute_cartesian_path(waypoints)
        
        if success:
            print("[LIFT] ✓ Lift behavior completed successfully")
        else:
            print("[LIFT] ✗ Lift behavior failed")
        
        return (success, fraction)
    
    
    def behavior_retreat(self, distance):
        """
        Move backward (negative X direction) from current position.
        
        Args:
            distance: Distance to retreat (meters, positive value = backward)
            
        Returns:
            Tuple of (success: bool, fraction: float)
        """
        print("\n" + "="*60)
        print("BEHAVIOR: RETREAT")
        print("="*60)
        print("Retreat distance: {:.3f}m".format(distance))
        
        waypoints = []
        
        # Start from current pose
        start_pose = self.get_current_pose()
        waypoints.append(copy.deepcopy(start_pose))
        
        print("Current X position: {:.3f}".format(start_pose.position.x))
        
        # Create retreat pose (only X changes, negative direction)
        retreat_pose = Pose()
        retreat_pose.position.x = start_pose.position.x - distance
        retreat_pose.position.y = start_pose.position.y
        retreat_pose.position.z = start_pose.position.z
        retreat_pose.orientation = start_pose.orientation
        waypoints.append(retreat_pose)
        
        print("Target X position: {:.3f}".format(retreat_pose.position.x))
        
        # Execute the retreat motion
        success, fraction = self.execute_cartesian_path(waypoints)
        
        if success:
            print("[RETREAT] ✓ Retreat behavior completed successfully")
        else:
            print("[RETREAT] ✗ Retreat behavior failed")
        
        return (success, fraction)
    
    
    def behavior_return_to_pose(self, target_pose):
        """
        Move to a specific pose (useful for returning to start position).
        
        Args:
            target_pose: Pose object to move to
            
        Returns:
            Tuple of (success: bool, fraction: float)
        """
        print("\n" + "="*60)
        print("BEHAVIOR: RETURN TO POSE")
        print("="*60)
        
        waypoints = []
        
        # Start from current pose
        start_pose = self.get_current_pose()
        waypoints.append(copy.deepcopy(start_pose))
        
        print("Current position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            start_pose.position.x,
            start_pose.position.y,
            start_pose.position.z
        ))
        
        # Add target pose
        waypoints.append(copy.deepcopy(target_pose))
        
        print("Target position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ))
        
        # Execute the return motion
        success, fraction = self.execute_cartesian_path(waypoints)
        
        if success:
            print("[RETURN] ✓ Return behavior completed successfully")
        else:
            print("[RETURN] ✗ Return behavior failed")
        
        return (success, fraction)
       
    def run_behavior_sequence(self, behavior_list):
        """
        Execute a sequence of behaviors with state management.
        
        Args:
            behavior_list: List of tuples defining behaviors
                          Format: (behavior_type, *args)
                          Examples: ("reach", 0.1, 0.0, 0.0)
                                   ("lift", 0.05)
                                   ("retreat", 0.1)
                                   ("hold", 2.0)
        """
        state = IDLE
        current_idx = 0
        
        while not rospy.is_shutdown() and current_idx < len(behavior_list):
            
            if state == IDLE:
                print("\n[STATE: IDLE] Ready for behavior {}/{}".format(
                    current_idx + 1, len(behavior_list)
                ))
                state = MOVING
                
            elif state == MOVING:
                behavior = behavior_list[current_idx]
                behavior_type = behavior[0]
                
                print("[STATE: MOVING] Executing: {}".format(behavior))
                
                if behavior_type == "reach":
                    success, fraction = self.behavior_reach(behavior[1], behavior[2], behavior[3])
                    
                elif behavior_type == "lift":
                    success, fraction = self.behavior_lift(behavior[1])
                    
                elif behavior_type == "retreat":
                    success, fraction = self.behavior_retreat(behavior[1])
                    
                elif behavior_type == "hold":
                    print("[HOLDING] Waiting {} seconds...".format(behavior[1]))
                    rospy.sleep(behavior[1])
                    success = True
                    
                elif behavior_type == "return":
                    success, fraction = self.behavior_return_to_pose(behavior[1])
                    
                else:
                    print("[ERROR] Unknown behavior type: {}".format(behavior_type))
                    success = False
                
                if success:
                    state = HOLDING
                else:
                    state = ERROR
                    
            elif state == HOLDING:
                print("[STATE: HOLDING] Behavior complete")
                rospy.sleep(0.5)
                current_idx += 1
                state = IDLE
                
            elif state == ERROR:
                print("[STATE: ERROR] Attempting recovery")
                self.behavior_retreat(0.03)
                current_idx += 1
                state = IDLE
        
        print("\n[COMPLETE] Behavior sequence finished")


    def test_simple_motion(self):
        """
        Test method to verify the controller works.
        Executes a simple square motion pattern.
        """
        print("\n" + "="*60)
        print("RUNNING SIMPLE MOTION TEST")
        print("="*60)
        
        # Define waypoints
        waypoints = []
        start_pose = self.get_current_pose()
        waypoints.append(copy.deepcopy(start_pose))
        
        # Waypoint 1: Move +X
        pose1 = Pose()
        pose1.position.x = start_pose.position.x + 0.1
        pose1.position.y = start_pose.position.y
        pose1.position.z = start_pose.position.z
        pose1.orientation = start_pose.orientation
        waypoints.append(pose1)
        
        # Waypoint 2: Move +Z
        pose2 = Pose()
        pose2.position.x = pose1.position.x
        pose2.position.y = pose1.position.y
        pose2.position.z = pose1.position.z + 0.05
        pose2.orientation = pose1.orientation
        waypoints.append(pose2)
        
        # Waypoint 3: Return to start
        waypoints.append(copy.deepcopy(start_pose))
        
        # Execute the path
        success, fraction = self.execute_cartesian_path(waypoints)
        
        if success:
            print("\n[TEST] ✓ Simple motion test PASSED!")
        else:
            print("\n[TEST] ✗ Simple motion test FAILED!")
        
        return success
    


# Main execution
if __name__ == "__main__":
    try:
        controller = PandaController()
        
        start_pose = controller.get_current_pose()
        
        rospy.sleep(2.0)
        
        # Define behavior sequence
        behaviors = [
            ("reach", 0.20, 0.0, -0.05),   # approach
            ("hold", 1.0),                  # grasp
            ("lift", 0.12),                 # lift object
            ("reach", 0.0, 0.15, 0.0),     # move side
            ("lift", -0.08),                # lower
            ("hold", 1.0),                  # release
            ("retreat", 0.20),              # clear
            ("return", start_pose),         # home
            ]
                
        print("\nStarting sequence with {} behaviors".format(len(behaviors)))
        controller.run_behavior_sequence(behaviors)
        
        print("\nSequence complete")

    except rospy.ROSInterruptException:
        print("\n[ERROR] ROS was interrupted!")
    except Exception as e:
        print("\n[ERROR] An error occurred:")
        print(str(e))
        import traceback
        traceback.print_exc()