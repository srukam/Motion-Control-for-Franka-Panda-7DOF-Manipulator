"""
ROS 2 equivalent of launch/ur5_real_execution.launch.

Brings up:
  1. ur_robot_driver/ur_control.launch.py with robot_ip + ur_type
     (RTDE driver, ros2_control, robot_state_publisher).
  2. ur_moveit_config/ur_moveit.launch.py with the same ur_type
     (MoveIt 2 + RViz).
  3. (optional) panda_arm_control_ros2 ur5_arm_controller node.

ur_type defaults to "ur5". Pass ur_type:=ur5e if your robot is e-Series
(Polyscope 5.x firmware) — the kinematics differ slightly so the move
group will mismatch reality if this is wrong.

Pendant prerequisites:
  - Pendant dropdown -> "Remote Control"
  - EITHER set headless_mode:=true (driver streams URScript directly)
  - OR install + run the "External Control" URCap program (Host IP =
    this computer). External Control is the production setup; headless
    is the quickest path for first contact.

Examples:
  # quickest first-contact test (no URCap configuration needed):
  ros2 launch panda_arm_control_ros2 ur5_real_execution.launch.py \\
       headless_mode:=true

  # full setup (URCap installed + program playing on pendant):
  ros2 launch panda_arm_control_ros2 ur5_real_execution.launch.py \\
       robot_ip:=192.168.1.15 launch_controller:=true

  # UR5e (e-Series):
  ros2 launch panda_arm_control_ros2 ur5_real_execution.launch.py \\
       ur_type:=ur5e headless_mode:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_controller = LaunchConfiguration("launch_controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")

    args = [
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.15",
            description="IP address of the UR robot."),
        DeclareLaunchArgument(
            "ur_type", default_value="ur5",
            choices=["ur3", "ur5", "ur10",
                     "ur3e", "ur5e", "ur7e", "ur10e",
                     "ur12e", "ur16e", "ur20", "ur30"],
            description="UR robot type. Use 'ur5e' for e-Series (Polyscope 5.x)."),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="Launch RViz with the MoveIt config."),
        DeclareLaunchArgument(
            "launch_controller", default_value="false",
            description="Also start the ur5_arm_controller node."),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Use ros2_control mock hardware instead of the real "
                        "robot (lets you test offline)."),
        DeclareLaunchArgument(
            "headless_mode", default_value="false",
            description="If true, the driver sends URScript directly and does "
                        "NOT require the External Control URCap program to be "
                        "loaded + playing on the pendant. Quickest way to test "
                        "the real robot before setting up the URCap."),
    ]

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "headless_mode": headless_mode,
            "launch_rviz": "false",  # MoveIt brings its own
        }.items(),
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    # Start the controller AFTER move_group + ros2_control are up. 15 s
    # is generous — driver activation takes ~10 s on this machine. Doing
    # this from inside the launch (rather than in a separate terminal)
    # keeps the gap between "trajectory controller activated" and "first
    # motion goal" small, so the URScript is fresh.
    controller_node = TimerAction(
        period=15.0,
        actions=[Node(
            package="panda_arm_control_ros2",
            executable="ur5_arm_controller",
            name="ur5_arm_controller",
            output="screen",
        )],
        condition=IfCondition(launch_controller),
    )

    return LaunchDescription(args + [ur_driver, ur_moveit, controller_node])
