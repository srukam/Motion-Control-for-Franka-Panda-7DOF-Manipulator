"""
ROS 2 equivalent of launch/moveit_real_execution.launch (Panda real-robot
bringup with FCI driver + MoveIt 2).

============================================================================
NOT WIRED OUT OF THE BOX ON THIS MACHINE
============================================================================
franka_ros2 is NOT installed in /opt/ros/jazzy at the time this file was
written, and the package is not (yet) in the apt index for Jazzy. Source-
build it before launching:

    cd ~/ros2_ws/src
    git clone https://github.com/frankarobotics/franka_ros2.git
    git clone https://github.com/frankarobotics/franka_description.git
    cd ..
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install

Tested upstream packages this launch expects:
    franka_bringup       — franka_arm.launch.py (FCI bringup)
    franka_moveit_config — moveit.launch.py (Panda MoveIt 2 config)

Robot prerequisites (the franka_ros2 equivalent of the ROS 1 setup):
    1. Connected to Franka via Ethernet on the FCI subnet.
    2. FCI enabled in Desk: Settings -> System -> "Activate FCI".
    3. Brakes released (Desk -> "Open Brakes") and emergency stop deactivated.
    4. User button on the pendant pressed (or guiding-mode button held).
    5. robot_ip arg matches the Franka FCI IP (default 172.16.0.2).

Example:
    ros2 launch panda_arm_control_ros2 panda_real_execution.launch.py \\
         robot_ip:=172.16.0.2 launch_controller:=true
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_controller = LaunchConfiguration("launch_controller")

    args = [
        DeclareLaunchArgument(
            "robot_ip", default_value="172.16.0.2",
            description="Franka FCI IP address."),
        DeclareLaunchArgument(
            "arm_id", default_value="fr3",
            description="Arm identifier (fr3 / panda). franka_ros2 currently "
                        "ships fr3 as the canonical id; pass 'panda' if your "
                        "fork supports it."),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false",
            description="Use mock hardware instead of FCI (offline test)."),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="Launch RViz with the MoveIt config."),
        DeclareLaunchArgument(
            "launch_controller", default_value="false",
            description="Also start the panda_arm_controller node."),
    ]

    pre_check = LogInfo(msg=[
        "panda_real_execution: requires franka_ros2 + franka_moveit_config ",
        "to be source-built in your colcon workspace. If launch fails with ",
        "'package franka_bringup not found', see the header comment of this ",
        "launch file for clone + build instructions."])

    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("franka_bringup"), "launch", "franka.launch.py"])),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "use_fake_hardware": use_fake_hardware,
            "load_gripper": "true",
        }.items(),
    )

    franka_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("franka_moveit_config"), "launch", "moveit.launch.py"])),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    controller_node = Node(
        package="panda_arm_control_ros2",
        executable="panda_arm_controller",
        name="panda_arm_controller",
        output="screen",
        condition=IfCondition(launch_controller),
    )

    return LaunchDescription(args + [pre_check, franka_bringup, franka_moveit, controller_node])
