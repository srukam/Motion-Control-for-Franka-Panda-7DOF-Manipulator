"""
Offline Panda demo using moveit_resources_panda_moveit_config (already in
ROS 2 Jazzy). Lets you exercise panda_arm_controller without franka_ros2.

Example:
  ros2 launch panda_arm_control_ros2 panda_demo.launch.py
  ros2 launch panda_arm_control_ros2 panda_demo.launch.py launch_controller:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_controller = LaunchConfiguration("launch_controller")

    args = [
        DeclareLaunchArgument(
            "launch_controller", default_value="false",
            description="Also start the panda_arm_controller node."),
    ]

    panda_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("moveit_resources_panda_moveit_config"),
            "launch", "demo.launch.py"])),
    )

    controller_node = Node(
        package="panda_arm_control_ros2",
        executable="panda_arm_controller",
        name="panda_arm_controller",
        output="screen",
        condition=IfCondition(launch_controller),
    )

    return LaunchDescription(args + [panda_demo, controller_node])
