"""
Offline UR5 demo — same stack as ur5_real_execution but with mock hardware,
so it runs with no robot connected. Useful for verifying the controller
logic before going to the real arm.

Example:
  ros2 launch panda_arm_control_ros2 ur5_demo.launch.py
  ros2 launch panda_arm_control_ros2 ur5_demo.launch.py launch_controller:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_controller = LaunchConfiguration("launch_controller")

    args = [
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="Launch RViz with the MoveIt config."),
        DeclareLaunchArgument(
            "launch_controller", default_value="false",
            description="Also start the ur5_arm_controller node."),
    ]

    # robot_ip is required by ur5.launch.py even with mock hardware; an
    # unreachable placeholder is fine because mock hardware never connects.
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), "launch", "ur5.launch.py"])),
        launch_arguments={
            "robot_ip": "0.0.0.0",
            "use_mock_hardware": "true",
            "launch_rviz": "false",
        }.items(),
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])),
        launch_arguments={
            "ur_type": "ur5",
            "launch_rviz": launch_rviz,
        }.items(),
    )

    controller_node = Node(
        package="panda_arm_control_ros2",
        executable="ur5_arm_controller",
        name="ur5_arm_controller",
        output="screen",
        condition=IfCondition(launch_controller),
    )

    return LaunchDescription(args + [ur_driver, ur_moveit, controller_node])
