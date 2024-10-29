"""
Based on:
 https://github.com/dvalenciar/robotic_arm_environment/blob/main/my_doosan_pkg/launch/my_doosan_gazebo_controller.launch.py
 https://github.com/automaticaddison/mycobot_ros2/blob/main/mycobot_gazebo/launch/mycobot_280_arduino_bringup_ros2_control_gazebo.launch.py
 https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/bringup/launch/r6bot_controller.launch.py#L62
 https://control.ros.org/master/doc/getting_started/getting_started.html
 https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/bringup/launch/rrbot.launch.py

"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# from launch.conditions import IfCondition


def generate_launch_description():

    robot_model = "mycobot_320pi"
    mycobot_description_pkg = "mycobot_description"

    xacro_file = (
        get_package_share_directory("mycobot_description")
        + "/xacro/"
        + robot_model
        + ".urdf.xacro"
    )

    # Robot State Publisher
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(mycobot_description_pkg),
                    "xarco",
                    xacro_file,
                ]
            ),
        ]
    )
    robot_description = {
        "use_sim_time": True,
        "robot_description": launch_ros.descriptions.ParameterValue(
            robot_description_content, value_type=str
        ),
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    spawn_entity_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            doc.toxml(),
            "-name",
            robot_model,
            "-allow_renaming",
            "true",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Gazebo
    world_file_name = "empty.world"
    world = os.path.join(
        get_package_share_directory(mycobot_description_pkg), "worlds", world_file_name
    )

    # Start Gazebo environment (via launching the file from ros_gz_sim package)
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", [" -r -v 4 ", world])],
    )

    # load and START the controllers in launch file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(mycobot_description_pkg),
            "config",
            "simple_controller.yaml",
        ],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {"use_sim_time": True}],
        output="both",
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(mycobot_description_pkg),
            "rviz",
            "mycobot_320pi_view_description.rviz",
        ]
    )

    # Initialize Arguments
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        # condition=IfCondition(gui),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", robot_controllers],
        parameters=[{"use_sim_time": True}],
    )

    #  ros2 run controller_manager spawner --param-file /config/simple_controller.yaml joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            # "--controller-manager",
            # "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
    )

    delay_joint_state_broadcaster = TimerAction(
        period=5.0, actions=[joint_state_broadcaster_spawner]
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay `joint_state_broadcaster_spawner` start after `arm_controller_spawner`
    # delay_joint_state_broadcaster_after_arm_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=arm_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )
    rqt_joint_trajectory_spawner = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        arguments=[],
    )
    delay_rqt_joint_trajectory_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rqt_joint_trajectory_spawner],
        )
    )
    return LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            spawn_entity_robot,
            start_gazebo_cmd,  # gazebo_node,
            arm_controller_spawner,
            delay_joint_state_broadcaster,  # delay_joint_state_broadcaster_after_arm_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_rqt_joint_trajectory_spawner,
        ]
    )
