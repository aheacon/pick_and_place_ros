from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import ExecuteProcess


def generate_launch_description():
    # Get the paths to the necessary files
    # Package ros_gz_sim(for Harmonics), or ros_gz or gazebo_ros
    gazebo_ros_share = get_package_share_directory("")
    mycobot_description_share = get_package_share_directory("mycobot_description")
    mycobot_gazebo_share = get_package_share_directory("mycobot_gazebo")


    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gz_sim.launch.py")
        ),

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "world", default_value="empty", description="Specify the world file name"
        ),
        DeclareLaunchArgument(
            "paused",
            default_value="false",
            description="Whether to start Gazebo paused",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        ),
        DeclareLaunchArgument(
            "gui", default_value="true", description="Whether to launch the GUI"
        ),
        DeclareLaunchArgument(
            "headless", default_value="false", description="Run headless mode"
        ),
        DeclareLaunchArgument(
            "debug", default_value="false", description="Enable debug"
        ),
    ]

   
    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(mycobot_gazebo_share, "worlds", "empty.sdf"),
            "paused": LaunchConfiguration("paused"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "gui": LaunchConfiguration("gui"),
            "headless": LaunchConfiguration("headless"),
            "debug": LaunchConfiguration("debug"),
        }.items(),
    )

    # Load robot description from xacro
    # This adds --ros-args that ROS2 doesn't recoginze
    # robot_description = Node(
    #     package="xacro",
    #     executable="xacro",
    #     arguments=[
    #         os.path.join(mycobot_description_share, "urdf", "mycobot_280.xacro")
    #     ],
    #     output="screen",
    #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    # )

    robot_description = ExecuteProcess(
        cmd=[
            "xacro",
            os.path.join(mycobot_description_share, "urdf", "mycobot_280.xacro"),
            "-o",
            os.path.join("/tmp", "mycobot_280.urdf"),
        ],
        output="screen",
    )
    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "mycobot_280",
            "-file",
            os.path.join("/tmp", "mycobot_280.urdf"),
        ],
    )

    # Return the LaunchDescription
    return LaunchDescription(
        declared_arguments + [gazebo_launch, robot_description, spawn_robot]
    )
