from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


# $ ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
# File found in the /opt/ros/humble/share/ros_gz_sim/launch/gz_sim.launch.py
# https://robotics.stackexchange.com/questions/107124/ros2-humble-how-to-pass-arguments-to-python-launch-files-from-command-line


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_gazebo_pkg = get_package_share_directory("gazebo_pkg")
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    gz_model_path = PathJoinSubstitution([pkg_gazebo_pkg, "models"])

    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value="moon",
            choices=["moon", "mars", "enceladus"],
            description="World to load into Gazebo",
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

    set_world = SetLaunchConfiguration(
        name="world_file",
        value=[LaunchConfiguration("world"), TextSubstitution(text=".sdf")],
    )
    set_gzim_resource_env = (
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_model_path),
    )
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": "empty.sdf",
            "on_exit_shutdown": "True",
        }.items(),
    )
    ros_gz_bridge = (
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[],
            remappings=[],
            output="screen",
        ),
    )
    #    Node(
    #             package='robot_state_publisher',
    #             executable='robot_state_publisher',
    #             name='robot_state_publisher',
    #             output='screen',
    #             parameters=[{'use_sim_time': use_sim_time}],
    #             arguments=[urdf]),
    res = []
    # res.append(set_world)
    # res.append(set_gzim_resource_env)
    res.append(launch_gazebo)
    res.append(ros_gz_bridge)

    return LaunchDescription(declared_arguments + res)
