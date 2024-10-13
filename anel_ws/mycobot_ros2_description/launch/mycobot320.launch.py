# Author: Anel Husakovic
# Date: October 13, 2024
# Description: Display the robotic arm with RViz

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_path


def generate_launch_description():
    # Define filenames
    urdf_package = "mycobot_ros2_description"
    urdf_filename = "mycobot_320_m5_2022.urdf"
    rviz_config_filename = "mycobot_320_view_description.rviz"

    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    # Alternatively to get package share foreser and join with subforesers
    # pkg_share=  get_package_share_path("mycobot_ros2_description")
    # default_value = os.path.join(pkg_share, "urdf/mycobot_320_m5_2022/mycobot_320_m5_2022.urdf")
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, "rviz", rviz_config_filename]
    )
    print(
        f"Package share: {pkg_share_description}, URDF path: {default_urdf_model_path}"
    )

    # Launch configuration variables specific to simulation
    jsp_gui = LaunchConfiguration("jsp_gui")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    urdf_model = LaunchConfiguration("urdf_model")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name="jsp_gui",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=default_rviz_config_path,
        description="Full path to the RVIZ config file to use",
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_urdf_model_path,
        description="Absolute path to robot urdf file",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name="use_rviz", default_value="true", description="Whether to start RVIZ"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Specify the actions

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(jsp_gui),
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(jsp_gui),
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_model]), value_type=str
    )
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_content,
            }
        ],
    )

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # rviz_node = Node(
    #     name="rviz2",
    #     package="rviz2",
    #     executable="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration(rviz_config_filename)],
    # )

    # Create the launch description and populate
    res = []

    # Declare the launch options
    res.append(declare_jsp_gui_cmd)
    res.append(declare_rviz_config_file_cmd)
    res.append(declare_urdf_model_path_cmd)
    res.append(declare_use_rviz_cmd)
    res.append(declare_use_sim_time_cmd)

    # Add any actions
    res.append(start_joint_state_publisher_cmd)
    res.append(start_joint_state_publisher_gui_cmd)
    res.append(start_robot_state_publisher_cmd)
    res.append(start_rviz_cmd)
    # res.append(rviz_node) # not working

    return LaunchDescription(res)
