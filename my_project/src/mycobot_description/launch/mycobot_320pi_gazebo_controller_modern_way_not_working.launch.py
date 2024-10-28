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
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.actions import TimerAction
import launch_ros.descriptions


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
    # use_sim_time = LaunchConfiguration("use_sim_time") # not exists
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
        parameters=[
            robot_description
            # {
            #     # "use_sim_time": use_sim_time,
            #     "robot_description": Command(
            #         ["xacro", " ", xacro_file]
            #     ),  # or doc.toxml()
            # }
        ],
    )

    # # Spawn the robot in Gazebo (old way with gazebo_ros package)
    # spawn_entity_robot = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-entity", "mycobot_320pi_robot", "-topic", "robot_description"],
    #     output="screen",
    # )
    # Set the default pose
    # x = LaunchConfiguration("x")
    # y = LaunchConfiguration("y")
    # z = LaunchConfiguration("z")
    # roll = LaunchConfiguration("roll")
    # pitch = LaunchConfiguration("pitch")
    # yaw = LaunchConfiguration("yaw")
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
            # "-x",
            # x,
            # "-y",
            # y,
            # "-z",
            # z,
            # "-R",
            # roll,
            # "-P",
            # pitch,
            # "-Y",
            # yaw,
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Gazebo
    world_file_name = "empty.world"
    world = os.path.join(
        get_package_share_directory(mycobot_description_pkg), "worlds", world_file_name
    )
    # Old way (gazebo command)
    # gazebo_node = ExecuteProcess(
    #     cmd=["gazebo", "--verbose", world, "-s", "libgazebo_ros_factory.so"],
    #     output="screen",
    # )
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
    # gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        # condition=IfCondition(gui),
    )
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "--set-state",
    #         "active",
    #         "joint_state_broadcaster",
    #     ],
    #     output="screen",
    # )
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
    return LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            spawn_entity_robot,
            start_gazebo_cmd,  # gazebo_node,
            arm_controller_spawner,
            delay_joint_state_broadcaster,  # delay_joint_state_broadcaster_after_arm_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            # load_joint_state_broadcaster,
            # load_joint_trajectory_controller,
        ]
    )

    """
    $ ros2 launch mycobot_description mycobot_320pi_gazebo_controller_modern_way_not_working.launch.py
[INFO] [launch]: All log files can be found below /home/anel/.ros/log/2024-10-27-18-21-09-130187-anel-23013
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ros2_control_node-1]: process started with pid [23016]
[INFO] [robot_state_publisher-2]: process started with pid [23018]
[INFO] [create-3]: process started with pid [23020]
[INFO] [ruby $(which gz) sim-4]: process started with pid [23022]
[INFO] [spawner-5]: process started with pid [23024]
[ros2_control_node-1] [INFO] [1730049669.465971810] [controller_manager]: Subscribing to '~/robot_description' topic for robot description file.
[ros2_control_node-1] [INFO] [1730049669.466941912] [controller_manager]: update rate is 100 Hz
[ros2_control_node-1] [WARN] [1730049669.467098779] [controller_manager]: No real-time kernel detected on this system. See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[create-3] [INFO] [1730049669.467497973] [ros_gz_sim]: Requesting list of world names.
[robot_state_publisher-2] [INFO] [1730049669.496701675] [robot_state_publisher]: got segment base_link
[robot_state_publisher-2] [INFO] [1730049669.496805361] [robot_state_publisher]: got segment link1
[robot_state_publisher-2] [INFO] [1730049669.496818206] [robot_state_publisher]: got segment link2
[robot_state_publisher-2] [INFO] [1730049669.496824487] [robot_state_publisher]: got segment link3
[robot_state_publisher-2] [INFO] [1730049669.496829958] [robot_state_publisher]: got segment link4
[robot_state_publisher-2] [INFO] [1730049669.496834917] [robot_state_publisher]: got segment link5
[robot_state_publisher-2] [INFO] [1730049669.496839856] [robot_state_publisher]: got segment link6
[robot_state_publisher-2] [INFO] [1730049669.496844886] [robot_state_publisher]: got segment world
[spawner-5] [INFO] [1730049669.718397881] [spawner_arm_controller]: waiting for service /controller_manager/list_controllers to become available...
[ruby $(which gz) sim-4] [Dbg] [gz.cc:166] Subscribing to [/gazebo/starting_world].
[ruby $(which gz) sim-4] [Dbg] [gz.cc:168] Waiting for a world to be set from the GUI...
[ruby $(which gz) sim-4] [Msg] Received world [/home/anel/GitHub/pick_and_place_ros/my_project/install/mycobot_description/share/mycobot_description/worlds/empty.world] from the GUI.
[ruby $(which gz) sim-4] [Dbg] [gz.cc:172] Unsubscribing from [/gazebo/starting_world].
[ruby $(which gz) sim-4] [Msg] Gazebo Sim Server v8.6.0
[ruby $(which gz) sim-4] [Msg] Loading SDF world file[/home/anel/GitHub/pick_and_place_ros/my_project/install/mycobot_description/share/mycobot_description/worlds/empty.world].
[ruby $(which gz) sim-4] [Msg] Serving entity system service on [/entity/system/add]
[ruby $(which gz) sim-4] [Dbg] [Physics.cc:870] Loaded [gz::physics::dartsim::Plugin] from library [/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics-dartsim-plugin.so]
[ruby $(which gz) sim-4] [Dbg] [SystemManager.cc:77] Loaded system [gz::sim::systems::Physics] for entity [1]
[ruby $(which gz) sim-4] [Msg] Create service on [/world/default/create]
[ruby $(which gz) sim-4] [Msg] Remove service on [/world/default/remove]
[ruby $(which gz) sim-4] [Msg] Pose service on [/world/default/set_pose]
[ruby $(which gz) sim-4] [Msg] Pose service on [/world/default/set_pose_vector]
[ruby $(which gz) sim-4] [Msg] Light configuration service on [/world/default/light_config]
[ruby $(which gz) sim-4] [Msg] Physics service on [/world/default/set_physics]
[ruby $(which gz) sim-4] [Msg] SphericalCoordinates service on [/world/default/set_spherical_coordinates]
[ruby $(which gz) sim-4] [Msg] Enable collision service on [/world/default/enable_collision]
[ruby $(which gz) sim-4] [Msg] Disable collision service on [/world/default/disable_collision]
[ruby $(which gz) sim-4] [Msg] Material service on [/world/default/visual_config]
[ruby $(which gz) sim-4] [Msg] Material service on [/world/default/wheel_slip]
[ruby $(which gz) sim-4] [Dbg] [SystemManager.cc:77] Loaded system [gz::sim::systems::UserCommands] for entity [1]
[ruby $(which gz) sim-4] [Dbg] [SystemManager.cc:77] Loaded system [gz::sim::systems::SceneBroadcaster] for entity [1]
[ruby $(which gz) sim-4] [Msg] Loaded level [3]
[ruby $(which gz) sim-4] [Msg] Serving world controls on [/world/default/control], [/world/default/control/state] and [/world/default/playback/control]
[ruby $(which gz) sim-4] [Msg] Serving GUI information on [/world/default/gui/info]
[ruby $(which gz) sim-4] [Msg] World [default] initialized with [default_physics] physics profile.
[ruby $(which gz) sim-4] [Msg] Serving world SDF generation service on [/world/default/generate_world_sdf]
[ruby $(which gz) sim-4] [Msg] Serving world names on [/gazebo/worlds]
[ruby $(which gz) sim-4] [Msg] Resource path add service on [/gazebo/resource_paths/add].
[ruby $(which gz) sim-4] [Msg] Resource path get service on [/gazebo/resource_paths/get].
[create-3] [INFO] [1730049670.262720279] [ros_gz_sim]: Requested creation of entity.
[create-3] [INFO] [1730049670.262782587] [ros_gz_sim]: OK creation of entity.
[INFO] [create-3]: process has finished cleanly [pid 23020]
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [Msg] Resource path resolve service on [/ga[Msg] Gazebo Sim GUI    v8.6.0
[ruby $(which gz) sim-4] [Dbg] [Gui.cc:263] Waiting for subscribers to [/gazebo/starting_world]...
[ruby $(which gz) sim-4] [Dbg] [Application.cc:96] Initializing application.
[ruby $(which gz) sim-4] [Dbg] [Application.cc:170] Qt using OpenGL graphics interface
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:657] Create main window
[ruby $(which gz) sim-4] [GUI] [Dbg] [PathManager.cc:68] Requesting resource paths through [/gazebo/resource_paths/get]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Gui.cc:343] GUI requesting list of world names. The server may be busy downloading resources. Please be patient.
[ruby $(which gz) sim-4] [GUI] [Dbg] [PathManager.cc:57] Received resource paths.
[ruby $(which gz) sim-4] [GUI] [Dbg] [Gui.cc:401] Requesting GUI from [/world/default/gui/info]...
[ruby $(which gz) sim-4] [GUI] [Dbg] [GuiRunner.cc:149] Requesting initial state from [/world/default/state]...
[ruby $(which gz) sim-4] [GUI] [Msg] Loading config [/home/anel/.gz/sim/8/gui.config]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [MinimalScene]
[ruby $(which gz) sim-4] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[ruby $(which gz) sim-4] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[ruby $(which gz) sim-4] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[ruby $(which gz) sim-4] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[ruby $(which gz) sim-4] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [3D View] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [MinimalScene] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libMinimalScene.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [EntityContextMenuPlugin]
[ruby $(which gz) sim-4] [GUI] [Msg] Currently tracking topic on [/gui/currently_tracked]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Entity Context Menu] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [EntityContextMenuPlugin] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libEntityContextMenuPlugin.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [GzSceneManager]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Scene Manager] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [GzSceneManager] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libGzSceneManager.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [InteractiveViewControl]
[ruby $(which gz) sim-4] [GUI] [Msg] Camera view controller topic advertised on [/gui/camera/view_control]
[ruby $(which gz) sim-4] [GUI] [Msg] Camera reference visual topic advertised on [/gui/camera/view_control/reference_visual]
[ruby $(which gz) sim-4] [GUI] [Msg] Camera view control sensitivity advertised on [/gui/camera/view_control/sensitivity]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Interactive view control] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [InteractiveViewControl] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libInteractiveViewControl.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [CameraTracking]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Camera tracking] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [CameraTracking] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libCameraTracking.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [MarkerManager]
[ruby $(which gz) sim-4] [GUI] [Msg] Listening to stats on [/world/default/stats]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Marker Manager] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [MarkerManager] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libMarkerManager.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [SelectEntities]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Select entities] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [SelectEntities] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libSelectEntities.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [Spawn]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Spawn] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [Spawn] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libSpawn.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [VisualizationCapabilities]
[ruby $(which gz) sim-4] [GUI] [Msg] View as transparent service on [/gui/view/transparent]
[ruby $(which gz) sim-4] [GUI] [Msg] View as wireframes service on [/gui/view/wireframes]
[ruby $(which gz) sim-4] [GUI] [Msg] View center of mass service on [/gui/view/com]
[ruby $(which gz) sim-4] [GUI] [Msg] View inertia service on [/gui/view/inertia]
[ruby $(which gz) sim-4] [GUI] [Msg] View collisions service on [/gui/view/collisions]
[ruby $(which gz) sim-4] [GUI] [Msg] View joints service on [/gui/view/joints]
[ruby $(which gz) sim-4] [GUI] [Msg] View frames service on [/gui/view/frames]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Visualization capabilities] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [VisualizationCapabilities] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libVisualizationCapabilities.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [WorldControl]
[ruby $(which gz) sim-4] [GUI] [Msg] Using world control service [/world/default/control]
[ruby $(which gz) sim-4] [GUI] [Msg] Listening to stats on [/world/default/stats]
[ruby $(which gz) sim-4] [GUI] [Dbg] [WorldControl.cc:237] Using an event to share WorldControl msgs with the server
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [World control] to main window
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityTree/EntityTree.qml:148:7: QML ToolButton: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [WorldControl] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libWorldControl.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [WorldStats]
[ruby $(which gz) sim-4] [GUI] [Msg] Listening to stats on [/world/default/stats]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [World stats] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [WorldStats] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libWorldStats.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [Shapes]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Shapes] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [Shapes] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libShapes.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [Lights]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Lights] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [Lights] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libLights.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [TransformControl]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Transform control] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [TransformControl] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libTransformControl.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [Screenshot]
[ruby $(which gz) sim-4] [GUI] [Msg] Screenshot service on [/gui/screenshot]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Screenshot] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [Screenshot] from path [/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libScreenshot.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [CopyPaste]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Copy/Paste] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [CopyPaste] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libCopyPaste.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [ComponentInspector]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Component inspector] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [ComponentInspector] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libComponentInspector.so]
[ruby $(which gz) sim-4] [GUI] [Dbg] [Application.cc:528] Loading plugin [EntityTree]
[ruby $(which gz) sim-4] [GUI] [Msg] Currently tracking topic on [/gui/currently_tracked]
[ruby $(which gz) sim-4] [GUI] [Msg] Added plugin [Entity tree] to main window
[ruby $(which gz) sim-4] [GUI] [Msg] Loaded plugin [EntityTree] from path [/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui/libEntityTree.so]
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityTree/EntityTree.qml:148:7: QML ToolButton: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/WorldStats/WorldStats.qml:53:3: QML RowLayout: Binding loop detected for property "x"
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Application.cc:908] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[ruby $(which gz) sim-4] libEGL warning: egl: failed to create dri2 screen
[ruby $(which gz) sim-4] libEGL warning: egl: failed to create dri2 screen
[INFO] [spawner-6]: process started with pid [23267]
[ruby $(which gz) sim-4] [GUI] [Dzebo/resource_paths/resolve].
[ruby $(which gz) sim-4] [Msg] Resource paths published on [/gazebo/resource_paths].
[ruby $(which gz) sim-4] [Msg] Server control service on [/server_control].
[ruby $(which gz) sim-4] [Msg] Found no publishers on /stats, adding root stats topic
[ruby $(which gz) sim-4] [Msg] Found no publishers on /clock, adding root clock topic
[ruby $(which gz) sim-4] [Dbg] [SimulationRunner.cc:551] Creating PostUpdate worker threads: 2
[ruby $(which gz) sim-4] [Dbg] [SimulationRunner.cc:562] Creating postupdate worker thread (0)
[ruby $(which gz) sim-4] [Dbg] [UserCommands.cc:1311] Created entity [10] named [mycobot_320pi]
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [base_link_collision] couldn't be created
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [link1_collision] couldn't be created
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [link2_collision] couldn't be created
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [link3_collision] couldn't be created
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [link4_collision] couldn't be created
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [link5_collision] couldn't be created
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[ruby $(which gz) sim-4] [Dbg] [SDFFeatures.cc:863] The geometry element of collision [link6_collision] couldn't be created
[ruby $(which gz) sim-4] [Msg] Serving scene information on [/world/default/scene/info]
[ruby $(which gz) sim-4] [Msg] Serving graph information on [/world/default/scene/graph]
[ruby $(which gz) sim-4] [Msg] Serving full state on [/world/default/state]
[ruby $(which gz) sim-4] [Msg] Serving full state (async) on [/world/default/state_async]
[ruby $(which gz) sim-4] [Msg] Publishing scene information on [/world/default/scene/info]
[ruby $(which gz) sim-4] [Msg] Publishing entity deletions on [/world/default/scene/deletion]
[ruby $(which gz) sim-4] [Msg] Publishing state changes on [/world/default/state]
[ruby $(which gz) sim-4] [Wrn] [Component.hh:144] Trying to serialize component with data type [N3sdf3v145WorldE], which doesn't have `operator<<`. Component will not be serialized.
[ruby $(which gz) sim-4] [GUI] [Wrn] [Component.hh:189] Trying to deserialize component with data type [N3sdf3v145WorldE], which doesn't have `operator>>`. Component will not be deserialized.
[spawner-6] [INFO] [1730049675.075755204] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-5] [WARN] [1730049679.760869153] [spawner_arm_controller]: Could not contact service /controller_manager/list_controllers
[spawner-5] [INFO] [1730049679.761568575] [spawner_arm_controller]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1730049685.119873392] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1730049685.120555992] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-5] [WARN] [1730049689.803883963] [spawner_arm_controller]: Could not contact service /controller_manager/list_controllers
[spawner-5] [INFO] [1730049689.804623790] [spawner_arm_controller]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1730049695.162106046] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1730049695.163056251] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-5] [WARN] [1730049699.850507963] [spawner_arm_controller]: Could not contact service /controller_manager/list_controllers
[spawner-5] [INFO] [1730049699.850941923] [spawner_arm_controller]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1730049705.206828696] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1730049705.207514502] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...


$ ros2 topic list
/controller_manager/robot_description
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static


$ ros2 service list
/controller_manager/describe_parameters
/controller_manager/get_parameter_types
/controller_manager/get_parameters
/controller_manager/list_controllers  # you are here ????
/controller_manager/list_parameters
/controller_manager/load_controller
/controller_manager/set_parameters
/controller_manager/set_parameters_atomically
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/spawner_arm_controller/describe_parameters
/spawner_arm_controller/get_parameter_types
/spawner_arm_controller/get_parameters
/spawner_arm_controller/list_parameters
/spawner_arm_controller/set_parameters
/spawner_arm_controller/set_parameters_atomically
/spawner_joint_state_broadcaster/describe_parameters
/spawner_joint_state_broadcaster/get_parameter_types
/spawner_joint_state_broadcaster/get_parameters
/spawner_joint_state_broadcaster/list_parameters
/spawner_joint_state_broadcaster/set_parameters
/spawner_joint_state_broadcaster/set_parameters_atomically

    """
