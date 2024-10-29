# Tutorials
[ROS noetic and gazebo](https://www.youtube.com/watch?v=J4AwGk2Qz-Q&list=PL_t44SOegIa50Po6Ei4mnvsbwEdo8Gx0y&index=2)
[ROS camera and gazebo](https://www.youtube.com/watch?v=A_PDyn2F9bI&list=PLXTzuLmr3zoGXJXgcvA6ftv2fPnh_I-tk)
[ROS2 with custom robot and moveit2](https://www.youtube.com/watch?v=EosEikbZhiM)
[ROS2 custom hardware interface](https://www.youtube.com/watch?v=be-5DPuDtO8)
[ROS2 robot URDF + Xacro creation, visualize in RVIZ2 and Gazebo + Teleoperation | Beginner Tutorial](https://www.youtube.com/watch?v=Vbh3--etiwg)
  - https://github.com/robofuntastic/my_first_pkg
[ROS1 basic urdf & rviz](https://www.youtube.com/watch?v=Ale55LcdZeE&list=PL8lID45FIDTThzI3LKHeyQSWICa6nOUro)
  - Construct video - from xacro starts create it from scratch

[ROS Arm Simulation](https://github.com/dvalenciar/robotic_arm_environment)
- GOOD https://davidvalenciaredro.wixsite.com/my-site/services-7
- https://github.com/dvalenciar/robotic_arm_environment
- He changed original file, added dampnig and friction for joints and included additional .xacro file
- He had wrapper with `xacro:macro` tag and `params` field
- https://wiki.ros.org/urdf/XML/joint - damping/friction
- `urdf` folder must have `ros2_control` folder
- Loading of the `.yaml` file of the controller in the `gazebo_ros2_control.xacro`
  - In ROS1 the configuration yaml file was loaded in the launch file while in ROS2 it is loaded in the description  URDF-xacro file.
  - `config` folder holds the settings for the controller `simple_controller`
  - Read about [gz_ros2_control pkg]()https://control.ros.org/iron/doc/gz_ros2_control/doc/index.html
    - Integrate `ros2_control` with Gazebo simulator
    - Package provides `Gazebo-Sim` system plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.
    - Docker uses [rocker](https://github.com/osrf/rocker/)
  - Basic configuration of controllers:
    - joint_state_broadcaster: This controller publishes the state of all resources registered to a hardware_interface::StateInterface to a topic of type sensor_msgs/msg/JointState.
    - joint_trajectory_controller: This controller creates an action called /joint_trajectory_controller/follow_joint_trajectory of type control_msgs::action::FollowJointTrajectory.

- Gazebo has 2 processes `server` and `client` - `pstree` (`gzserver`, `gzclient`)
- http://gazebosim.org/tutorials?tut=ros_urdf 
- Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.

-[Full tutorial with a 6DOF robot](https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/doc/userdoc.rst) - GREAT
- See https://control.ros.org/master/doc/getting_started/getting_started.html
- Problem
```bash
[INFO] [1730042328.133094781] [_ros2cli_31244]: waiting for service /controller_manager/list_controllers to become available...

[ros2_control_node-1] [WARN] [1730042240.553038035] [controller_manager]: No real-time kernel detected on this system. See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
```
- Commands
```bash
$ ros2 control list_hardware_interfaces
$ ros2 control list_controller_types
$ ros2 control list_controllers
[INFO] [1730042328.133094781] [_ros2cli_31244]: waiting for service /controller_manager/list_controllers to become available...
# https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
$ sudo addgroup realtime
$ sudo usermod -a -G realtime $(whoami)
# add to /etc/security/limits.conf (find alternative way to custom config)
$ ps aux|grep gazebo
anel       12801  0.2  0.1 353948 37860 pts/0    SNl+ 16:49   0:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch mycobot_description mycobot_320pi_gazebo_controller.launch.py
$ cat /proc/12801/limits |grep realtime
Max realtime priority     99                   99                   
Max realtime timeout      unlimited            unlimited            us  

# There is
$ ros2 node list
/controller_manager
/robot_state_publisher
/spawner_arm_controller

# Install packages (intalled)
$ sudo apt install ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller

$ ros2 node info ros2_control_node
$ ros2 run controller_manager ros2_control_node --ros-args --log-level debug
```
```bash
$ xacro mycobot_320pi.urdf.xacro > mycobot320pi_test.urdf
$ check_urdf mycobot320pi_test.urdf # checks xml tree
$ urdf_to_graphviz mycobot320pi_test.urdf mycobot320pi # verfiy kinematic chaing as .pdf
```
# ROS2 Humble with custom robot
## ros2_control
```bash
# [Err] [SystemLoader.cc:92] Failed to load system plugin [gz_ros2_control-system] : Could not find shared library. -->
# Gazebo Fortress and Humble are supported from pacakge `ros-humble-ign-ros2-control` https://github.com/ros-controls/gz_ros2_control/tree/humble-->
# We need to build the gz_ros2_control for Harmonic only from source https://github.com/ros-controls/gz_ros2_control/tree/iron and switch to ROS2 Iron 
$ mkdir -p gz_ros2_control_ws/src && cd gz_ros2_control_ws/src
$ git clone git@github.com:ros-controls/gz_ros2_control.git --branch iron
$ $ echo $ROS_DISTRO
iron
# Uninstall ROS2 humble and humble gazebo packages (or maybe not https://github.com/ros-controls/gz_ros2_control/issues/394 ?)
$ sudo apt remove ros-humble-desktop
$ sudo apt remove ros-humble-ros-gzharmonic
$ sudo apt autoremove
$ sudo rm -rf /opt/ros/humble/
# Make sure to install iron ROS2
$ sudo apt install ros-${ROS_DISTRO}-desktop
$ ls /opt/ros/
iron
# Install gazebo for iron
$ sudo apt update && sudo apt install ros-${ROS_DISTRO}-ros-gzharmonic # add it to source
# Install ros2_control pacakges https://control.ros.org/iron/doc/getting_started/getting_started.html
$ sudo apt-get install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
$ sudo apt-get remove gz-tools2 && sudo apt autoremove
$ sudo apt install ros-${ROS_DISTRO}-gazebo-ros2-control && sudo apt install ros-${ROS_DISTRO}-ros-gzharmonic  # again had to install
$ sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui # need to install
# [ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'controller_manager' not found, searching: ['/home/anel/GitHub/pick_and_place_ros/my_project/install/mycobot_description', '/opt/ros/iron']"


#  Instal gz_ros2_control
$ mkdir -p gz_ros2_control_ws/src &&  cd gz_ros2_control_ws/src
$ git clone https://github.com/ros-controls/gz_ros2_control -b ${ROS_DISTRO}
$ export GZ_VERSION=harmonic
# Problem with rosdep?
$ rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
gz_ros2_control: No definition of [gz-plugin2] for OS [ubuntu]
Continuing to install resolvable dependencies...
#All required rosdeps installed successfully

$ sudo apt-get install libgz-plugin2-dev # already on the latest

$ cd ~/gz_ros2_control_ws && colcon build
Finished <<< gz_ros2_control [16.5s]
Starting >>> gz_ros2_control_demos
Starting >>> gz_ros2_control_tests
Finished <<< gz_ros2_control_demos [8.63s]                                                                            
Finished <<< gz_ros2_control_tests [8.72s]                   

Summary: 3 packages finished [25.5s]
  1 package had stderr output: gz_ros2_control

$ find .|grep stderr # it is cmake warning

# Use it by sourcing (launch file works)
$ source /home/anel/GitHub/gz_ros2_control_ws/install/setup.bash
# $ protoc --version
#libprotoc 28.2
# $ sudo apt-get remove --purge libprotobuf-dev protobuf-compiler && sudo apt-get install libprotobuf-dev=3.12.4-1ubuntu1 protobuf-compiler=3.12.4-1ubuntu1


# Start launch file and check log file
```
- Found [Robot state publisher vs Joint state publisher](https://www.youtube.com/watch?v=9BdAkrX4Xkg)
  - `RSS` vs `JSP`
  - `ros2 param load <urdf> robot_description` # `param server` is like database that ROS provides to store parameters that any ROS program can access, exchange the same parameters for different ROS programs - load it and use it to visualize in RVIZ. `load <node_name> <parameter_file>`
    - Controller configuration is also loaded in `param server`
    - `ros2 param list`
    - `ros2 param get /robot_state_publisher robot_description` - the whole xml file
    - Visualise in rviz2 `ros2 run rviz2 rviz2` => Add `Robot Model` change descirption to `robot_description` topic and set fixed frame to `base_link`. Since we have `/robot_state_publisher` all works,since tranformations between links are known. 
  - ▸ What is Robot State Publisher
    - Is node, that uses URDF specified by `robot_description` and `joint positions` from the topic `joint_states` and calculatres transformations  (direct kinematic) and publishes to the `tf`.
  - ▸ What is the role of the /joint_states topic
    `$ ros2 topic info /joint_states` in my case there is 1 publisher and 1 subscriber
    - We need to publish the values of `/joint_states` and that is the job for `joint_state_publisher` (how to see publishers and subscribers) 
  - ▸ What is the Joint State Publisher
    - It is emulator, faking the status - not moving the robot, just visualize
    - Publishes messages of joint values in topic `JointState`. Reads URDF (from `robot_description`) and reads all non-fixed joints.
    - `use_gui = true` - showing values for each joint
    - Which value to show ?
    - `sudo apt install ros-iron-joint-state-publisher-gui`
    - `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
  - ▸ What is `JointStateController`
    - Special kind of contrller that is publishing into the `/joint_states` topic the current position of all the joints. 
    - Both are publishing to `joint_states` topic
  - ▸ When to use Rviz to visualize the joint states and when to use Gazebo
    - When working with real robot not using `joint_state_publisher` but instead using `joint_state_controller`
- List of [ros2_controllers](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html)
- Modern Gazebo
  - https://github.com/ros-controls/gz_ros2_control
- Classic Gazebo
  - Using https://github.com/ros-controls/gazebo_ros2_control
  - ROS 2 package for integrating the ros2_control controller architecture with the Gazebo Classic simulator.
    - Gazebo classic goes EOL in 2025, migrate to moder Gazebo (Ignition 3 formerly)

- My ROS2 is Humble
  - Works with Gazebo Fortress
  - Have installed and decided to proceed with Gazebo Harmonic
  - Install from https://packages.ros.org `ros-humble-ign-ros2-control`

- My ROS2 can be Iron
  - Works with Gazebo Harmonic
    - Install from source https://github.com/ros-controls/gz_ros2_control

- using `gazebo_ros` package (clasical) or `ros_gz` bridge package?
  - Based on [ROS2 integration](https://gazebosim.org/docs/all/ros2_overview/)
    - ROS interact with Gazebo via `ros_gz` - https://github.com/gazebosim/ros_gz
      - For Gazebo Harmonics: `sudo apt-get install ros-humble-ros-gzharmonic`
      - Found in `/opt/ros/humble/lib/ros_gz_sim` package with `create` binary
    - Communication via topics, Gazebo has middleware, Transport to expose set of topics
    - ROS used to spawn Gazebo model at runtime.
    - Based on [launch gazebo](https://gazebosim.org/docs/all/ros2_launch_gazebo/) it uses `ros_gz_sim` package (not recognized by the `Ignition`)
  - Based on [ROS2 integration classic](https://classic.gazebosim.org/tutorials/?tut=ros2_overview)
    - Integration is done through https://github.com/ros-simulation/gazebo_ros_pkgs (where we have `gazebo_ros` too)
    - This leads me to use the latest - Gazebo Harmonic instead of Ignition


## Pipeline for robotic arms
1. Bring robot in env 
  - urdf, xacro
  - Based on this [ROS Arm Simulation](https://github.com/dvalenciar/robotic_arm_environment)
2. Add controllers 
  - for movement of joints(controller), 
  - joint trajectory controller
    - controlls all joints with single command
  - ros2_controllers - friction, rotational aspect
  - integrate inverse kinematics utilize libraries Ikpy, robotics toolbox by Peter Cork, to drive robot
3. MoveIT configuration
  - https://moveit.picknik.ai/main/index.html
  - GUI moveit interface => generate package to drive it easily
  - Gripper part - position controller, effort controller
  - Setup Simulation
  - Setup ROS communication
4. moveit
  - generates ROS2 control tags, addition of depth sensor
5. demo
6. utilizing move group nodes
  - write codes
  - send goals

## Tutorial 1 - ROS Noetic
- Based on this https://www.youtube.com/watch?v=T_UOIHEol0I&list=PLeEzO_sX5H6TNMBiworxO8RpQlcYfl54y
- ROS package - exported from Solidworks can be used in ROS (Noetic) without any issues - [see this](https://youtube.com/playlist?list=PLeEzO_sX5H6TBD6EMGgV-qdhzxPY19m12)
  - [fusion2urdf-ros2](https://youtu.be/_ZFo6wPXjeQ?t=790)

- [Simulate Custom Arm in ROS Noetic Using Moveit.ofd](https://github.com/ageofrobotics/Simulate_Your_Robot_Arm_In_ROS_Noetic/blob/main/Importing_Your_ROS_Package_Generated_from_SolidWorks_in_ROS_Noetic.pdf)
## Tutorial 2
- Based on this [ROS2 with custom robot and moveit2](https://www.youtube.com/watch?v=EosEikbZhiM)
- Not good - no GitHub examples
- uses `.xml` launch file to spawn robot model - TODO more
- For all resourcees that come along with package, manually specify install dir in CMakeLists.txt (setup.py)
- Gazebo cannont find mesh models located in the current Gazebo model path
  - modify `package.xml` add `<gazebo_ros gazebo_model_path="intsall/pkg/share">` in `<export>`
- Ads `<ros2_control ...>` tag in the URDF file with `joint` and states and commands interfaces