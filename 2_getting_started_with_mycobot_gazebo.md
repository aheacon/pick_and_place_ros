## 1. Getting started with ROS2 and myCobot with Webots simulator

### Prerequisites
TODO

### 1.1 Based on
1. [mycobot_ros2 tutorial](https://github.com/automaticaddison/mycobot_ros2)
2. [automaticaddison tutorials myCobot 280](https://automaticaddison.com/tutorials/)

### Basics 🛠️ 
- TODO for later - needs polishing

## Gazebo ignition
- Versions:
Ionic (9.x.x), Harmonic (8.x.x), Garden (7.x.x), and Fortress (6.x.x) 
```bash
$ gz sim shapes.sdf # empty.sdf
$ gz sim --version # --versions also work
Gazebo Sim, version 8.6.0
# One can have multiple versions
$ gz sim --force-version 9.0.0 shapes.sdf
Version error: I cannot find this command in version [9.0.0].
```
[Tutorial 1](https://gazebosim.org/docs/latest/gui/)
## PLugins
A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. Plugins make us control many aspects of the simulation like world, models, etc.

1. `Component inspector`
2. `Entity Tree`
3. `Grid config`
4. `Transform control`
5. `View angle`
6. `Align tool`
7. `Resource Spawner`
  - resource owner `openrobotics`
8. `diff_drive`

- [Gazebo fuel](https://app.gazebosim.org/fuel/models

[Tutorial 2](https://www.theconstruct.ai/category/gazebo-tutorials/)

## Download the model
```bash
$ ls -la
total 76
drwxrwxr-x  5 anel anel  4096 Oct 22 12:45 .
drwxr-xr-x 22 anel anel 45056 Oct 22 12:45 ..
drwxrwxr-x  4 anel anel  4096 Sep 27  2023 materials # important
drwxrwxr-x  2 anel anel  4096 Sep 27  2023 meshes    # important
-rw-rw-r--  1 anel anel   247 Sep 27  2023 metadata.pbtxt
-rw-rw-r--  1 anel anel   289 Sep 27  2023 model.config # important
-rw-rw-r--  1 anel anel  1430 Sep 27  2023 model.sdf # important
$ export GZ_SIM_RESOURCE_PATH=/home/anel/GitHub/pick_and_place_ros/my_models
```
- Models to download http://models.gazebosim.org/
  - `sun`, etc.

## Build the robot
[Based on this](https://gazebosim.org/docs/latest/building_robot/)
[SDFormat](http://sdformat.org/) (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.
  - Fortress uses SF 1.8

[GitHub sdf file](https://github.com/gazebosim/docs/blob/master/ionic/tutorials/building_robot/building_robot.sdf)

```bash
$ gz sim building_robot.sdf
```
Every model is a group of links (can be just one link) connected together with joints.
[amesweb calculate mass moment of inertia matrix](https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx)

```bash
$ sudo apt-get install libnvidia-egl-wayland1
```

Joint types:
- `revolute` - 1 DOF
- `ball` - 3 DOF



## 2. Move robot
### 2.1 Move with plugin diff_drive
- `diff_drive` plugin

Plugin tag:
1. `filename` - library filename
2. `name` - name of the plugin
3. topic `<topic>cmd_vel</topic>` - `cmd_vel` wil publish commands (messages)
```bash
$ gz topic -h
$ gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}" # press play simualtion
# MOVE IN THE GAZEBO - PRESS KEYS AND SEE IN TERMINAL
```
More about [Gazebo topics - Transport library](https://gazebosim.org/api/transport/14/tutorials.html) - TODO

### 2.2 Move with KeyPublisher
```
$ gz sim building_robot.sdf
```
- Use plugin `KeyPublisher` in Gazebo
- Topic `/keyboard/keypress` - keystrokes are being published to this topic
- Run `gz topic -e -t /keyboard/keypress`  (and play button in simulation)
- Map these keystrokes into messages of type Twist and publish them to the /cmd_vel with `TriggeredPublisher`
  - See [gz-sim/tutorials/triggered_publisher.md](https://github.com/gazebosim/gz-sim/blob/gz-sim9/tutorials/triggered_publisher.md)
- There is no stop keystroke

- Where it comes from `/model/vehicle_blude/odometry` ? From plugins
  - This is without pluging (just `build_robot.sf`)
```bash
$ gz topic -l
/clock
/gazebo/resource_paths
/gui/camera/pose
/gui/currently_tracked
/gui/track
/stats
/world/car_world/clock
/world/car_world/dynamic_pose/info
/world/car_world/pose/info
/world/car_world/scene/deletion
/world/car_world/scene/info
/world/car_world/state
/world/car_world/stats
/world/car_world/light_config
/world/car_world/material_color
```
  - This is with pluging (just `moving_robot.sf`)
```bash
$ gz topic -l
/clock
/gazebo/resource_paths
/gui/camera/pose
/gui/currently_tracked
/gui/track
/model/vehicle_blue/odometry
/model/vehicle_blue/tf
/stats
/world/car_world/clock
/world/car_world/dynamic_pose/info
/world/car_world/pose/info
/world/car_world/scene/deletion
/world/car_world/scene/info
/world/car_world/state
/world/car_world/stats
/cmd_vel
/model/vehicle_blue/enable
/world/car_world/light_config
/world/car_world/material_color
```

## SDF worlds
[Based on this](https://gazebosim.org/docs/latest/sdf_worlds/)
[gz-gui](https://github.com/gazebosim/gz-gui/)
```bash
# Publish stat from the world
$ gz topic -e -t /world/world_demo/stats

```

## Sensors
- [GitHub examples](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds)
- [Library of gz-sensors](https://github.com/gazebosim/gz-sensors)
- [ROS launches](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos/launch)
- The inertial measurement unit (IMU) sensor
```bash
# Get positions and orientations
$ gz topic -e -t /imu
# Topic to check if wall is touched
$ gz topic -e  -t /wall/touched
# Lidar
$ gz topic -e -t /lidar
```
- [Publish/subscribe in Gazebo](https://gazebosim.org/api/transport/14/messages.html)
  - Prerequisits is to [install Gazebo Transport](https://gazebosim.org/api/transport/14/installation.html)
  ```bash
  $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  # This is depracated cannot install the package
  $ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  # List  keys
  $ sudo apt-key list
  pub   rsa2048 2015-04-01 [SC]
        D248 6D2D D83D B692 72AF  E988 6717 0598 AF24 9743
  uid           [ unknown] OSRF Repository (OSRF Repository GPG key) <osrfbuild@osrfoundation.org>

  # Export key
  $ sudo apt-key export AF249743 | sudo gpg --dearmor -o /usr/share/keyrings/gazebo.gpg
  # Update source
  $ cat /etc/apt/sources.list.d/gazebo-stable.list 
  deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main
  # Install doesn't work
  $ sudo apt-get install libgz-transport8-dev
  # sudo apt-get install libgazebo11-dev
  # sudo apt install libignition-transport8-log-dev
  ```

```bash
# Publish/subscribe node
$ mkdir build
$ cd build && cmake .. && make lidar_node && ./build/lidar_node
```
- Launch
```bash
$ gz launch sensor_launch.gzlaunch
```

## Actors
- Animated model - actor
- Trajectories animation - actors lnks around the world as one group
  - Scripted in SDF, [see](http://sdformat.org/spec?ver=1.8&amp;elem=actor)
- Skeleton animatino - motion between links
  - Imported from COLLADA (.dae) and Biovision Hierarchy(BVH)(.bvh) files
- Combined

## Spawing the services
- While `SDF` can describe a world with multiple robot models, `URDF` can only describe one robot model.
- If you have a `xacro` representation of a robot model, you can turn the `xacro` file into a `URDF` file
  - using the `xacro` package: [see this tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) for more information

- [gazebo demo package](https://github.com/ros-simulation/gazebo_ros_demos) has `rrbot.urdf`
- Use `/world/empty/create` service from `gz sim empty.sdf && gz service -l`
```bash
# Informaton about service
$  gz service -is /world/empty/create
# Spawn (https://gazebosim.org/libs/sdformat/ library will convert URDF->SDF XML)
$ gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/anel/GitHub/pick_and_place_ros/gazebo/urdf_mycobot/mycobot_320_pi_2022.urdf", name: "mycobot_320_pi"'
```
- Note [unofficial mycobot](https://github.com/Tiryoh/mycobot_ros/blob/main/mycobot_gazebo/launch/mycobot_with_emptyworld.launch) and [automaticaddision](https://github.com/automaticaddison/mycobot_ros2/blob/main/mycobot_gazebo/urdf/mycobot_280_gazebo.urdf.xacro) are using `xacro` files. TODO in new folder this
## Questions
1. Cannot move `caster` around build_robot.sdf?
2. Installing and using Gazebo Transport?
3. Loading `mycobot` via `gz service` not working
```bash
Warning [parser_urdf.cc:2774] Error Code 19: Msg: urdf2sdf: link[base] has no <inertial> block defined. Please ensure this link has a valid mass to prevent any missing links or joints in the resulting SDF model.
Error Code 19: Msg: urdf2sdf: [1] child joints ignored.
Error Code 19: Msg: urdf2sdf: [1] child links ignored.
Warning [parser_urdf.cc:2777] Error Code 19: Msg: urdf2sdf: link[base] is not modeled in sdf.[Err] [UserCommands.cc:1145] Error Code 17: Msg: A model must have at least one link.
[Err] [UserCommands.cc:1145] Error Code 17: Msg: A model must have at least one link.
[Err] [UserCommands.cc:1145] Error Code 25: Msg: FrameAttachedToGraph error: scope context name[] does not match __model__ or world.
```

## Convert URDF to SDF
```bash
$ gz sdf -p urdf/mycobot_320_m5_2022_gazebo.urdf > urdf/mycobot_320_m5_2022_gazebo.sdf
Error [parser_urdf.cc:3348] Unable to call parseURDF on robot model
Error Code 40: Msg: Failed to parse the URDF file after converting to SDFormat.
Error: SDF parsing the xml failed.
```

## Try again
```bash
$  gz sim empty.sdf
$ gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_gazebo/urdf/mycobot_320_m5_2022_gazebo.urdf", name: "mycobot_320_pi"'
# Service call timed out -  if empty.sdf is not started
[Err] [UserCommands.cc:1145] Error Code 40: Msg: Failed to parse the URDF file after converting to SDFormat.
[Err] [UserCommands.cc:1145] Error Code 1: Msg: Unable to read file: [/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_gazebo/urdf/mycobot_320_m5_2022_gazebo.urdf]

$ gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 --req 'sdf_filename: "/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_gazebo/mycobot_320_gazebo.xarco", name: "mycobot_320_pi"'
Service call timed out

# CHECK URDF
$  check_urdf /home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_gazebo/urdf/mycobot_320_m5_2022_gazebo.urdf
Error:   Unable to parse component [${pi/2}] to a double (while parsing a vector value)
         at line 114 in ./urdf_parser/src/pose.cpp
Error:   Could not parse visual element for Link [base_link]
         at line 453 in ./urdf_parser/src/link.cpp
Error:   effort value (${effort}) is not a valid float
         at line 144 in ./urdf_parser/src/joint.cpp
Error:   Could not parse limit element for joint [joint2_to_joint1]
         at line 454 in ./urdf_parser/src/joint.cpp
Error:   joint xml is not initialized correctly
         at line 233 in ./urdf_parser/src/model.cpp
ERROR: Model Parsing the xml failed

$ check_urdf <(xacro ./mycobot_gazebo/mycobot_320_gazebo.xacro)
robot name is: mycobot_280
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  link1
            child(1):  link2
                child(1):  link3
                    child(1):  link4
                        child(1):  link5
                            child(1):  link6
                                child(1):  link6_flange
                                    child(1):  gripper_base
                                        child(1):  gripper_left2
                                        child(2):  gripper_right2
                                        child(3):  gripper_right3
                                            child(1):  gripper_right1
                                        child(4):  gripper_left3
                                            child(1):  gripper_left1

$ pwd
/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_ros2_description/urdf

$ check_urdf mycobot_320_m5_2022.urdf 
robot name is: firefighter
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  link1
        child(1):  link2
            child(1):  link3
                child(1):  link4
                    child(1):  link5
                        child(1):  link6

# Zaista ovaj URDF nema intertial property, ali ima .xacro
$ gz sim empty.sdf
Warning [parser_urdf.cc:2774] Error Code 19: Msg: urdf2sdf: link[base_link] has no <inertial> block defined. Please ensure this link has a valid mass to prevent any missing links or joints in the resulting SDF model.
Error Code 19: Msg: urdf2sdf: [1] child joints ignored.
Error Code 19: Msg: urdf2sdf: [1] child links ignored.
Warning [parser_urdf.cc:2777] Error Code 19: Msg: urdf2sdf: link[base_link] is not modeled in sdf.[Err] [UserCommands.cc:1145] Error Code 17: Msg: A model must have at least one link.
[Err] [UserCommands.cc:1145] Error Code 17: Msg: A model must have at least one link.
[Err] [UserCommands.cc:1145] Error Code 25: Msg: FrameAttachedToGraph error: scope context name[] does not match __model__ or world.

$ gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_ros2_description/urdf/mycobot_320_m5_2022.urdf", name: "mycobot_320_pi"'
data: true


#  Opet sa XACRO fielom gzim ne radi
$ check_urdf <(xacro ./mycobot_320_gazebo.xacro) # radu
$ gz sim empty.sdf
ror [parser_urdf.cc:3348] Unable to call parseURDF on robot model
[Err] [UserCommands.cc:1145] Error Code 40: Msg: Failed to parse the URDF file after converting to SDFormat.
[Err] [UserCommands.cc:1145] Error Code 1: Msg: Unable to read file: [/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_gazebo/mycobot_320_gazebo.xacro]
$ gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_gazebo/mycobot_320_gazebo.xacro", name: "mycobot_320_pi"'

# Konvertuj xacro u URDF
$ xacro ./mycobot_320_gazebo.xacro > anel_robot.urdf
$ check_urdf anel_robot.urdf
$ colon build && source install/setup.bash
$ gz sdf -p anel_robot.urdf > anel_robot.sdf
$ gz sim anel_robot.sdf
# Meshes doesn't exist
[Err] [Server.cc:204] Error Code 14: Msg: Parser configurations requested resolved uris, but uri 
[file:///home/anel/GitHub/pick_and_place_ros/anel_ws/install/mycobot_ros2_description/share/mycobot_ros2_description/meshes/mycobot_280/base_link.dae] could not be resolved.
      ///home/anel/GitHub/pick_and_place_ros/anel_ws/install/mycobot_ros2_description/share/mycobot_ros2_description/mesh/mycobot_280/base_link.dae]
      ///home/anel/GitHub/pick_and_place_ros/anel_ws/install/mycobot_ros2_description/share/mycobot_ros2_description/mesh/base_link.dae]
gz sim anel_robot.sdf
$ ls /home/anel/GitHub/pick_and_place_ros/anel_ws/install/mycobot_ros2_description/share/mycobot_ros2_description/
hook  launch  mesh  package.bash  package.dsv  package.ps1  package.sh  package.xml  package.zsh  rviz  urdf
$ ls /home/anel/GitHub/pick_and_place_ros/anel_ws/install/mycobot_ros2_description/share/mycobot_ros2_description/mesh/
base.dae  link1.dae  link2.dae  link3.dae  link4.dae  link5.dae  link6.dae

```
## Questions:
4. How to download gazebo packages?
