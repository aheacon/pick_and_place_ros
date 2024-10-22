# Learn Gazebo
- There are 2 ways
1. classic way - `gazebo` https://classic.gazebosim.org/
2. new way (ignition) - `ros_gz` https://gazebosim.org/docs/latest/tutorials/

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

- Download the model
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

## Questions
- Cannot move `caster` around build_robot
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