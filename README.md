# Project
- Pick and place project with Elephant robot in ROS

# System
- Ubuntu `jammy` 22.04
- ROS2  (Humble LTS)
- Elephant 320

# Scope
Tasks that will be done
1. <span style="color:green;">Getting started with ROS2 and myCobot</span> (**done** ✅)
2. Create environment of 3 objects (hammer 🔨, screwdriver 🪛 and scissors ✂️) (**in progress** 🔴 )
3. Use camera at the top and capture pictures
4. Implement CNN
5. Integrate CNN with rule based algorithm
6. DRL


## 1. Getting started with ROS2 and myCobot

- Refer to [Getting started with ROS2 and myCobot 320](1_getting_started_with_ros2_and_mycobot320.md) (**done** ✅)
- Refer to [Getting started with ROS2 and myCobot with Webots simulator](1_getting_started_with_mycobot_webots.md) (**cannot be done with ROS2** :x:)


## 2. Create environment of 3 objects (hammer 🔨, screwdriver 🪛 and scissors ✂️)
- **in progress** 🔴

# Other literature
[1] [ROS2 Humble tutorial](https://docs.ros.org/en/humble/Tutorials.html)

# Optional and troubleshooting

#### Rosdep missing  for mycobot
- This is step to check compatilbity before building (not needed for now).
```bash
$ rosdep install -i --from-path src --rosdistro humble -y
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
...
mycobot_280: Cannot locate rosdep definition for [actionlib]
mycobot_320: Cannot locate rosdep definition for [actionlib]
mycobot_320pi: Cannot locate rosdep definition for [actionlib]
...
```
- No fix needed

#### Launch file problem
```bash
$ ros2 launch mycobot_320 test.launch.py
[INFO] [launch]: All log files can be found below /home/anel/.ros/log/2024-10-05-16-49-37-621332-anel-27323
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'joint_state_publisher_gui' not found, searching: ['/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320', '/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_communication', '/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_interfaces', '/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_description', '/opt/ros/humble']"
```
- To fix it install package `sudo apt install ros-humble-joint-state-publisher-gui`