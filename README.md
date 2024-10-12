# Project
- Pick and place project with Elephant robot in ROS

# System
- Ubuntu `jammy` 22.04
- ROS2  (Humble LTS)
- Elephant 320

# Scope 1
Tasks that will be done
1. <span style="color:green;">Getting started with ROS2 and myCobot</span>
2. Create environment of 3 objects (hammer 🔨, screwdriver 🪛 and scissors ✂️) 
3. Use camera at the top and capture pictures
4. Implement CNN
5. Integrate CNN with rule based algorithm
6. DRL


## 1. Getting started with ROS2 and myCobot

- Refer to [Getting started with ROS2 and myCobot 320](1_getting_started_with_ros2_and_mycobot320.md) (**done** ✅)
- Refer to [Getting started with ROS2 and myCobot with Gazebo simulator][1_getting_started_with_mycobot_gazebo](**in progress** 🔴)
- Refer to [Getting started with ROS2 and myCobot with Webots simulator](1_getting_started_with_mycobot_webots.md) (**cannot be done with ROS2** :x:)
  - Eveything else is blocked since there is no way to simulate interface (no `launch` file between ROS2 and Webots). 
    - Options:
    1. Use `ubuntu 20.4` (`ROS1`) - issue: VM settings
    2. Use `ROS2` and `Gazebo` - issue: learning `Gazebo`
    3. Postpone this scope and switch to [Scope 2 (`NVIDIA Isaac`)](#scope2)


# <a name="scope2"></a>Scope 2
### Blocker :x:
1. Getting started with `NVIDIA Isaac Sim` (**currently cannot be done on my hardware**:x:)
  - Needs better GPU and Isaac Sim is not rendering


