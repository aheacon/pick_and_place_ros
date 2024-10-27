## 1. Getting started with ROS2 and myCobot with Gazebo simulator

### Prerequisites
TODO

### 1.1 Based on
1. [mycobot_ros2 tutorial](https://github.com/automaticaddison/mycobot_ros2)
2. [automaticaddison tutorials myCobot 280](https://automaticaddison.com/tutorials/)

### 1.2 Steps to install
<details closed>
  <summary>
    Installation process for Gazebo [Harmonics & Fortress LTS]
  </summary>

  - The most important step is to:
  1. [Check platform and Gazebo version](https://gazebosim.org/docs/fortress/getstarted/)
    - In my case Ubuntu 22.04 => Gazebo Fortress (since using ROS2 Humble)
  2. [Check Dependencies ROS and Gazebo](https://gazebosim.org/docs/latest/ros_installation/)
    - ROS2 Humble (LTS) goes with GZ Fortress (LTS)
  - 2 main repositories that host Gazebo simulator and Gazebo libraries
  1. packages.ros.org
    - ROS2 Humble: Gazebo Fortress
  2. packages.osrfoundation.org - non ROS official Gazebo binary packages
    - Used to pair different ROS with differet Gazebo like Gazebo Harmonic
  - I have already installed from non-official ROS gazebo package
  ```bash
  # Install
  $ sudo apt install ros-humble-ros-gzharmonic
  # Uninstall
  $ sudo apt remove ros-humble-ros-gzharmonic && sudo apt autoremove
  ```
  - **DESIGN** After researching about classical and modern way I decided to use Gazebo Harmonic and ROS2 Humble
  - Install official ROS2 and Gazebo pair
  ```bash
  # Installation
  $ ROS_DISTRO=humble sudo apt-get install ros-${ROS_DISTRO}-ros-gz
  $ ign gazebo --version # instead of command gz sim , for Gazebo Harmonics
  Gazebo Sim, version 6.16.0
  # To start gazebo (verbosity 4, -r run simulatin on stasrt, and sdf file)
  $ ign gazebo -v 4 -r visualize_lidar.sdf
  # Configuration files
  $ l$ ls ~/.ignition/gazebo/6/
  gui.config     server.config  
  # Check tutorials/gazebo_sdf
  # Uninstall
  $ sudo apt remove ignition-fortress && sudo apt autoremove
  $ ROS_DISTRO=humble sudo apt-get remove ros-${ROS_DISTRO}-ros-gz && sudo apt autoremove && rm -rf ~/.ignition
  ```bash
  - Otherwise with non ROS official Gazebo pacakges (that are favorized by gazebosim.org)
    - Following installation for [Gazebo Fortress LTS](https://gazebosim.org/docs/fortress/install_ubuntu/) EOL 2026, since using ROS2 Humble (Ubuntu Jammy).
    - Extensions for VSC: Microsoft ROS extension, XML Redhat, python, autoDocstring, 
    ```bash
    $ cat /etc/apt/sources.list.d/gazebo-stable.list 
    deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main
  ```
  - [Gazebo 6 tutorials](https://gazebosim.org/api/gazebo/6/tutorials.html)
 </details>

### 1.3 Create ROS2 launch files
<details closed>
  <summary>
    ROS2 launch files
  </summary>

 </details>
