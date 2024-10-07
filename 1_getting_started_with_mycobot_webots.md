## 1. Getting started with ROS2 and myCobot with Webots simulator

### Prerequisites
- Make sure you have installed [ROS2](0_ros2_getting_started.md)
- Make sure that `underlay` workspace is properly sourced `source /opt/ros/humble/setup.bash`
- Make sure you have installed [Webots](0_webots_getting_started.md)

### 1.1 Based on
1. [mycobot_webots](https://github.com/elephantrobotics/mycobot_webots)

### 1.2 Steps to build
```bash
$  mkdir ros2_ws_webots && cd ros2_ws_webots
$ git clone https://github.com/elephantrobotics/mycobot_webots.git
$ $ ll mycobot_webots/
total 32
drwxrwxr-x 7 anel anel 4096 Oct  7 11:19 ./
drwxrwxr-x 3 anel anel 4096 Oct  7 11:19 ../
drwxrwxr-x 8 anel anel 4096 Oct  7 11:19 .git/
drwxrwxr-x 6 anel anel 4096 Oct  7 11:19 mercury_a1/
drwxrwxr-x 6 anel anel 4096 Oct  7 11:19 mercury_b1/
drwxrwxr-x 6 anel anel 4096 Oct  7 11:19 mycobot_280_pi/
-rw-rw-r-- 1 anel anel 1074 Oct  7 11:19 README.md
drwxrwxr-x 2 anel anel 4096 Oct  7 11:19 res/
```