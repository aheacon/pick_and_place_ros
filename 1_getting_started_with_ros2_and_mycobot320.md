## 1. Getting started with ROS2 and myCobot

### Prerequisites
- Make sure you have installed [ROS2](0_ros2_getting_started.md)
- Make sure that `underlay` workspace is properly sourced `source /opt/ros/humble/setup.bash`
- Make sure you have installed `sudo apt install ros-humble-joint-state-publisher-gui`

### 1.1 Based on
1. [mycobot ros2 tutorial](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/12.2-ROS2/12.2.1-ROS2%E7%9A%84%E5%AE%89%E8%A3%85.html)
2. [mycobot_ros2 GitHub](https://github.com/elephantrobotics/mycobot_ros2)
3. [myCobot YT](https://www.youtube.com/watch?v=-Jo_IJ8RaXc)
4. [GitBook myCobot](https://docs.elephantrobotics.com/docs/gitbook-en/)
5. [myCobot Isaac Gym](https://www.hackster.io/Elephant-Robotics-Official/mycobot-gripping-task-reinforcement-learning-with-isaac-gym-5621db)

### 1.2 Steps to build
- Note this is not needed since we are going to create our own package ⛔
```bash
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone https://github.com/elephantrobotics/mycobot_ros2 -b humble --depth 1
$ cd ..
# there is no resdep files (see `Optional and troubleshooting`)
```
<details closed>
<summary> Check the URDF file </summary>
- Use `check_urdf` utility to check parsing of XML file of `.urdf` file
```bash
$ check_urdf urdf/mycobot_320_pi_2022.urdf 
robot name is: mycobot_320pi
---------- Successfully Parsed XML ---------------
root Link: base has 1 child(ren)
    child(1):  link1
        child(1):  link2
            child(1):  link3
                child(1):  link4
                    child(1):  link5
                        child(1):  link6

```
- Use `urdf_tutorial` ROS package to start the `.urdf` file - NOT important  - bug 🐛 ❌ fixed 💻
```bash
$ sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial
$ ros2 launch urdf_tutorial display.launch.py model:=/home/anel/GitHub/pick_and_place_ros/ros2_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_320_m5_2022/mycobot_320_m5_2022.urdf
[INFO] [launch]: All log files can be found below /home/anel/.ros/log/2024-10-13-10-05-18-532420-anel-18124
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [18126]
[INFO] [joint_state_publisher_gui-2]: process started with pid [18128]
[INFO] [rviz2-3]: process started with pid [18130]
[robot_state_publisher-1] [INFO] [1728806718.885357636] [robot_state_publisher]: got segment base
[robot_state_publisher-1] [INFO] [1728806718.885452194] [robot_state_publisher]: got segment link1
[robot_state_publisher-1] [INFO] [1728806718.885460760] [robot_state_publisher]: got segment link2
[robot_state_publisher-1] [INFO] [1728806718.885466230] [robot_state_publisher]: got segment link3
[robot_state_publisher-1] [INFO] [1728806718.885470919] [robot_state_publisher]: got segment link4
[robot_state_publisher-1] [INFO] [1728806718.885475608] [robot_state_publisher]: got segment link5
[robot_state_publisher-1] [INFO] [1728806718.885480186] [robot_state_publisher]: got segment link6
[joint_state_publisher_gui-2] [INFO] [1728806719.467995466] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1728806719.475423519] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1728806719.539369567] [joint_state_publisher]: Centering
[rviz2-3] [INFO] [1728806719.600012480] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1728806719.600101668] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-3] [INFO] [1728806719.648563810] [rviz2]: Stereo is NOT SUPPORTED
```
Picture of error:
![alt text](error_urdf_tutorial_package.png)

- To solve it we must change name of the parent `base` to `base_link` in `urdf` file
```bash
# Launch file https://github.com/ros/urdf_tutorial/tree/ros1/launch
$ ros2 launch urdf_tutorial display.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):
    'gui':
        Flag to enable joint_state_publisher_gui. Valid choices are: ['true', 'false']
        (default: 'true')

    'rvizconfig':
        Absolute path to rviz config file
        (default: LocalVar('FindPackageShare(pkg='urdf_tutorial') + 'rviz' + 'urdf.rviz''))

    'model':
        Path to robot urdf file relative to urdf_tutorial package
        (default: '<launch.substitutions.path_join_substitution.PathJoinSubstitution object at 0x742f329dc490>')

    'jsp_gui':
        Flag to enable joint_state_publisher_gui. Valid choices are: ['true', 'false']
        (default: 'true')

    'rviz_config':
        Absolute path to rviz config file
        (default: LocalVar('FindPackageShare(pkg='urdf_launch') + 'config' + 'urdf.rviz''))

    'urdf_package':
        The package where the robot description is located

    'urdf_package_path':
        The path to the robot description relative to the package root

```
Bug fixed
![alt text](solved_error_urdf_tutorial_package.png)
</details>

- Build package (from `package.xml`) with all its dependency and show console output while building
- Alternative to create `COLCON_IGNORE` file in the directory to skip building of specific package (didn't test this)
```bash
# Build target package
$ colcon build --packages-up-to mycobot_320 --event-handlers console_direct+
```

  <details closed>
  <summary> Result of build </summary>
  <br>

  ```bash
  # Output
  Installing camera_display script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing detect_marker script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing follow_display script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing following_marker script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing listen_real script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing listen_real_of_topic script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing opencv_camera script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing simple_gui script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing slider_control script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  Installing teleop_keyboard script to /home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320/lib/mycobot_320
  writing list of installed files to '/home/anel/GitHub/pick_and_place_ros/ros2_ws/build/mycobot_320/install.log'
  Finished <<< mycobot_320 [1.03s]           

  Summary: 4 packages finished [10.9s]

  # Check directories
  $ ll
  total 24
  drwxrwxr-x 6 anel anel 4096 Oct  5 16:21 ./
  drwxrwxr-x 4 anel anel 4096 Oct  5 15:50 ../
  drwxrwxr-x 6 anel anel 4096 Oct  5 16:22 build/
  drwxrwxr-x 6 anel anel 4096 Oct  5 16:22 install/
  drwxrwxr-x 3 anel anel 4096 Oct  5 16:21 log/
  drwxrwxr-x 3 anel anel 4096 Oct  5 15:51 src/
  ```
  </details>



- Install `mycobot` dependency
```bash
$ pip install pymycobot --upgrade
# Successfully installed msgpack-1.0.8 packaging-24.1 pymycobot-3.5.3 pyserial-3.5 python-can-4.4.2 wrapt-1.16.0
```
- Source `overlay` (and add it to `~/.bashrc`) to configure ROS2 environment
```bash
$ source install/setup.bash
```

### 1.3 Start robot
- To start the robot, execute launch file
```bash
$ ros2 launch mycobot_320 test.launch.py
```

- To visiualize the robot start `rviz2`
```bash
$ rviz2
```

#### 1.3.1 Results
<details closed>
<summary> Result picture of launching the robot </summary>
<br>

![alt cobot](myCobotRviz.png)
</details>

<details closed>
<summary> Result log of launching the robot </summary>
<br>

```bash
$ ros2 launch mycobot_320 test.launch.py
[INFO] [launch]: All log files can be found below /home/anel/.ros/log/2024-10-05-16-52-33-598578-anel-27721
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [27723]
[INFO] [joint_state_publisher_gui-2]: process started with pid [27725]
[INFO] [rviz2-3]: process started with pid [27727]
[robot_state_publisher-1] [INFO] [1728139953.868858638] [robot_state_publisher]: got segment base
[robot_state_publisher-1] [INFO] [1728139953.868971142] [robot_state_publisher]: got segment link1
[robot_state_publisher-1] [INFO] [1728139953.868985669] [robot_state_publisher]: got segment link2
[robot_state_publisher-1] [INFO] [1728139953.868999516] [robot_state_publisher]: got segment link3
[robot_state_publisher-1] [INFO] [1728139953.869009174] [robot_state_publisher]: got segment link4
[robot_state_publisher-1] [INFO] [1728139953.869019614] [robot_state_publisher]: got segment link5
[robot_state_publisher-1] [INFO] [1728139953.869027940] [robot_state_publisher]: got segment link6
[rviz2-3] [INFO] [1728139954.542204804] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1728139954.542377282] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-3] [INFO] [1728139954.613967999] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1728139954.786821033] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1728139954.798732051] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1728139954.884827043] [joint_state_publisher]: Centering
```
</details>


### 1.4 Real robot connection 💁
- TODO 💁 for later ⚒️
- Check [this](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/12.2-ROS2/12.2.4-rviz%E4%BB%8B%E7%BB%8D%E5%8F%8A%E4%BD%BF%E7%94%A8/) for serial connection 
- Check [Basic Learning, unbox robot](https://www.youtube.com/watch?app=desktop&v=WPDMkrLcMIE)
- Check [other](https://www.youtube.com/watch?app=desktop&v=yhFuFvxx8aI)


### 1.5 Troubleshooting ⛑️

#### 1.5.1. Rosdep missing  for mycobot
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

#### 1.5.2. Launch file problem
```bash
$ ros2 launch mycobot_320 test.launch.py
[INFO] [launch]: All log files can be found below /home/anel/.ros/log/2024-10-05-16-49-37-621332-anel-27323
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'joint_state_publisher_gui' not found, searching: ['/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_320', '/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_communication', '/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_interfaces', '/home/anel/GitHub/pick_and_place_ros/ros2_ws/install/mycobot_description', '/opt/ros/humble']"
```
- To fix it install package `sudo apt install ros-humble-joint-state-publisher-gui`


### 1.6. Other literature 📖
[1] [ROS2 Humble tutorial](https://docs.ros.org/en/humble/Tutorials.html)

