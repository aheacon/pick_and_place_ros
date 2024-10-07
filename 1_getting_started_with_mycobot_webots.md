## 1. Getting started with ROS2 and myCobot with Webots simulator

### Prerequisites
- Make sure you have installed [ROS2](0_ros2_getting_started.md)
- Make sure that `underlay` workspace is properly sourced `source /opt/ros/humble/setup.bash`
- Make sure you have installed [Webots](webots/0_webots_getting_started.md)

### 1.1 Based on
1. [mycobot_webots](https://github.com/elephantrobotics/mycobot_webots)

### 1.2 Steps to build
<details>
  <summary>
    Create directory and start
  </summary>

  - Get the directory
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
  - Start the `webots`
    ```bash
    $ webots
    ```
  - Load the `world` file from `webots` GUI: `File -> Open World` and choose `mycobot_webots/mycobot_280_pi/worlds/mycobot_280_pi.wbt`
  - We must get the following
  ![alt text](webots/image_docs/myCobot_webots_initial.png)
</details>



### 3. Setting up robot simulation
- Based on [Setting up a robot simulation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)
- Based on [Setting up a robot simulation (Advanced)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html)

- Usage of `webots_ros2` package as interface between ROS2 and Webots and `webots_ros2_driver` subpackage.
  - Other examples of [webots_ros2](https://github.com/cyberbotics/webots_ros2/wiki/Examples)
- Steps:
```bash
# 1. Create the package/node with dependencies
$ ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_robot_driver my_package --dependencies rclpy geometry_msgs webots_ros2_driver
# 2. Add launch file
$ cd my_package && mkdir launch && mkdir worlds
# list files
$ cd .. && tree
.
└── my_package
    ├── launch
    ├── LICENSE
    ├── my_package
    │   ├── __init__.py
    │   └── my_robot_driver.py
    ├── package.xml
    ├── resource
    │   └── my_package
    ├── setup.cfg
    ├── setup.py
    ├── test
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    └── worlds
# 3. Add world example file
$ wget https://docs.ros.org/en/humble/_downloads/5ad123fc6a8f1ea79553d5039728084a/my_world.wbt -P my_package/worlds/
# Check
$ ls my_package/worlds/
my_world.wbt
# 4. Extend webots_ros2_driver by creating custom plugin
#    this is ROS node == robot controller, than will use Webots API
#    One can use `webots_ros2_control` that creates interface with `ros2_control`
#    Note: webots_ros2_universal_robot uses `webots_ros2_driver` in launch file for robot nodes 
#          and `webots_ros2_control` for `resource/ur5e_with_gripper.urdf`
# 5. Create the plugin in my_robot_driver.py
# 6. Create the `my_robot.urdf` file in resource folder with name `MyRobotDriver` (name of class in plugin/node)
#    If plugin would need input parameter we would add `parameterName` tag in resource
# 7. Create launch file
#    More details in https://github.com/cyberbotics/webots_ros2/wiki/References-Nodes
$ cd launch && touch robot_launch.py # add content
# 8. Edit setup.py
# 9. Start simulatino
$ ros2 launch my_package robot_launch.py
# 10. Publish the messages
$ ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
# 11. To avoid obstaces we need node with dinstsance sensors
#     <device> tag in URDF file of robot loads the plugins of webots_ros2_driver
# 12. add new node and update according to the Advanced tutorial
$ touch my_package/my_package/obstacle_avoider.py # add content
# 13. Build and check (it should work)
$ colcon build && source install/setup.bash
$ ros2 launch my_package robot_launch.py 
```
- Example for [UR45e](https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots)
  - Used `launch` files for:
    1. simulator `robot_world_launch.py` (workaround can be started manually)
    2. to spawn robot nodes `robot_nodes_launch.py` (workaround can be staretd manually - but how to connect to webots?)
    - Location of files
    ```bash
    $ ll - /opt/ros/humble/share/webots_ros2_universal_robot/launch
    ```

### 3.1 Setting up robot simulation for mycobot
- [URDF file of 280 pi](mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_pump.urdf)
- 
### 2. Troubleshooting

#### 2.1 Opening the world file
- When `webot` starts and world file from `mycobot` is used, robot will fail, with errors

  <details>
    <summary>
      Error while starting the webots with world file
    </summary>

    ```bash
    INFO: my_controller: Starting controller: python3 -u my_controller.py
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 26685 contact points.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 34464 contact points.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 76 contact points.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 44 contact points.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 11446 contact points.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 65 contact points.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 93 contact points.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 12 contact points.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 12 contact points.
    WARNING: Contact joints between materials 'default' and 'default' will only be created for the 10 deepest contact points instead of all the 12 contact points.
    ```
  </details>

- **FIX**
  - To fix the issue `Pause` and `Reset Simulation` (restore to initial state) or `Tools->Preferences->General->Startup mode` and choose `Pause` over `Real time`

