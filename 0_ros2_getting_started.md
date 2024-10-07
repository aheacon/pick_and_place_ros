# ROS tutorials

### ROS2 installation 22.04 (jammy)
<details>
  <summary>
    Installing ROS2 Humble (LTS) on 22.04
  </summary>

  Based on [this](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [this for Gazebo](https://robotics.stackexchange.com/questions/23554/how-to-install-gazebo-to-use-along-with-ros2-humble-hawksbill-on-ubuntu-22-04) (I didn't used [this for Gazebo](https://gazebosim.org/docs/latest/ros_installation/))
  Following [gazebosim.org](https://gazebosim.org/docs/all/install_ubuntu/)
  ```bash
  $ sudo apt update && sudo apt install curl -y
  $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  $ sudo apt update
  # Extended base package with simulation: `ros-humble-simulation` (not recommended)
  # Desktop install `ros-humble-desktop`: OS, RViz, demos, tutorials.
  # Desktop install full `ros-humble-desktop-full` should contain Gazebo, like `ros-humble-simulation`
  # For allm above see http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages
  $ sudo apt install ros-humble-desktop-full
  # Got error, although is possible to install libmysqlclient-dev
  Some packages could not be installed. This may mean that you have
  requested an impossible situation or if you are using the unstable
  distribution that some required packages have not yet been created
  or been moved out of Incoming.
  The following information may help to resolve the situation:

  The following packages have unmet dependencies:
  libmariadb-dev : Conflicts: libmysqlclient-dev but 8.0.39-0ubuntu0.22.04.1 is to be installed
                   Breaks: libmysqlclient-dev but 8.0.39-0ubuntu0.22.04.1 is to be installed
  E: Error, pkgProblemResolver::Resolve generated breaks, this may be caused by held packages.

  $ sudo apt remove 
  # Need to get 209 MB of archives.
  # After this operation, 1,256 MB of additional disk space will be used.
  # Do you want to continue? [Y/n] Y
  $ dpkg -s libmysqlclient-dev
  Package: libmysqlclient-dev
  Status: install ok installed
  # Seems mysql packaged installed with ros2
  $ grep libmysql /var/log/dpkg.log
  2024-09-25 18:20:14 install libmysqlclient-dev:amd64 <none> 8.0.39-0ubuntu0.22.04.1
  # Check ROS2 installation
  $ ls /opt/ros/humble/
  # Update env
  $ source /opt/ros/humble/setup.bash
  #  Test
  $ ros2 run demo_nodes_cpp talker
  [INFO] [1727281856.916109841] [talker]: Publishing: 'Hello World: 1'
  $ ros2 run demo_nodes_py listener
  [INFO] [1727281877.931552916] [listener]: I heard: [Hello World: 22]

  # Check gazebo
  $ dpkg -s ros-humble-ros-ign-gazebo-demos
  Package: ros-humble-ros-ign-gazebo-demos
  Status: install ok installed
  Description: Shim package to redirect to ros_gz_sim_demos.
  bin  cmake  include  lib  local  local_setup.bash  local_setup.sh  _local_setup_util.py  local_setup.zsh  opt  setup.bash  setup.sh  setup.zsh  share  src  tools
  # ROS2 Humble: Gazebo Fortress should be availbe under packages.ros.org ( gazebosim says we need gz-harmonic see at the end from packages.osrfoundation.org)
  # $ ROS_DISTRO=humble sudo apt-get install ros-${ROS_DISTRO}-ros-gz
  # $ apt list --installed|grep gazebo
  # libignition-gazebo6-dev/jammy,now 6.16.0-2~jammy amd64 [installed,automatic]
  # libignition-gazebo6-plugins/jammy,now 6.16.0-2~jammy amd64 [installed,automatic]
  # libignition-gazebo6/jammy,now 6.16.0-2~jammy amd64 [installed,automatic]
  # ros-humble-ros-ign-gazebo-demos/jammy,now 0.244.15-1jammy.20240901.084435 amd64 [installed,automatic]
  # ros-humble-ros-ign-gazebo/jammy,now 0.244.15-1jammy.20240729.004702 amd64 [installed,automatic]
  # Not working above, try with gazebosim.org and Gazebo Harmonic binaries
  $ sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  $ sudo apt-get update
  $ sudo apt-get install gz-harmonic
  # Now it works
  $ gz sim
  ```
</details>


### ROS1 installation 20.4 (focal)
<details>
  <summary>
    Installing ROS1 Noetic (LTS) on 22.04 (you can **skip it**)
  </summary>

  - Install ROS 1 (can be done on 20.04) - [see](https://cyberbotics.com/doc/guide/tutorial-9-using-ros)
  - Note 22.04 ROS Noetic is not supported for the Ubuntu 22.04 (no `jammy` here http://packages.ros.org/ros/ubuntu/dists/)
  ```bash
  $ export ROS_DISTRO=noetic # add in .bashrc
  # this works for focal, but not for jammy
  $ sudo sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  $ sudo apt install curl
  # needs update
  $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  # Warning: apt-key is deprecated. Manage keyring files in trusted.gpg.d instead (see apt-key(8)).
  #OK
  $ sudo apt-get update
  $ sudo apt-get install ros-noetic-desktop-full ros-noetic-moveit # takes time, get a coffee :)
  $ sudo apt-get install python3-rosdep
  $ sudo rosdep init
  $ rosdep update
  ```
- Alternatively see [here for 22.04](https://github.com/GNDeSouza/ROS-Noetic-and-Gazebo-in-Ubuntu-22.04) or [here](https://www.reddit.com/r/ROS/comments/yrzqmt/ros_noetic_on_ubuntu_2204/)
  ```bash
  #  Remove old dependencies
  sudo apt-get remove '.gazebo.' '.sdformat.' '.ignition-math.' '.ignition-msgs.' '.ignition-transport.' '.ignition.'
  sudo apt autoremove

  # First install gazebo
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable lsb_release -cs main" > /etc/apt/sources.list.d/gazebo-stable.list'
  sudo wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update

  # Remove ogre 2.1 from the list of $ogre
  sudo vi /tmp/dependencies.sh

  # Install dependecies
  GAZEBO_MAJOR_VERSION=11 ROS_DISTRO=noetic . /tmp/dependencies.sh
  sudo echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\' | xargs sudo apt-get -y install
  ...TODO have not tried
  ```
</details>


## ROS learnings
- Based on [ROS2 tutorials](https://docs.ros.org/en/crystal/Tutorials.html)


### CLI tools

- 1.[Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html):
  - `colcon_ws`
    - official default ROS2 workspace (location on system where we develope ROS2)
  - core ROS2 worskpace (`underlay`)
  - add `/opt/ros/humble/setup.bash` to `bashrc`
  - add [ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) `export ROS_DOMAIN_ID=99` (adsd in bash)
  - update `export ROS_LOCALHOST_ONLY=1`

- 2. [Turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html):
  - `ros2` tool 
  - `rqt` tool GUI for ros2 => can be done on CLI
    - `sudo apt install '~nros-humble-rqt*'`
    - `rqt --force-discover` if `Plugin->Services` are not visible
    - couldn't find services for `turlte1` (node) to `spawn` new turtle
      - I thought the problem is with DOMAIN_ID, but wasn't - just open new terminals or restart
    - After spawn there is service `/turtle2/...`
    - Open new `teleop` node for `turtle2` in new terminal
      - `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel`
      - stop node: `q`
  - `ros2 pkg executables turtlesim` - check if package is installed 
    - `sudo apt install ros-humble-turtlesim` 
  - Start node `turtlesim_node`:
    - `ros2 run turtlesim turtlesim_node`
    - close it with `CTRL+C`
  - Start new node `turtle_teleop_key` to control the turtle in first node `turtlesim_node`:
    - `ros2 run turtlesim turtle_teleop_key`
  - Check nodes:
  ```bash
  ros2 node list
  ros2 topic list -v # see published and subscribed
  ros2 service list
  ros2 action list
  ```
- 3. [Understand nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
  - ROS2 graph
  - `ros2 run <package_name> <executable_name>`
    -  `ros2 run turtlesim turtlesim_node`
  - Remapping - reassign node properties
    - `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle`
  - `ros2 node info <>`,etc

- 4. [Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
  - Bus for nodes to exchange the messages
  - `rqt_graph` (or in `rqt` `Plugins->Introspection->NodeGraph`)
  - `ros2 topic pub <topic_name> <msg_type> '<args>'`
    - `args` data in YAML syntax
    - `ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"` 
    - `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"` 
    - Waiting on matching subscriptinos
      - `ros2 topic pub -w 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"` 
    - `ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'`
      - `ros2 topic pub /reference sensor_msgs/msg/TimeReference '{header: "auto", time_ref: "now", source: "dumy"}'`

- 5. [Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
  - While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.
  - `ros2 service list`
  - Service types
    - `ros2 service type <service_name>`
      - `std_srvs/srv/Empty`
        - The Empty type means the service call sends no data when making a request and receives no data when receiving a response.
    - To find the service name based on type
      - `ros2 service find std_srvs/srv/Empty`
    - `ros2 interface show std_srvs/srv/Empty`, `ros2 interface show turtlesim/srv/Spawn`
    - `ros2 service call <service_name> <service_type> <arguments>`

- 6. [Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
  - Node settings
  - `ros2 param list`
    - `use_sim_time` for each node
  - `ros2 param get <node_name> <parameter_name>`
  - `ros2 param set <node_name> <parameter_name> <value>`
  - `ros2 param dump <node_name>`
  - `ros2 param load <node_name> <parameter_file>`
  - `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`

- 7. [Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
  - They consist of three parts: a goal, feedback, and a result.
  - Similar to services, except action can be canceled
  - An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.
  - Example `/turtle_teleop_key` node
  - `ros2 node info /teleop_turtle`
  - Action have types
    - `ros2 action list -t`
  - `ros2 action info /turtle1/rotate_absolute`
  - `ros2 interface show turtlesim/action/RotateAbsolute` - based on type get how is created
  - `ros2 action send_goal <action_name> <action_type> <values>`

- 8. [rqt_console](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)
  - Logger 
  - `ros2 run turtlesim turtlesim_node --ros-args --log-level WARN` - set level

- 9. [Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
  - Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
  - `ros2 launch turtlesim multisim.launch.py`
  - [Launch file tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

- 10. [Recording and playing back data](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
  - `ros2 bag` is a command line tool for recording data published on topics in your system.
  - It accumulates the data passed on any number of topics and saves it in a database.
  - You can then replay the data to reproduce the results of your tests and experiments.
  - Recording topics is also a great way to share your work and allow others to recreate it.
  - `ros2 bag record /turtle1/cmd_vel`
    - Creates new folder `rosbag2_2024_10_05-15_34_46`
    - `ros2 bag  info rosbag2_2024_10_05-15_34_46` info about the folder
    - ` ros2 bag play rosbag2_2024_10_05-15_34_46`

### Client libraries

  1. [Creating the workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
    - A workspace is a directory containing ROS 2 packages
      - `underlay`
        - ROS 2 installation workspace in `/opt/ros/humble/setup.bash`
        - It provides build dependencies and must be sourced
      - `overlay`
        - Our project: A secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending, or “underlay” 
        - It will only add pacakges available in overlay to environment
        - It takes precedance over the contents of the `underlay`, it gets prepended to the pathy
    - Best practice is to create a new directory for every new workspace and to put any packages in your workspace into the src directory
    - If package contains `COLCON_IGNORE` file it is not built
    - Example of cloning the repo `git clone https://github.com/ros/ros_tutorials.git -b humble` in `src`
    - Resolve dependencies
      - Best practice is to check for dependencies every time you clone.
      - From root directory `rosdep install -i --from-path src --rosdistro humble -y`
      - Packages declare their dependencies in the `package.xml` file (you will learn more about packages in the next tutorial). 
      - [rosdep tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
    - Build the workspace
      - Relies that [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) is installed
        - `sudo apt install python3-colcon-common-extensions`
        - It does out of source builds
        - It will create folders `install`, `build`, `log`
        - Overlay must be sourced `. install/setuyp.bash` before using executables
        - Build type supported `cmake`, `ament_cmake`, `ament_python`
        - Uses `package.xml` specification
        - Create own package `ros2 pkg create` based on template
        - Follow [colcon-argcomplete setu[p]](https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion)
          - Note this is not package on 22.04 (already included as part of Debian package `python3-colcon-argcomplete`)
          ```bash
          $ ls /usr/share/colcon_argcomplete/hook/
          colcon-argcomplete.bash  colcon-argcomplete.zsh
          ```
        - Build specific file (using ``) or create empty COLCON_IGNORE file in the directory to skip building of specific package
        - Avoid configure and build tests pass `--cmake-args -DBUILD_TESTING=0`
          - To run the tests `colcon test`
          - To run single test `colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG`
        - There is `colcon_cd` (not used by me)
      - From root `colcon build --packages-up-to <name_of_package> --event-handlers console_direct+`

2. [Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
  - A package is an organizational unit for your ROS 2 code.
  - Package creation in ROS 2 uses ament as its build system and colcon as its build tool.
  - You can create a package using either CMake or Python
  - Cmake
    - `CMakeLists.txt`, file that describes how to build the code within the package
    - `include/<package_name>`, directory containing the public headers for the package
    - `package.xml`,  file containing meta information about the package
    - `src` directory containing the source code for the package
    ```bash
    my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
    ```
    - Command: `ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>`
  - Python
    - `package.xml`,  file containing meta information about the package
    - `resource/<package_name>` marker file for the package
    - `setup.cfg` is required when a package has executables, so ros2 run can find them
    - `setup.py` containing instructions for how to install the package
    - `<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py
    ```bash
    my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
    ```
    - Command: `ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>`
    - Simple executable in the package created with `--node-name`: `ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package`
  - A single workspace can contain as many packages as you want, each in their own folder
  - Example of creating the package
  ```bash
  $ mkdir -p ros2_ws/src && cd src
  $ ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node anel_package
  ```
  <details closed>
  <summary> Result of package creation </summary>
  <br>

  ```bash
  $ ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node anel_package
  going to create a new package
  package name: anel_package
  destination directory: /home/anel/GitHub/private/phd_anel/phd/anel_phd/webots_tutorial/ros2_ws/src
  package format: 3
  version: 0.0.0
  description: TODO: Package description
  maintainer: ['anel <anelhusakovic88@gmail.com>']
  licenses: ['Apache-2.0']
  build type: ament_python
  dependencies: []
  node_name: my_node
  creating folder ./anel_package
  creating ./anel_package/package.xml
  creating source folder
  creating folder ./anel_package/anel_package
  creating ./anel_package/setup.py
  creating ./anel_package/setup.cfg
  creating folder ./anel_package/resource
  creating ./anel_package/resource/anel_package
  creating ./anel_package/anel_package/__init__.py
  creating folder ./anel_package/test
  creating ./anel_package/test/test_copyright.py
  creating ./anel_package/test/test_flake8.py
  creating ./anel_package/test/test_pep257.py
  creating ./anel_package/anel_package/my_node.py
  ```
  </details>
  - Note: Noted that `package.xml` of myCobot has additional tags `<build_depend>`, `<build_export_depend>` and `exec_depend`
    - Tag name ending with `_depende` is where your `package.xml` would list its dependencies on other packages, for `colcon` to search for.
  - `colcon build` builds the workspace (that may have multiple packages)
  ```bash
  $ colcon build --packages-select anel_package
    Starting >>> anel_package
    Finished <<< anel_package [1.05s]          
    Summary: 1 package finished [1.42s]
  ```
  - To execute the `anel_package/my_node.py` run `ros2 run anel_package my_node`

  3. [Write simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
    - Create the new package in `src`: `ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub`
    - Go to the package folder `src/py_pubsub/py_pubsub` and call `wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py`
    - `rclpy` library is used
    - Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough
    - Interestingly `msg.String()` uses `__slot__`([see](https://stackoverflow.com/questions/472000/usage-of-slots))
    - Configuration settings:
      1. Add dependencies to node's import in `package.xml`
      - Dependencies
      ```xml
      <exec_depend>rclpy</exec_depend>
      <exec_depend>std_msgs</exec_depend>
      ```
      - Without adding I could still build and run the `ros2 run package topic`
      - After adding
      2. Add an entry point for `ros2 run` in `setup.py`: `'talker = py_pubsub.publisher_member_function:main',`
      3. Check `setup.cfg` - nothing should be changed, tells `setuptools` to put executables in `lib`, where `ros2 run` will look for them
    - Build the package
    ```bash
    $ rosdep install -i --from-path src --rosdistro humble -y
    #All required rosdeps installed successfully
    $ colconbuild py_pubsub && . install/setup.sh
    $ find .|grep talker
    ./install/py_pubsub/lib/py_pubsub/talker
    ```
    - Rule of thumb: **Topic name** and **Message type** **MUST MATCH** to be used by the publisher and subscriber to allow them to communicate
    - Several ways to write Publish/Subscribe in Python see [ros2/examples](https://github.com/ros2/examples/tree/humble/rclpy/topics)

  4. [Writing a simple service and client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
    - Note [jazzy](ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces) documentation is for ROS2
    - When nodes communicate using services, the node that sends a request for data is called the client node, and the one that responds to the request is the service node. The structure of the request and response is determined by a .srv file.
    - Build the package and proceed:
    ```bash
    $ ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
    $ rosdep install -i --from-path src --rosdistro humble -y
    $ cd src
    $ colcon build --packages-select py_srvcli
    $ source install/setup.bash
    $ ros2 run py_srvcli service
    [INFO] [1728208748.510683924] [minimal_service]: Incoming request
    a: 2 b: 3
    $ ros2 run py_srvcli client 2 3
    [INFO] [1728208748.521106484] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
    # if we change the name of client from minimal_client_async to X nothing will happen
    # [INFO] [1728208824.535251837] [x]: Result of add_two_ints: for 2 + 3 = 5
    ```
    - `--dependencies` => adds to `package.xml`
    - `example_interface` => example_interfaces is the package that includes the [.srv](https://github.com/ros2/example_interfaces/blob/humble/srv/AddTwoInts.srv) file you will need to structure your requests and responses
    ```python
    # From module example_interfaces, module srv import class
    from example_interfaces.srv import AddTwoInts # there are SetBool, Trigger also
    ```
    - Rule of thumb: **Service node** and **Client node** MUST HAVE the SAME TYPE AND NAME
    - Note client is using `futures` - async programing
    - Check more about [async vs sync calls](https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html)

  5. [Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
    - Define custom interface files (.msg and .srv) and use them with Python 
    - Interface package can only be, a CMake package, but this doesn’t restrict in which type of packages you can use your messages and services. 
    - The `.msg` and `.srv` files are required to be placed in directories called `msg` and `srv` respectively.
    - Steps
    ```bash
    $ ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces_pkg
    $ cd tutorial_interfaces_pkg && mkdir msg srv
    $ cd msg && touch Num.msg && echo "int64 num" > Num.msg
    $ touch Sphere.msg && printf "geometry_msgs/Point center\nfloat64 radius" > Sphere.msg
    # 3 request integers and 1 respond sum
    $ cd ../srv && touch AddThreeInts.srv && printf "int64 a\nint64 b\nint64 c\n---\nint64 sum" > AddThreeInts.srv
    # build with colcon
    # check that interface exists
    $ ros2 interface show tutorial_interfaces_pkg/msg/Num
    int64 num
    $ ros2 interface show tutorial_interfaces_pkg/msg/Sphere
    $ ros2 interface show tutorial_interfaces_pkg/srv/AddThreeInts
    ```
    - Interface rely  on `rosidl_default_generators` package (CmakeLists.txt) for generating language-specific code,
      we must declare `build tool` dependency (`<buildtool_depend>`)/
    - `<exec_depend>` (runtime, execution-stage dependency) - `rosidl_default_runtime` 
    - `<member_of_group>` - `rosidl_interface_packages`name of dependecny group

  6. [Implementing custom interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
    - [Basic interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
    - Skip for now using `ament_cmake` instead of `ament_cmake_pyton` TODO for later


  7. [Using parameters in a class (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
    - When making your own nodes you will sometimes need to add parameters that can be set from the launch file.
    - `ros2 param describe /minimal_param_node my_parameter`
    - `ros2 param list`
    - `ros2 param set /minimal_param_node my_parameter earth`
    - Add `launch` directory and `launch` file with changes in `setup.py`
      - `ros2 launch python_parameters python_parameters_launch.py`

  8. [Using ros2doctor to identify issues](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html)
    - ros2doctor is part of the ros2cli package
    - I still see [this problem](https://github.com/ros2/ros2cli/pull/860/files), related to [this](https://github.com/ros2/ros2cli/issues/806)

  9. [Creating and using plugins (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
    - TODO it is for C++


### Intermediate
  1. [Managing Dependencies with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
    - rosdep is a dependency management utility that can work with packages and external libraries.
    - `apt-get install python3-rosdep`
    - The `package.xml` is the file in your software where `rosdep` finds the set of dependencies.
    - The dependencies in the `package.xml` file are generally referred to as `“rosdep keys”`.
    - Pure Python packages generally don’t have a build phase, so should never use `<depend>` tag (build and runtime) and should use `<exec_depend>` instead
    - rosdep works by retrieving the central index (`rosdistro`)on to your local machine so that it doesn’t have to access the network every time it runs.
    - Type of dependencies:
      - ROS packages
        - List of all released ROS packages for ROS distribution - [rosdistro/humble](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml)
      - System dependencies
        - [rosdep/base.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) contains the apt system dependencies
        - [rosdep/python.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml) contains the Python dependencies
    - Usage:
    ```bash
    $ sudo rosdep init
    $ rosdep update
    $ rosdep install --from-paths src -y --ignore-src
    ```

  2. [Creating an action](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
    - Skip for now
  3. [Writing an action server and client (Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
    - Skip for now
  4. [Composing multiple nodes in a single process](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)
    - Skip for now

  5. [Launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
    - ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
    - The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described.
    - **Unique namespaces** allow the system to start two nodes without node name or topic name conflicts.
    - Launch it from directory `$ ros2 launch turtlesim_mimic_launch.py`
    - Packages with `launch` file should have `<exec_depend>ros2launch</exec_depend>`
    - Substitutions can be used in arguments to provide more flexibility when describing reusable launch files.
    - Substitutions are variables that are only evaluated during execution of the launch description and can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html) - TODO later

  6. [tf2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
    - Skip for now

  7. [Testing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
    - Skip for now
  
  8. [URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
    - TODO for later - building R2D2

  9. [Rviz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html)
    - RViz is a 3D visualizer for the Robot Operating System (ROS) framework.
    - [User guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
    - [Documentation](https://wiki.ros.org/rviz2)

### Advanced
  1. [Simulators](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Simulation-Main.html)
  - [Webots](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Simulation-Webots.html)
  - [Gazebo](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
    - Install see [package](https://github.com/gazebosim/ros_gz)
    - To be able to communicate our simulation with ROS 2 you need to use a package called `ros_gz_bridge`
      - `sudo apt-get install ros-humble-ros-ign-bridge`
    - World `visualize_lidar.sdf`
    - Example
    ```
    $ ign gazebo -v 4 -r visualize_lidar.sdf
    $ ign topic -l
    ```
- Other:
  - See `ros_tutorials` folder in https://github.com/an3l/ros_tutorials
  - See [linux tutorial](https://www2.cs.sfu.ca/~ggbaker/reference/unix/)
  - See [ros2/examples](https://github.com/ros2/examples/tree/humble/rclpy/topics)



## ROS2 tutorials and mycobot
- Based on [this](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/12.2-ROS2/12.2.1-ROS2%E7%9A%84%E5%AE%89%E8%A3%85.html)
- Dependency `ros2` and `moveIT2` (`sudo apt-get install ros-humble-moveit`)
- `mycobot_ros2` ROS2 package http://github.com/elephantrobotics/mycobot_ros2
- `pymycobot` API dirver library  https://github.com/elephantrobotics/pymycobot `pip install pymycobot --upgrade`