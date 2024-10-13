## 1. Getting started with ROS2 and myCobot with Webots simulator

### Prerequisites
TODO

### 1.1 Based on
1. [mycobot_ros2 tutorial](https://github.com/automaticaddison/mycobot_ros2)
2. [automaticaddison tutorials myCobot 280](https://automaticaddison.com/tutorials/)

### 1.2 Steps to build
<detail closed>
  <summary>
    Installation process
  </summary>

  Extensions for VSC: Microsoft ROS extension, XML Redhat, python, autoDocstring, 
  
  <details>



### Know-how
- URDF stored in package `mycobot_description` (see [this](https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/))
- [Naming and organizing projects](https://automaticaddison.com/naming-and-organizing-packages-in-large-ros-2-projects/)
  - Based on [best practice](https://ros.org/reps/rep-0144.html)
  - Suffix for purpose of the package `_description` 
    - URDF file and mesh files
  - [metapackage](https://wiki.ros.org/Metapackages), list of dependencies to other packages. (name of roboto `my_cobot`)
  - One package one purpose
- `xacro` files in `_description`
  - blueprints for [URDF](https://automaticaddison.com/how-to-load-a-urdf-file-into-rviz-ros-2/) files
  - using macros and variables to simplify complex robot descriptions.
  - Before a ROS tool or component can use the information in a XACRO file, it must first be processed (translated) into a URDF file. This step allows for the dynamic generation of robot    descriptions based on the specific configurations defined in the XACRO file. (/meshes)
    - Mesh files are used to visually represent the geometric shape of the robot parts in simulations and visualizations.
      These files are typically in formats such as `STL` (Stereo Lithography – .stl) or `COLLADA` (.dae).
    - Mesh files define the 3D shapes of components such as links,
      which are visualized in tools like [RViz (ROS visualization tool)](https://automaticaddison.com/how-to-load-a-urdf-file-into-rviz-ros-2/)
      and [Gazebo (a robot simulation environment)](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/).
      TODOs ^
    - 
- To visualize URDF file in RVIS
  - red - x, green - y, blue - z
```bash
$ sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial
$ ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_280_urdf.xacro
```


## Mycobot 320 URDF file create
```bash
# URDF files package
$ ros2 pkg create --build-type ament_python --license Apache-2.0 --description &quot;mycobot ros2 URDF files, mesh files&quot; mycobot_ros2_description
# based package (metadata) exec depends on _description package (see package.xml)
$ ros2 pkg create --build-type ament_python --license Apache-2.0 --description "mycobot ros2 anel" mycobot_ros2 --dependencies mycobot_ros2_description
# Build
$ colcon build
$ source ./install/setup.sh
# Check
$ ros2pkg |grep mycobot_ros2
mycobot_ros2
mycobot_ros2_description

# Create URDF file
$ mkdir mycobot_ros2_description/mycobot_ros2_description/urdf
$ touch mycobot_ros2_description/mycobot_ros2_description/mycobot_320_urdf.xacro
$ mkdir -p mycobot_ros2_description/mycobot_ros2_description/meshes/mycobot_320
$ git clone https://github.com/elephantrobotics/mycobot_ros2/tree/humble/mycobot_description/urdf/mycobot_320_m5_2022
$ ll ../ros2_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_320_m5_2022/
total 29064
drwxrwxr-x  2 anel anel     4096 Oct  5 15:52 ./
drwxrwxr-x 20 anel anel     4096 Oct  5 15:52 ../
-rwxrwxr-x  1 anel anel 14544765 Oct  5 15:52 base.dae*
-rwxrwxr-x  1 anel anel  2456810 Oct  5 15:52 link1.dae*
-rwxrwxr-x  1 anel anel  3525724 Oct  5 15:52 link2.dae*
-rwxrwxr-x  1 anel anel  4088651 Oct  5 15:52 link3.dae*
-rwxrwxr-x  1 anel anel  2160196 Oct  5 15:52 link4.dae*
-rwxrwxr-x  1 anel anel  2526859 Oct  5 15:52 link5.dae*
-rwxrwxr-x  1 anel anel   408646 Oct  5 15:52 link6.dae*
-rw-rw-r--  1 anel anel     5091 Oct  5 15:52 mycobot_320_m5_2022.urdf

$  cp -r ../ros2_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_320_m5_2022/ mycobot_ros2_description/mycobot_ros2_description/meshes/
# They should be in parent folder
$ mv mycobot_ros2_description/mycobot_ros2_description/urdf/ mycobot_ros2_description/
$ mv mycobot_ros2_description/mycobot_ros2_description/meshes/ mycobot_ros2_description/
# Create xacro file in urdf folder
$ touch ./urdf/mycobot_320.xacro

# Visualise URDF
$  sudo apt-get install ros-humble-urdf-tutorial

# Change
# <mesh filename="file://$(find mycobot_ros2_description)/meshes/mycobot_320_m5_2022/base.dae"/>
# to 
# <mesh filename="package://mycobot_ros2_description/meshes/mycobot_320_m5_2022/base.dae"/>
```
- We can find URDFs here [mycobot](https://github.com/elephantrobotics/mycobot_ros/tree/noetic/mycobot_description/urdf), or [mycobot ros2](https://github.com/elephantrobotics/mycobot_ros2/tree/humble/mycobot_description)


- Running `ros2 urdf_tutoril` not working
```bash
$ ros2 launch urdf_tutorial display.launch.py model:=/home/anel/GitHub/pick_and_place_ros/anel_ws/mycobot_ros2_description/urdf/mycobot_320.xacro 
```
- Mesh file doesn't exist in `share`
```bash
$ ls install/mycobot_ros2_description/share/
ament_index  colcon-core  mycobot_ros2_description
$ ls install/mycobot_ros2_description/share/mycobot_ros2_description/
hook  package.bash  package.dsv  package.ps1  package.sh  package.xml  package.zsh
```

- `chown -x *.dae && chown a+x *.dae`

Unofficial https://github.com/Tiryoh/mycobot_ros/tree/main
