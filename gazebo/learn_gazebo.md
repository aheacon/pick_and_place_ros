# Learn Gazebo
- There are 2 ways
1. classic way - `gazebo` https://classic.gazebosim.org/
2. new way (ignition) - `ros_gz` https://gazebosim.org/docs/latest/tutorials/

## Gazebo ignition
- Versions:
Ionic (9.x.x), Harmonic (8.x.x), Garden (7.x.x), and Fortress (6.x.x) 
```bash
$ gz sim shapes.sdf # empty.sdf
$ gz sim --version
Gazebo Sim, version 8.6.0
# One can have multiple versions
$ gz sim --force-version 9.0.0 shapes.sdf
Version error: I cannot find this command in version [9.0.0].
```
[Tutorial 1](https://gazebosim.org/docs/latest/gui/)
- PLugins:
1. `Component inspector`
2. `Entity Tree`
3. `Grid config`
4. `Transform control`
5. `View angle`
6. `Align tool`
7. `Resource Spawner`
  - resource owner `openrobotics`


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
[SDFormat](http://sdformat.org/) (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

[GitHub sdf file](https://github.com/gazebosim/docs/blob/master/ionic/tutorials/building_robot/building_robot.sdf)