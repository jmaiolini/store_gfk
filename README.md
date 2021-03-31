# store_gfk

## Description
This repository uses already available ROS robots and adds some cameras on them for retail applications.
## Usage
### TIAGo-base
* Follow the official installation of the robot [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation) and install it into the ROS workspace.
* Clone this repository in the same workspace and build both
* Comment and add/modify the line as shown
```bash
<!--  <xacro:include filename="$(find pmb2_description)/urdf/base/base.urdf.xacro"/> -->
<xacro:include filename="$(find store_gfk)/robot_description/base.urdf.xacro"/>
```
in the file 
```bash
~/<your_workspace>/src/pmb2_public_ws/src/pmb2_robot/pmb2_description/urdf/base/base_sensors.urdf.xacro
```
This enables to use our robot model consisting into the TIAGo-base + 4 additional RGBD cameras mounted on top.
* Use the following command to spawn the robot within the store map
```bash
roslaunch store_gfk store_gfk.launch
```

### Mapping (classic step)
The mapping step uses gmapping with the configurations of Tiago-Base.
* Launch the mapping step: spawn robot and the store
```bash
roslaunch store_gfk map_store_gfk.launch public_sim:=true
```
* Then move the robot around and explore all the store by typing the following in a new terminal
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=nav_vel
```
I prefer this solution with respect to the classic key_teleop. To install this package simply run
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
```
* Once done, to save the map in the right folder run
```bash
rosservice call /pal_map_manager/save_map "directory: '../../../<your_workspace>/src/store_gfk/map/<store_name>/<map_name>'"
```
### Mapping (custom)
Given that we have a perfectly known map of the store we should use this information.
First of all you need to generate a .dae file for the world. Place this file under ```/store_gfk/models/<store_name>/meshes/``` and create/modify the world under ```/store_gfk/worlds/```. Launch files have to be changed accordingly.
**Remarks** 
1. It is good practise to have a collision all over the boundaries of the store. This avoids the possibility for the global planner to plan a path for the desired waypoint which is outside the store (it can happen only if you map with the classic step though). 

2. Given 1, if the robot is spawn at (0,0) as well as the map (by deafault) they will collide. As a workaround, simply shift the world of a desired offset by the parameter ```<pose>-5.0 -5.0 0.0 0.0 0.0 0.0</pose> ``` inside the world file. This however modifies only the gazebo part and does not shift the map for rviz that uses the package ```map_server```. To have coherency, after generating the custom map as will be explained next, in the .yaml file you have to also shift the pose of the same amount.

#### Create a map
Given the top-view image corresponding to the model .dae of before, the most straightforward way to generate a map is to crop this image to have exaclty the store. To do this, some useful functions can be found in ```/scripts/map_generator.py```

### Navigation
The navigation step
* Launch the navigation step: spawn robot and the store together with the map already created
```bash
roslaunch store_gfk nav_store_gfk.launch 
```
The lost argument of the tutorial can be omitted (deafult is false). If set to true it will spawn the robot in an unknown position (you can modify it in the nav_store_gfk.launch file). If you set it to be lost, you have to follow the instructions for the navigation step [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Navigation/Localization). Otherwise you can simply send a goal to the robot and it will find a path to it.

If you want to load a specific map of a specific store you have to provide them as command line parameters as e.g.

Alternatively you can modify these two parameters in the launch file directly by setting the desired default map.
```bash
roslaunch store_gfk nav_store_gfk.launch store_name:=edeka map_name:=map
```
#### Sending goals
You can run the follow
```bash
rosrun store_gfk wpoints_generator.py <trajectory_filename>
```
TODO add the other required args to the py script
to let
Alternatively it is possible to enable the trajectory generation directly from the previous step launch file by enable it or by command line or in the launch itself (param ```generator_on:=true```). This will cause the robot to immediately start to follow the desired waypoints. In the same launchfile it is possible to change all the parameters for the script

### Additional notes
##### 1
The package is structured as follow:
- The store map created with the Mapping step should follow the convention to be placed in ```../maps/<store_name>/<map_name>```
- The same goes for trajectory files, that should be placed in ```../trajectories/<store_name>/``` as ```.json``` and numbered. As example see the edeka folders
##### 2
Pal Robotics uses a custom Rviz plugin for displaying covariance ellipses that is out-of-date. To avoid errors on the simulations (this however does not impact the simulation itself, just visualization) you should install an additional package in the workspace and build it
```bash
cd ~/<your_workspace>/src/pmb2_public_ws/src/
git clone https://github.com/pal-robotics/rviz_plugin_covariance.git
cd ~/<your_workspace>
catkin build
```

