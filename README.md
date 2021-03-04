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

### Mapping
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

### Additional notes
Pal Robotics uses a custom Rviz plugin for displaying covariance ellipses that is out-of-date. To avoid errors on the simulations (this however does not impact the simulation itself, just visualization) you should install an additional package in the workspace and build it
```bash
cd ~/<your_workspace>/src/pmb2_public_ws/src/
git clone https://github.com/pal-robotics/rviz_plugin_covariance.git
cd ~/<your_workspace>
catkin build
```

