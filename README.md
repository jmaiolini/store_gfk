# store_gfk

## Description
This repository uses already available ROS robots and adds some cameras on them for retail applications.
## Usage
### TIAGo-base
* Follow the official installation of the robot [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation) and install it into the ROS workspace.
* Clone this repository in the same workspace
* Comment and modify the line as shown
```bash
<!--  <xacro:include filename="$(find pmb2_description)/urdf/base/base.urdf.xacro"/>-->
<xacro:include filename="$(find store_gfk)/robot_description/base.urdf.xacro"/>
```
in the file located at
```bash
~/<your_workspace>/src/pmb2_robot/pmb2_description/
```
* Use the following command to spawn the robot within the store map
```bash
roslaunch store_gfk store_gfk.launch
```

### Mapping
The mapping step uses gmapping with the configurations of Tiago-Base.
* Launch the mapping step: spawn robot and the store


* Then move the robot around and explore all the store by typing the following in a new terminal
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=nav_vel
```
remark: i prefer this solution with respect to the classic key_teleop.


