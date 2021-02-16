# store_gfk

## Description
This repository uses already available ROS robots and adds some cameras on them for retail applications.
## Usage
### TIAGo-base
* Follow the official installation of the robot [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation) and install it into the ROS workspace.
* Clone this repository in the same workspace
* Comment and modify the line as shown\
>`<!--  <xacro:include filename="$(find pmb2_description)/urdf/base/base.urdf.xacro"/>-->`
>`<xacro:include filename="$(find store_gfk)/robot_description/base.urdf.xacro"/>`\
in the file located at\
>`~/<your_workspace>/src/pmb2_robot/pmb2_description/`\
* Use the following command to spawn the robot within the store map\
>`roslaunch store_gfk store_gfk.launch`
