# orm-ros
ROS 2 Work space for ORM-1 Open Robotic Manipulator

This project is a clone of the repository: 

https://github.com/jasonhillier/we_r2_moveit_config 

With the model updated to match the kinematics of ORM-1 Open Robotic Manipulator: 

https://www.telemetrybalkan.com/orm-1-2/ 

# Prerequisites
 - ROS Noetic http://wiki.ros.org/noetic
 - MoveIt https://moveit.ros.org/install/

# Launch 
 - create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
 - Next, git clone this repository in the workspace/src directory
 - Run `catkin_make`
 - Launch the demo: `roslaunch we_r2_moveit_config demo.launch`

# ORM-1 Hardware & Library
In order to apply the elaborated joint angles to the real hardware - install and launch the python ORM-1 Library:

https://github.com/sergei-nntu/orm/

the Arduino firmware and STL files for mechanics are available there as well
