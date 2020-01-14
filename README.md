# pepper-ros-stack
Stack of ROS packages for Pepper robot, mostly Python based.

## Code
This repo contains code from the following repositories:

 - https://github.com/ros-naoqi/naoqi_driver
 - https://github.com/ros-naoqi/pepper_robot
 - https://github.com/ros-naoqi/pepper_meshes

All repos are added as subtrees and eventual changes in the original repos (which are not likely to happen but one can always hope) can be easily pulled.

## Dependencies
In order to run, apart from having ROS installed (works on Ubuntu 18.04 and ROS Melodic), you have to have *pynaoqi* installed. 

When you try to launch *pepper_full_py.launch* from the *pepper_bringup* just look up the error messages to install whatever needs to be installed (rgbd_launch, camera_info_manager_py will probably generate those errors). Most of those you can install with *apt*. Still not sure why we need *rgbd_launch* package.

## Running 
Just launch *pepper_gmapping.launch* to set up Pepper for mapping and then use some node to drive with joystick (just launch it in *pepper_robot* namespace or remap the topic).
