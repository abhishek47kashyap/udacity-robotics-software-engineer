#!/bin/sh

# Note that my native installation of Ubuntu already has the following lines added to the bashrc file:
#	source /opt/ros/kinetic/setup.bash
#	source /path/to/directory/devel/setup.bash




# TURTLEBOT_WORLD: deploys turtlebot in an environment
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"/home/abhishek47kashyap/Desktop/catkin_ws/src/map/L_shaped_world.world\"" &
sleep 15  #(takes quite some time to load)

# GMAPPING_DEMO: performs SLAM
xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 15

# VIEW_NAVIGATION: observe map in Rviz
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

# KEYBOARD_TELEOP: to manually control the robot using keyboard
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
