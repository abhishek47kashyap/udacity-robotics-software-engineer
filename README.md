# Udacity's Robotics Software Engineer nanaodegree
#### Course website: [Robotics Software Engineer](www.udacity.com/course/robotics-software-engineer--nd209)

There are a total of 5 projects, starting off easy with ROS essentials and building a world in Gazebo, and 
ending with making a mobile robot perform a pickup-dropoff in an unknown map using the ROS navigation stack.

All media files can be found in the [media](https://github.com/abhishek47kashyap/udacity-robotics-software-engineer/tree/master/media) folder.

## Projects
Project folders in this repository are complete on their own and require no additional downloads.
Contents of each project should be placed in the ```src``` folder of the catkin workspace. To avoid any possible conflicts,
it's advised to have a separate workspace for every project.

_Requirements_: **Ubuntu 16.04** and **ROS Kinetic**

_Dependencies_: listed in every project's README

Before getting started, open the terminal and execute this line:
```
sudo apt-get update && sudo apt-get upgrade -y
```

### Project 1: Build My World
An introduction to Gazebo: making models and creating worlds

### Project 2: Go Chase It!
ROS essentials, chasing a ball

### Project 3: Where Am I
Given a map generated  by pgm_creator, Adaptive Monte Carlo Localization for the robot to localize itself in the map

ROS package used: ```amcl```

### Project 4: Map My World
This project assumes known robot pose to map an environment. The robot uses camera data to recognize previously visited regions in the map using a graph-based SLAM approach known as Real-Time Appearance Based (RTAB) mapping (under the hood, bag-of-words is used for detecting loop closures). More information on this can be found [here](http://introlab.github.io/rtabmap/). 

ROS package used: ```rtabmap_ros```
Actual world in Gazebo             |  Map created by RTAB mapping
:-------------------------:|:-------------------------:
![](media/Project4%20media/Lworld.png)  |  ![](media/Project4%20media/Lshaped_world.png)
![](media/Project4%20media/small_warehouse_gazebo.png)  |  ![](media/Project4%20media/small_warehouse_world.png)

### Project 5: Home Service Robot
This is where all the previous projects come together. The Adaptive Monte-Carlo Localization algorithm 
can be seen localizing the turtlebot in the map as it traverses from the 
starting point to its destination via the pick-up waypoint. The package is represented by a blue marker.

![](media/Project5%20media/homeServiceRobot.gif)
