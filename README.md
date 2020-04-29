# Udacity's Robotics Software Engineer nanaodegree
#### Course website: [Robotics Software Engineer](www.udacity.com/course/robotics-software-engineer--nd209)

There are a total of 5 projects, starting off easy with ROS essentials and building a world in Gazebo, and 
ending with making a mobile robot perform a pickup-dropoff in an unknown map using the ROS navigation stack.

## Projects
All project folders in this repository are complete on their own and require no additional downloads.
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
Uses the ```amcl``` package for Adaptive Monte Carlo Localization for the robot to localize itself in the map

### Project 4: Map My World
Loop closures are used for generating a map of the environment

### Project 5: Home Service Robot
This is where all the previous projects come together. The Adaptive Monte-Carlo Localization algorithm 
can be seen localizing the turtlebot in the map as it traverses from the 
starting point to its destination via the pick-up waypoint.
