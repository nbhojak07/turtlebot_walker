# Overview
Implementation of a simple walker algorithm like a Roomba robot vacuum cleaner.The robot moves forward until it reaches an obstacle (but not colliding), then rotates in place until the way ahead is clear, then moves forward again and repeat.

## Dependencies 
The following dependencies are required to run this package:
1. ROS Melodic 
2. catkin
3. Ubuntu 18.04
4. turtlebot packages

## Standard install via Commandline 
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/nbhojak07/turtlebot_walker.git
cd ..
catkin_make

```
## Running the Code
Open the terminal and run the following code:

Terminal #1
```
roscore 
```
Terminal #2
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker turtlebot_walker.launch 
```
To run the simulation alongwith recording the bag file, run the following command:
```
roslaunch turtlebot3_walker turtlebot3_walker.launch rosbag_record:=true
```
## Inspecting the bag file
```
cd catkin_ws
source devel/setup.bash
cd src/turtlebot_walker/results/
rosbag info turtlebot_walker.bag
```
## Playing the rosbag file
```
cd <path to repository>/results
rosbag play walker.bag
```
