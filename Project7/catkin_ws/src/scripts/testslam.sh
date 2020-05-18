#!/bin/sh
xterm  -e  " roslaunch my_turtle turtlebot3_home_world.launch" &
sleep 10
xterm  -e  " roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping " &
sleep 5
xterm  -e  " roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch "