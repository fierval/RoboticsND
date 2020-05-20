#!/bin/sh
xterm  -e  " roslaunch my_turtle turtlebot3_home_world.launch" &
sleep 10
xterm  -e  " roslaunch my_turtle turtlebot3_navigation.launch" &
