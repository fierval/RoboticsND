#!/bin/sh
xterm  -e  " roslaunch my_turtle turtlebot3_home_world.launch" &
sleep 10
xterm  -e  " roslaunch my_turtle turtlebot3_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers_node" &
sleep 5
xterm -e " rosrun pick_objects pick_objects_node" &
