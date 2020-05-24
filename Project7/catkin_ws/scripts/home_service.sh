#!/bin/sh
t="7"
if [ $# -gt 0 ]; then
  t="$1"
fi

xterm  -e  " roslaunch my_turtle turtlebot3_home_world.launch" &
sleep $t
xterm  -e  " roslaunch my_turtle turtlebot3_navigation.launch" &
sleep $t
xterm -e " rosrun add_markers add_markers_node" &
sleep $t
xterm -e " rosrun pick_objects pick_objects_node" &
