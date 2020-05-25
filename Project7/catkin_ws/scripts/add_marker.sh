#!/bin/sh
xterm  -e  " roslaunch my_turtle turtlebot3_home_world.launch" &
sleep 10
xterm  -e  " roslaunch my_turtle turtlebot3_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers_node" &
rostopic pub -1 /goal_state pick_objects/PointReached '{name: set, x: 3.0, y: 4.0}'
echo "sleeping for 2 sec"
sleep 2
rostopic pub -1 /goal_state pick_objects/PointReached '{name: pickup, x: 3.0, y: 4.0}'
echo "sleeping for 2 sec"sleep 2
rostopic pub -1 /goal_state pick_objects/PointReached '{name: set, x: -5.0, y: 1.0}'
echo "sleeping for 2 sec"
sleep 2
rostopic pub -1 /goal_state pick_objects/PointReached '{name: dropoff, x: -5.0, y: 1.0}'