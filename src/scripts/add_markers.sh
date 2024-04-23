#!/bin/sh

WORLD_PATH="$(pwd)/../worlds/myworld.world"
MAP_PATH="$(pwd)/../map/map.yaml"

echo "Executing turtlebot_world launch file"
xterm -e "export TURTLEBOT_GAZEBO_WORLD_FILE=$WORLD_PATH; roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

echo "Executing amcl_demo launch file"
xterm -e "export TURTLEBOT_GAZEBO_MAP_FILE=$MAP_PATH; roslaunch turtlebot_gazebo amcl_demo.launch " &

sleep 5

echo "Executing view_navigation launch file"
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

echo "Executing add_markers node"
xterm  -e "rosrun add_markers add_markers" &

