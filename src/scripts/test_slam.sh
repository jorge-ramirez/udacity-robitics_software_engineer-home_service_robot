#!/bin/sh

WORLD_PATH="$(pwd)/../worlds/myworld.world"

echo "Executing turtlebot_world launch file"
xterm -e "export TURTLEBOT_GAZEBO_WORLD_FILE=$WORLD_PATH; roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

echo "Executing gmapping_demo launch file"
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5

echo "Executing view_navigation launch file"
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

echo "Executing keyboard_teleop launch file"
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &

