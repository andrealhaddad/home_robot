#!/bin/sh
# export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/map/office.world
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_robot/src/map/simple_world/simple_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
