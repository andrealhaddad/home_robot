xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_robot/src/map/simple_world/simple_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/home_robot/src/map/simple_world.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e " rosrun pick_objects pick_objects" &
xterm -e " rosrun add_markers add_markers"
