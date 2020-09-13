# home_robot

Home robot: a simple robot system capable of mapping and localization in closed environment based on SLAM/Gmapping and AMCL. 
Two robots could be launched: 
- Turtlebot2 with Kobuki Base and Kinect sensor. Runs on ROS Kinetic.
- Custom robot with hokuyo laser sensor, kinect (not used for this project), a front camera, imu sensor, and differential_drive_controller.

## References
This project utilizes the following ROS packages. Clone the packages to src file for update versions.

- [gmapping](http://wiki.ros.org/gmapping)
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

## Shell launch files:

All launch files are in script folder.

### launch.sh:
launches gazebo with a specific world for simulation, gmapping to create the map, RVIZ for viewing the mapping progress, and keyboard teleop to move the robot.
```
#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_robot/src/map/simple_world/simple_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
```
Move the robot around until a map is generated. Slow teleop controls is recommended for high fidelity map. After a map is gnerated, in a terminal run
```
rosrun map_server map_saver -f 'location/file_name'
```
A YAML and PGM files are generated. Several worlds and their maps are prepared in the src/map folder.

### test_navigation.sh

To test if the map and navigation works, test_navigation.sh launches a gazebo simulation and AMCL package (for probabilistic localization). AMCL is launched in combination with the 
- mapserver: publishes the map generated using the launch file
- move_base: reads position goals and trajectory planner.

```
#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_robot/src/map/simple_world/simple_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/home_robot/src/map/simple_world.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch"
```
In the case of turtlebot: AMCL uses a nodlet generates a fake-laser-scan out of the Kinect sensor.
Navigation stack is not yet fully configured on custom robot. Global planner is not yet fully tuned to generate the most optimum paths. Local planner does a good job avoiding obstacles but not optimized for lowest cost paths.

### pick_object and markers:
In the home-robot package, the robot can navigate towards a goal to pickup an object, then navigates to a second location for drop-off.
#### pick_objects.sh
This files launchs a CPP file that sends goals to the navigation stack. The robot will move based on the generated trajectory towards each point, and wait for 5 seconds for pick-up and drop-off.
A <geometry_msgs/Pose2D.h> target is advertised for each goal sent to the navigation stack.
Unlimited number of goals (drop off and pickup locations) can be sent using a vector of x,y,angle locations.
```
#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_robot/src/map/simple_world/simple_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/home_robot/src/map/simple_world.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " rosrun pick_objects pick_objects"
```

#### add_markers
This CPP files subscribes to the amcl locations and pick-up and drop-off targets. Based on the location of the robot and load status of the robot, it will advertise a marker object to RVIZ to draw a blue box or remove it.
The robot is always defined as empty load and ready to pick-up, and the second goal will automatically be a drop-off. The pickup and drop-off operations are alternating. NO multi-drop-off or multi-pick-up mode is available.
```
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_robot/src/map/simple_world/simple_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/home_robot/src/map/simple_world.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e " rosrun pick_objects pick_objects" &
xterm -e " rosrun add_markers add_markers"
```

