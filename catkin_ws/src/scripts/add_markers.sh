#!/bin/sh

path_catkin_ws="/home/workspace/catkin_ws"

xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/UdacityOffice.world" &

sleep 10

xterm  -e  " cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/map.yaml" &

sleep 10

xterm  -e  " cd ${path_catkin_ws} &&  source devel/setup.bash && roslaunch turtlebot_rviz_launchers  view_navigation.launch" &

sleep 10


xterm  -e " cd ${path_catkin_ws} && source devel/setup.bash && rosrun test_add_markers test_add_markers"


