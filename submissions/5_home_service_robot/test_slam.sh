#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/workspace/catkin_ws/src/map/daichi_simple.world
xterm -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
#xterm -e "roslaunch my_robot world_final.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch "
