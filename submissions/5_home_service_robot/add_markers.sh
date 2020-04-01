#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/workspace/catkin_ws/src/map/daichi_simple.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/robond/workspace/catkin_ws/src/my_robot/maps/map.yaml

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers" &

# show marker
xterm -e "rostopic pub /pick_status pick_objects/pick_status '{did_reach_first_goal: 0, did_reach_second_goal: 0, pick_x: -6, pick_y: 4, drop_x: 1, drop_y: 0}'" &
sleep 5
xterm -e "rostopic pub /pick_status pick_objects/pick_status '{did_reach_first_goal: 1, did_reach_second_goal: 0, pick_x: -6, pick_y: 4, drop_x: 1, drop_y: 0}'" &
sleep 5
xterm -e "rostopic pub /pick_status pick_objects/pick_status '{did_reach_first_goal: 1, did_reach_second_goal: 1, pick_x: -6, pick_y: 4, drop_x: 1, drop_y: 0}'"
