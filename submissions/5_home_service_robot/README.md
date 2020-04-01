# Home Service Robot Project

## Used packages
- my_robot
  - my custom package from previous projects
  - only use the maps and worlds folders
- turtlebot
  - meta package for using turublebot
- turtlebot_simulator
  - launch gazebo
  - load the world in my_robot/worlds/
  - load AMCL demo for localization
- turtlebot_interactions
  - launch rviz
  - load a customized config file that has the Markers displays (```turtlebot_interactions/turtlebot_rviz_launchers/rviz/navigation_with_marker.rviz```)
- pick_objects
  - navigate the robot to the specified goals
  - publish "pick_status" topic to broadcast the pickup state in a custom message
- add_markers
  - show a marker at a specified position
  - subscribe "pick_status" topic
  - depending on the values in "pick_status" message, show/hide the marker
