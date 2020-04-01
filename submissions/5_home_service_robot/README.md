# Home Service Robot Project
This is the final project in the Robotics Software Nanodegree.

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

## Notes
- This project is developed on the Udacity virtual machine on my local Windows 10 PC.
- Environment vaiables in ```*.sh``` files are in absolute paths. You may need to modify those depending on your environment.
- Defining a custom message and make publishing/subscribing work was a pain. The official documentation helped some, but I needed to dig into the Q&A and googling a lot in order to make it work. I'm not sure how this usecase is common, but that seems a pretty straightfoward solution for me, so it was frustrating that I needed to spend too much time on this.
