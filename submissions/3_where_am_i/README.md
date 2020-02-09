# Where am I project

## How to launch
```roslaunch my_robot world.launch```

This will also launch ```amcl.launch```

## Modified parameters
- amcl.launch
  - min_particles
  - max_particles
  - update_min_a
  - update_min_d


## Other notes
- Worked on my local machine (VMWare)
- AMCL launch errors needed to be addressed
  - Posted an [answer](https://knowledge.udacity.com/questions/38931#69871) to the Student Hub
- Modified the original map because one of the walls is interfering with the robot at the origin
- Map file had to be rotated -90 degrees to match with the gazebo world
