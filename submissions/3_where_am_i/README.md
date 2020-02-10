# Where am I project

## How to launch
```roslaunch my_robot world.launch```

This will also launch ```amcl.launch```

## Modified parameters
### amcl.launch
I reduced the max_particles to minimize the computation for updating the particles.

I also reduced update_min_a and update_min_d to 1/10 of the default values so the update occurs more frequently. If the values are too small, it seems it became more unstable to localize, especially when the robot rotates itself at the same place (i.e. when I tap the 'm' key).

- ```max_particles: 2000``` (default: 5000)
- ```min_particles: default (100)```
- ```update_min_a: 0.02``` (default: 0.2)
- ```update_min_d: 0.052``` (default: PI/6 = 0.52358)

For the laser parameters, I tried to mimic the Hokuyo sensor as much as possible, which left ```laser_*_range``` parameters as default (```-1.0```). Then I tried to "degrade" the sensor by setting ```laser_z_*``` to both ```0.5```. The particles diverged with the defualt update_min_* parameters. That's the reason I increased the update frequency by reducing those parameters.

One thing I wasn't quite sure from the document is the ```laser_max_beam``` parameter. I left it as is.

- ```laser_min_range: default (-1.0)```
- ```laser_max_range: default (-1.0)```
- ```laser_max_beam: default (30)```
- ```laser_z_hit: 0.5``` (default: 0.95)
- ```laser_z_rand: 0.5``` (default: 0.05)

For the 4 ```odom_alpha*``` parameters, I kept them as is. When I increased all of them to ```0.5```, the particles are relatively spread out. When I set them to ```0.0``` and the robot tried to go back, it lost its position and the map jumped around. This happened when I increased the sensor update frequency too much. Because ```0.0``` means it expects no noise, it became too sensitive to the actual noises.

## Other notes
- Worked on my local machine (VMWare)
- AMCL launch errors needed to be addressed
  - Posted an [answer](https://knowledge.udacity.com/questions/38931#69871) to the Student Hub
- Modified the original map because one of the walls is interfering with the robot at the origin
- Map file had to be rotated -90 degrees to match with the gazebo world
