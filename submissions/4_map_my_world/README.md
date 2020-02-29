# Map My World project

2.29.2020

## Mapping result
See "rtabmap0.db" in the project directory.
It has 25 closed loops.

## How to launch
```roslaunch my_robot world.launch```
```roslaunch my_robot localization.launch```
```rosrun teleop_twist_keyboard teleop_twist_keyboard.py```

## Necessary changes
- I needed to change the <frame_id> value in mapping.launch to "robot_footprint", as hinted in this [answer](https://knowledge.udacity.com/questions/36912)

## Other notes
- Worked on my local machine (VMWare)
- I didn't have to modify the topic names as requested in the [text](https://classroom.udacity.com/nanodegrees/nd209/parts/3882e4dc-c5d8-4f7e-9889-82c8e97fef6a/modules/d0a7a100-33ba-4778-a648-c3d7eb502c1b/lessons/c6b200bf-a0a5-456c-a988-d2d4cda5e003/concepts/af33921b-6ca3-46ac-8184-b9103ad973f9), which was extremely confusing. 
