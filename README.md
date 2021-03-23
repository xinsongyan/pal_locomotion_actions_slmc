
# PAL Locomotion Actions SLMC

* Launch gazebo simulation: 
```
$ roslaunch talos_gazebo talos_gazebo.launch
``` 

* Launch whole-body controller:
```
$ roslaunch talos_pal_locomotion talos_dcm_walking_controller.launch estimator:=kinematic_estimator_params
```

* Generated trajectory and push results to ROS param server:
```
$ python3 ~/catkin_ws/src/pal_locomotion_actions_slmc/script/walking_motion_generation_LIPM.py
```
or
```
$ python3 ~/catkin_ws/src/pal_locomotion_actions_slmc/script/walking_motion_generation_CoM.py
```

note: to use python3 with ROS, you need to do install:
```
$ sudo apt-get install python3-pip python3-yaml
$ sudo pip3 install rospkg catkin_pkg
```

* Run walking action, the action executes the motion in a open loop fashion:
```
$ rosrun pal_locomotion_actions_slmc push_walking_action_slmc
```


