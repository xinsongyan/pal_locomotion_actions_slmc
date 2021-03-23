
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
$ python ~/catkin_ws/src/pal_locomotion_actions_slmc/script/walking_motion_generation_LIPM.py
```

* Run walking action, the action executes the motion in a open loop fashion:
```
$ rosrun pal_locomotion_actions_slmc push_walking_action_slmc
```


