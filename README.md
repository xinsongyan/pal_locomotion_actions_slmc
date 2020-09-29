# PAL Locomotion Actions

This package contains the main actions that are already implemented for the Biped DCM controller state machine.

The available actions are:
- *Balance DS action*: Action to maintain the Center of Mass (CoM) in the center of the polygon of support when the robot is in double support. It is the default state when starting the Biped DCM controller.
- *StaticWalkAction*: Action to walk, always maintaining the CoM inside the support polygon.
- *WBCAction*: Wrapper of the Kinematic WBC used for manipulation that allows to push and pop dynamic tasks dynamically.
- *StandOneLegAction*: Action to lift and balance in one leg.
- *StandDownLegAction*: Action to pass from single support balancing to double support balancing.
- *MoveICPDSAction*: Action to move the CoM from the left foot to the right foot in double support.

## Implementation

All the implemented actions are plugins derived from the `WalkingActionBase`, which is in turn derived from the `pal_robot_tools::Action`.

It has the following methods:
- `configure`: Non-RT method used to configure the action and load it in the state machine. It has a property_bag (serialized dictionary) used to get params online by key.
- `enterHook`: RT method that is executed the first time it enters the loop.
- `cycleHook`: RT method that is called at every update cycle.
- `isOverHook`: Method that returns true if `cycleHook` is over or in a safe state in which the action could be preempted by a new one.
- `endHook`: RT method that is called when the action is removed from the queue replaced by another one.

Every time an action is pushed, it tries to preempt the actual one by calling `isOverHook` periodically. Once this method returns true, it calls `endHook` and loads the new action.

At the end of the `cycleHook`, all the actions call the `control` method from [icp_control_utils](./src/icp_control_utils.cpp), that sends the desired references to the biped controller. This method requires the desired DCM position and velocity, and the reference Center of Pressure (CoP). The implemented control law is based on this [paper](https://ieeexplore.ieee.org/document/6094435).

### StaticWalkAction

The `StaticWalkAction` is implemented as a state machine.

* STAGE_DOUBLE_SUPPORT_COM_HOLD_REFPOINT: Stays in Double Support (DS) with the DCM at the center of the support polygon, waiting to receive a command.
* STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY: Initializes the CoM trajectory to the stance foot.
* STAGE_DOUBLE_SUPPORT_MOVE_COM: Executes the CoM trajectory.
* STAGE_COM_HOLD_REFPOINT: Sends a reference CoM point until the error is below the given threshold.
* STAGE_SINGLE_SUPPORT_SWING_INITIALIZE: Initializes the swing motion from a velocity command (which is integrated as a pose) or a desired pose.
* STAGE_SINGLE_SUPPORT_SWING_INITIALIZE: Executes a swing motion until the FT sensor detects a contact. If no contact is detected, it generates a new command (lowered foot) and goes back to STAGE_SINGLE_SUPPORT_SWING_INITIALIZE.
* STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY: Initializes the CoM trajectory to the center of the support polygon.
* STAGE_DOUBLE_SUPPORT_MOVE_COM: Executes the CoM trajectory.
* STAGE_COM_HOLD_REFPOINT: Sends a reference CoM point until the error is below the given threshold
* STAGE_DOUBLE_SUPPORT_COM_HOLD_REFPOINT: Waits in DS to receive a new command.

### WBCAction

The `WBCAction` pushes a complete kinematic whole body controller into the state machine.

The same functionality of kinematic controller is achieved in the dynamic Whole Body Control (WBC) controller.
To start it, run:
```
$ rosrun pal_locomotion StartWBCAction
```
The procedure to push/pop/status actions for the Biped DCM controller state machine is the same as that for the whole body kinematic controller, but to make sure these refer to the biped DCM controller state machine the namespace `/biped_walking_dcm_controller/wbc` has to be specified.

For example, once the action is started, to control both end-effectors in Cartesian space, run the same launch file from [talos_wbc](https://gitlab/control/talos_wbc) but specifying the new cited namespace.
```
$ roslaunch talos_wbc interactive_markers.launch ns:=/biped_walking_dcm_controller/wbc
```
After this, open an RViz terminal to control both end-effectors using interactive markers.

> To avoid damaging the robot, make sure you do not send the robot close to a joint limit or far from the actual reference when controlling the robot using interactive markers or topic.

## Execution

This section is devoted to describe how to push the different tasks in the state machine (SM).

### Push a new task in the SM

When the biped DCM controller starts, a service to push new actions is prompted: `/biped_walking_dcm_controller/push_actions`.

This action receives a `pal_locomotion_msgs/ActionWithParameters[]` where the `action_type` and `action_parameters` need to be specified.

Example ([push stand down leg action](./src/nodes/push_stand_down_leg_action.cpp)):
```
pal_locomotion_msgs::PushActions push_actions_request;

pal_locomotion_msgs::ActionWithParameters new_action;
new_action.action_type = "pal_locomotion::StandDownLegAction";

property_bag::PropertyBag parameters;
parameters.addProperty("side", std::string(side._to_string()));
parameters.addProperty("ds_duration", ds_duration);
parameters.addProperty("swing_leg_duration", swing_leg_down_time);
parameters.addProperty("target_swing_leg_orientation", target_swing_leg_orientation);
parameters.addProperty("target_swing_leg_position", target_swing_leg_position);

std::stringstream ss;
boost::archive::text_oarchive oa(ss);
oa << parameters;
new_action.action_parameters = ss.str();
push_actions_request.request.actions.push_back(new_action)
```
Another option is to use the launch file from `pal_locomotion`:
 
For example:
```
 $ roslaunch pal_locomotion push_action.launch action:=pal_locomotion::StaticWalkAction
``` 

Or to use the nodes from this packages:
- `push_stand_one_leg_action`
- `push_stand_down_leg_action`
- `push_icp_ds_action`

> Take special attention when pushing the `push_stand_one_leg_action` or the `push_stand_down_leg_action`. The `push_stand_one_leg_action` should be never pushed again when the robot is balancing on one leg. It may crash the robot. The same happens when pushing down the leg.

For example:

To put the robot standing on the left leg
```
$ rosrun pal_locomotion_actions push_stand_one_leg_action -d 3.0 -u 3.0 -z 0.15 -s LEFT
```
And to lower the leg
```
$ rosrun pal_locomotion_actions push_stand_down_leg_action -d 3.0 -u 3.0
```
