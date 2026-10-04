# Gait_controller

The `gait_controller` package provides the high-level control node for three pneumatic cylinders: **CU**, **CP**, and **CL**.
It communicates with an Arduino via the `/arduino_command` topic, receives live cylinder positions from `/cu_pos`, `/cp_pos`, and `/cl_pos`, and exposes a **ROS 2 Action** interface for commanding cylinder movements.

This package works together with the `gait_controller_msgs` package, which defines the custom action type `MoveCylinder.action`.


## Features

###  ROS 2 Action Server (`/move_cylinder`)

Handles asynchronous cylinder movements:
-   supports **feedback** (live cylinder position)
-   supports **preemption** (new goal cancels old goal)
-   supports **cancel** from command line
-   supports blocking / non-blocking action clients
-   accepts simple string commands such as:
```
cu in
cu out
cl door_open
cp stop
```

### Arduino command publishing

The node sends commands to:

`/arduino_command (std_msgs/String)`

### Position subscriptions

The node receives cylinder position updates from:
```
 /cu_pos   (std_msgs/Int32)
 /cp_pos   (std_msgs/Int32)
 /cl_pos   (std_msgs/Int32)
```
These positions drive the action feedback and goal-completion detection.


## Move Commands

Each command has two parts:  `<axis> <state>`
### Supported axes:

-   `cu`
-   `cp`
-   `cl`

### Supported states:

|Axis  |States  |
|--|--|
|cu  | in, out, stop, door_open, door_close |
|cp  | in, out, stop |
|cl  | in, out, stop, door_open, door_close |


### Target positions

|  State| Meaning | Target Positions |
|--|--|--|
| in | retract | 100|
| out | extend | 400|
| stop | hold current position | |
| door_open | short retract | 10|
| door_close | close door position | 100|


## Running the Action Server

After building the workspace:
```
source install/setup.bash
ros2 run gait_controller gait_main_node
```

In addition, you can also run the cylinder position publisher node, if there is no real encoder connected:
```
ros2 run gait_controller cyl_pos_pub_node
```

## Using the Action From the Terminal

## 1. Check the action server

`ros2 action list
ros2 action info /move_cylinder`

## 2. Send an action goal

Example:

`ros2 action send_goal /move_cylinder gait_controller_msgs/action/MoveCylinder "{command: 'cu in'}"`

## 3. Send and show feedback live

`ros2 action send_goal /move_cylinder gait_controller_msgs/action/MoveCylinder "{command: 'cu out'}" --feedback`

You will see position feedback in the terminal.



## Cancel a running goal

To cancel all active goals:

`ros2 action cancel /move_cylinder`

If multiple goals are active:

`ros2 action list_goals /move_cylinder
ros2 action cancel /move_cylinder <goal_id>`

----------

## Preemption (Overwrite a Running Goal)

Simply send a new goal:

`ros2 action send_goal /move_cylinder gait_controller_msgs/action/MoveCylinder "{command: 'cu door_open'}"`

The previous goal automatically aborts and the new one executes immediately.


## Monitoring Activity

### View position topics:

`ros2 topic echo /cu_pos
ros2 topic echo /cp_pos
ros2 topic echo /cl_pos`

### View Arduino commands:

`ros2 topic echo /arduino_command`



## Node Overview

The node:

-   receives an action goal (`"cu in"`)
-   determines target position for that axis
-   publishes the command to `/arduino_command`
-   publishes feedback continuously
-   succeeds when current position reaches target
-   aborts on preempt or cancel


### Executor

The node uses a **MultiThreadedExecutor**, allowing:

-   subscriber callbacks for positions
-   action callback execution  to run concurrently.
