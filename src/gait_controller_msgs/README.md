# README — `gait_controller_msgs`

This package defines the custom ROS 2 interfaces (Actions & Services) used by the gait controller.

----------

## Package Overview

`gait_controller_msgs` contains:

### **Actions**

#### `MoveCylinder.action`

Used to command a pneumatic cylinder (CU, CP, CL) to move to a target position or perform a motion such as:

-   `in`
-   `out`
-   `stop`
-   `door_open`
-   `door_close`


`# Goal  string  command  # e.g. "cu in"  ---  # Result  string  result  # human-readable result string  ---  # Feedback  int32  current_position`

### **Services (Optional)**

If present in your package:

-   `SendCommand.srv`
    _(Legacy string-based interface, deprecated in favor of the action.)_


----------

## Dependencies

Your `package.xml` correctly lists:

-   `rosidl_default_generators`

-   `rosidl_default_runtime`

-   `std_msgs`


And the `CMakeLists.txt` uses:

`rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveCylinder.action"
  DEPENDENCIES std_msgs
)`

----------

## Build
Inside your workspace:

`colcon build --packages-select gait_controller_msgs source install/setup.bash`