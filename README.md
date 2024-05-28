# esp32_diffdrive_robot
ESP32-based diffdrive robot.

Microros node implemented on ESP32 subscribes `wheel_command` topic and publishes `wheel_state`.
There are command and state velocities in rad/s.

The `microros_hardware_interfaces` subscribes to `wheel_state` and publishes `wheel_command`. 
It also exports velocity interfaces for joints described in `<ros2_control>` tag in
[robot description](microros_hardware_interfaces/urdf/ros2_control.urdf.xacro).

# Flash firmware
Open VS Code in `firmware` folder wait for PlatformIO initialization and `Upload` program.


# Build ROS 2 workspace
To build workspace install [ROS humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation.html), then:

```bash
mkdir esp32bot_ws
cd esp32bot_ws

git clone https://github.com/wust-dcr/esp32_diffdrive_robot.git src

vcs import src < src/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

# Running
This example uses `ROS_DOMAIN_ID=0` remember:

```bash
source install/setup.bash

export ROS_DOMAIN_ID=0
ros2 launch esp32_diffdrive_robot_bringup microros.launch.py  serial_port:=/dev/ttyUSB0 serial_baudrate:=115200
```

The `diff_drive_controller` should activate correctly. Now you can drive the robot using `teleop_twist_keyboard`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
