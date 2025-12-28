# leptrino_force_sensor

A ROS 2 driver for the **Leptrino 6-axis force/torque sensor**.  
This package communicates with the sensor via serial connection and publishes
force and torque data as `geometry_msgs/msg/WrenchStamped`.

It supports startup calibration, offset save/load, and **service-based
recalibration without restarting the node**.

[![IMAGE](http://img.youtube.com/vi/Nr7wmg-SAko/0.jpg)](https://youtu.be/Nr7wmg-SAko)

## Features

- Serial communication with Leptrino force/torque sensors
- Publishes `geometry_msgs/msg/WrenchStamped`
- Automatic zero-offset calibration at startup (mean-based)
- Save and reload offset values from a file
- ROS 2 services for:
  - Saving offset values
  - Triggering re-calibration at runtime

## Environment

- ROS 2 Humble
- Ubuntu 22.04
- Leptrino 6-axis force/torque sensor **FFS055YA501U6**

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/open-rdc/leptrino_force_sensor
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Execute

### Basic execution

```
ros2 run leptrino_force_sensor leptrino_force_sensor_node
```
### Execute with offset file specified

```
ros2 run leptrino_force_sensor leptrino_force_sensor_node --ros-args -p offset_file:=/home/ubuntu/leptrino_offset.txt
```

## Published Topic

- /force_data
Type: geometry_msgs/msg/WrenchStamped

Contents:
```
force.x, force.y, force.z [N]
torque.x, torque.y, torque.z [Nm]
```
Values are published after offset compensation  

## Services

### Save Offset

Saves the current offset values to the file specified by offset_file.
```
ros2 service call /save_offset std_srvs/srv/Trigger
```

### Recalibrate

Resets the calibration state and starts re-calibration using incoming sensor data.
```
ros2 service call /recalibrate std_srvs/srv/Trigger
```
Node restart is not required.
