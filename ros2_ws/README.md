# ROS 2 workspace

A ROS 2 Jazzy colcon workspace for Indy7 + Mark7 + RealSense RGB collection and
real-robot ACT evaluation.

## Layout

| Directory | Role |
| --- | --- |
| `src/` | The ROS package sources. Package-by-package detail is in [`src/README.md`](src/README.md) |
| `build/` | colcon intermediate build output. Not in git |
| `install/` | colcon setup files and installed package tree. Not in git |
| `log/` | colcon build and test logs. Not in git |

## Building

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Never edit anything under `build/`, `install/` or `log/` - they are regenerated
on every build.

## Before running anything that moves the robot

Clear the workspace, keep an emergency stop within reach, and stay at the
keyboard. Do not start two copies of the same driver, camera graph or ZMQ
endpoint; they will fight over the same hardware.

The hardware setup and evaluation procedure of the finished experiment are in
[`../docs/experiment_design.md`](../docs/experiment_design.md).
