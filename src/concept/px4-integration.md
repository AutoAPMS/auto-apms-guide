---
order: 50
title: PX4 Integration
---
# PX4 Autopilot Integration

For controlling autonomous sytems running [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) we recommend incorporating the [PX4/ROS2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html). We provide ready-to-use plugins for AutoAPMS in the [`auto-apms-px4`](https://github.com/AutoAPMS/auto-apms-px4) repository. This repo introduces two abstractions which allows the user to define custom PX4 modes that are dynamically registered at runtime and communicate with the autopilot through [uORB interfaces](https://docs.px4.io/main/en/middleware/uorb.html):

- `ModeExecutor`
- `ModeExecutorFactory`
