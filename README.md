# openarmx_head_joint_slider_panel

English | [中文](./README-CN.md)

---

![Cover](./image/cover.gif)


RViz2 panel plugin providing slider-based control for the OpenArmX head joints (yaw and pitch).

## Overview

This package implements an RViz2 panel (`rviz_common::Panel` plugin) that allows direct control of the OpenArmX head's two joints via sliders. It publishes position commands to the `head_forward_position_controller`, supports stepped motion execution, live TF preview, record/playback of joint positions, and automatic synchronization with current robot state.

## Features

- **Slider control**: Two sliders (Yaw and Pitch) with real-time degree readout
- **Stepped motion execution**: Commands are sent incrementally at a configurable step size (1-200 mrad per cycle at 50 Hz) to ensure smooth motion
- **Automatic sync**: On first `/joint_states` message, sliders automatically sync to current robot position
- **Manual sync**: "Sync from /joint_states" button to re-sync at any time
- **Home button**: Smoothly move all joints to zero position with confirmation dialog
- **Record points**: Save current joint positions as named waypoints; move to any recorded point
- **Live TF preview**: Publishes preview transforms for visualization in RViz before executing motion
- **URDF-aware joint limits**: Automatically reads `robot_description` to set slider ranges; falls back to +/-90 degrees for Yaw and -62.2 to +23.1 degrees for Pitch
- **Persistent records**: Record points are saved/loaded from RViz config and a fallback settings file

## Plugin Info

- **Plugin name**: `openarmx_head_joint_slider_panel/HeadJointSliderPanel`
- **Base class**: `rviz_common::Panel`
- **Plugin type**: RViz2 panel

## Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_head_joint_slider_panel
source install/setup.bash
```

## Usage

1. Launch the head bringup:
   ```bash
   ros2 launch openarmx_head_bringup head.launch.py
   ```

2. In RViz2, add the panel:
   - Panels -> Add New Panel -> openarmx_head_joint_slider_panel / HeadJointSliderPanel

3. The panel will:
   - Automatically detect joint limits from the URDF
   - Sync slider positions from `/joint_states`
   - Allow direct slider manipulation to command the head

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions for sync |
| `/robot_description` | `std_msgs/String` | URDF for joint limit extraction (transient local QoS) |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/head_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Position commands [yaw, pitch] in radians |

## TF Published

Preview transforms are published under prefixed frame names (`preview_<link_name>` and `preview_/<link_name>`) for visualizing the target pose in RViz without moving the robot model.

## Configuration

The panel saves/restores the following via RViz config:
- `joint_step_mrad`: Step size in milliradians (1-200)
- `record_targets`: Serialized record points

Fallback settings file: `~/.config/OpenArmX/HeadJointSliderPanel.conf`

## Dependencies

- `rclcpp`, `rviz_common`, `pluginlib`
- `sensor_msgs`, `std_msgs`, `geometry_msgs`
- `tf2_ros`, `urdf`, `kdl_parser`
- Qt5 (Core, Widgets)

## Prerequisites

- `openarmx_head_bringup` must be running (provides controller_manager, joint_state_broadcaster, and forward_position_controller).
- RViz2 must be running to host the panel.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenArmX robotic platform ecosystem, developed for research and industrial applications in collaborative robotics.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 🌐 Documentation  | [http://docs.openarmx.com/](http://docs.openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
