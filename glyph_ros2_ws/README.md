# `ros2_ws` Overview

## Purpose

`ros2_ws` is the ROS 2 workspace that ties together:

- the real Go2 robot bridge
- the RL-sar simulation bridge
- navigation, mapping, and localization
- shared robot description assets

The active workspace is under `~/ros2_ws/src/`.

## Active Packages

- `go2_unitree_bridge`
  - bridge layer for the real robot and RL-sar simulation
- `go2_navigation`
  - mapping, localization, Nav2 bringup, and helper nodes
- `go2_robot_sdk`
  - robot-side driver/config package used by the real robot launch path
- `go2_interfaces`
  - custom message definitions
- `lidar_processor`
  - Python LiDAR processing used by the default robot launch path
- `unitree_go2_description`
  - URDF/xacro, meshes, and world assets for visualization and TF

Optional packages still present in the workspace:

- `coco_detector`
  - object detection package, not part of the default navigation stack
- `speech_processor`
  - TTS/audio helpers, optional at runtime

## Source Layout

Top-level workspace directories:

- `~/ros2_ws/src/go2_unitree_bridge`
- `~/ros2_ws/src/go2_navigation`
- `~/ros2_ws/src/go2_robot_sdk`
- `~/ros2_ws/src/go2_interfaces`
- `~/ros2_ws/src/lidar_processor`
- `~/ros2_ws/src/unitree_go2_description`
- `~/ros2_ws/src/coco_detector`
- `~/ros2_ws/src/speech_processor`

Non-package workspace directories:

- `~/ros2_ws/src/docker`
- `~/ros2_ws/build`
- `~/ros2_ws/install`
- `~/ros2_ws/log`

## Supported Entry Points

- `go2_bridge`
  - starts the real robot bridge from `~/ros2_ws/src/go2_unitree_bridge/launch/go2_unitree_bridge.launch.py`
- `go2_nav_robot`
  - starts mapping/live SLAM for the real robot from `~/ros2_ws/src/go2_navigation/launch/go2_nav_robot.launch.py`
- `go2_nav_robot_localize map:=...`
  - starts localization/Nav2 for the real robot from `~/ros2_ws/src/go2_navigation/launch/go2_nav_robot_localization.launch.py`
- `go2_rlsar_sim`
  - starts the RL-sar bridge from `~/ros2_ws/src/go2_unitree_bridge/launch/go2_rlsar_sim.launch.py`
- `go2_nav_rlsar`
  - starts mapping/live SLAM for RL-sar from `~/ros2_ws/src/go2_navigation/launch/go2_nav_rlsar.launch.py`
- `go2_nav_rlsar_localize map:=...`
  - starts localization/Nav2 for RL-sar from `~/ros2_ws/src/go2_navigation/launch/go2_nav_rlsar_localization.launch.py`

## Dependency Notes

Real robot path:

- `go2_unitree_bridge`
- `go2_navigation`
- `go2_robot_sdk`
- `lidar_processor`
- `unitree_go2_description`

RL-sar path:

- `go2_unitree_bridge`
- `go2_navigation`
- `unitree_go2_description`

## External Sources

This workspace also depends on runtime pieces outside `~/ros2_ws`:

- ROS 2 Humble under `/opt/ros/humble`
  - base ROS environment sourced by the shell helpers
  - provides system packages used directly by the launches such as `nav2_bringup`, `slam_toolbox`, `pointcloud_to_laserscan`, `robot_localization`, and `foxglove_bridge`
  - also provides fallback robot description at `/opt/ros/humble/share/unitree_go2_description/urdf/unitree_go2_robot.xacro`
- Official Unitree ROS2 support: `~/unitree_ros2`
  - `~/unitree_ros2/setup_go2_humble.sh`
    - sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
    - sets `CYCLONEDDS_URI` for the robot Ethernet interface (`enP8p1s0`)
  - `~/unitree_ros2/example/install/setup.bash`
    - sourced by `robot_mode()` before real-robot launches
- Shell helpers in `~/.bashrc`
  - `robot_mode()`
    - sources `~/unitree_ros2/setup_go2_humble.sh`
    - sources `~/unitree_ros2/example/install/setup.bash`
    - re-sources `~/ros2_ws/install/setup.bash`
  - `go2_bridge`, `go2_nav_robot`, and `go2_nav_robot_localize`all rely on `robot_mode()`
  - `sim_mode()`, `go2_rlsar_sim`, `go2_nav_rlsar`, and `go2_nav_rlsar_localize` set the non-robot simulation environment outside the launch files themselves
- `~/go2_python_sdk` or `GO2_PYTHON_SDK_DIR`
  - used by `go2_robot_sdk/go2_robot_sdk/presentation/cyclonedds_bridge_node.py`
  - expected to provide the Python SDK virtualenv site-packages and IDL definitions
- `~/rl_sar` or `RLSAR_ROOT`
  - used by `go2_unitree_bridge/launch/go2_rlsar_sim.launch.py`
  - expected to contain `./cmake_build/bin/rl_sim_mujoco`
- Jetson host networking configuration
  - the real robot path assumes the dedicated robot link is on `enP8p1s0`
  - EEE stability fix:
    - `/usr/local/sbin/go2-eee-off`
    - `/etc/NetworkManager/dispatcher.d/99-go2-eee-off`
    - `/etc/systemd/system/go2-eee-off.service`
    - `/etc/udev/rules.d/99-go2-eee-off.rules`
    - the fix is not just a boot-time `ethtool` call:`go2-eee-off.service` calls the helper, which waits for `enP8p1s0` to exist and retries disabling EEE. `udev` rule re-triggers the helper when `enP8p1s0` is added or changes. NetworkManager dispatcher script is kept as a fallback path and also calls the same helper.
    - manual fallback if EEE ever comes on:
      - `sudo ethtool --set-eee enP8p1s0 eee off`
    - verification:
      - `ethtool --show-eee enP8p1s0`
      - `journalctl -t go2-net -b`

## Maps

Saved maps live in `~/ros2_ws/src/go2_navigation/maps`.
