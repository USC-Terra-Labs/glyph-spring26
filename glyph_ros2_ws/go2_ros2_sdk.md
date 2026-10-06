# Unitree Go2 Jailbroken SDK Setup Guide

## SDK Information

### Repository
- **Source**: https://github.com/abizovnuralem/go2_ros2_sdk
- **Installation Location**: `~/ros2_ws/src/`
- **Connection Type**: CycloneDDS via Ethernet
- **Robot IP**: `192.168.123.161`
- **Host IP**: `192.168.123.162/24` (interface: `enP8p1s0`)

### Credentials
- **Robot Password**: `theroboverse`
- **Sudo Password**: `UniTree21!`

---

## Network Configuration

### CycloneDDS Configuration
- **Config File**: `/home/ming/cyclonedds_jetson.xml`
- **Network Interface**: `enP8p1s0`
- **Settings**: Shared memory disabled, topic discovery enabled

### Connection Verification
```bash
# Check robot connectivity
ping -c 3 192.168.123.161

# Check network interface
ip addr show enP8p1s0

# Verify DDS environment
env | grep -E '(ROS|DDS|CYCLONE)'
```

---

## Installed Packages

The workspace includes 5 packages:
1. **go2_interfaces** - Custom message/service definitions for Go2
2. **go2_robot_sdk** - Main robot driver and control nodes
3. **lidar_processor** - LiDAR data processing (Python)
4. **coco_detector** - Object detection using COCO dataset
5. **speech_processor** - Text-to-speech functionality

---

## Quick Start

### Basic Launch (Full Stack)
```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Set environment variables
export ROBOT_IP="192.168.123.161"
export CONN_TYPE="cyclonedds"

# Launch the robot SDK
ros2 launch go2_robot_sdk robot.launch.py
```

### What Gets Started:
- 🤖 **go2_driver_node** - Main robot driver
- 📹 **Front camera stream** - Color video feed
- 🗺️ **LiDAR point cloud** - 3D environment scanning
- 🎮 **Joystick teleop** - Xbox/PS controller support
- 🦊 **Foxglove bridge** - Web visualization (port 8765)
- 🗺️ **SLAM toolbox** - Real-time mapping
- 🧭 **Nav2 stack** - Autonomous navigation
- 📊 **RViz2** - 3D visualization

---

## Usage Examples

### View Available Topics
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

### Monitor Robot State
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /joint_states
ros2 topic echo /imu
ros2 topic hz /go2_camera/color/image
```

### Control Robot via Command Line
```bash
source ~/ros2_ws/install/setup.bash

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### View LiDAR Point Cloud
```bash
# Already visualized in RViz when robot.launch.py is running
# Point cloud topic: /cloud
# Laser scan topic: /scan
```

---

## Foxglove Studio Integration

### Connection
1. Open Foxglove Studio (web or desktop)
2. Connect to: `ws://192.168.123.162:8765`
3. Or access web version: https://app.foxglove.dev

### Features Available:
- Real-time 3D visualization
- Camera feeds
- Topic monitoring
- Custom layouts
- Data recording

---

## SLAM and Navigation

### Creating a Map
1. Launch the robot SDK (see Quick Start)
2. In RViz, use the **SlamToolboxPlugin** panel
3. Click "Start At Dock" to set initial pose
4. Drive the robot around using joystick to explore
5. Save the map:
   - Enter filename in "Save Map" field → Click "Save Map"
   - Enter filename in "Serialize Map" → Click "Serialize Map"
6. Files created: `map_name.yaml`, `map_name.pgm`, `map_name.data`, `map_name.posegraph`

### Autonomous Navigation
1. Launch the robot SDK
2. In RViz SlamToolboxPlugin:
   - Enter map filename (without extension) in "Deserialize Map"
   - Click "Deserialize Map"
3. Ensure robot is correctly positioned on the map
4. Use "Nav2 Goal" tool in RViz to set navigation targets
5. The robot will autonomously navigate to the goal

**⚠️ Warning**: Always monitor the robot during autonomous navigation. Incorrect maps or positioning can cause collisions.

---

## Rebuilding the Workspace

### Full Clean Rebuild
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Update SDK from GitHub
```bash
cd ~/ros2_ws/src
git pull origin master
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --symlink-install
```

---

## Troubleshooting

### Robot Not Connecting
```bash
# Check network connectivity
ping 192.168.123.161

# Verify CycloneDDS configuration
cat /home/ming/cyclonedds_jetson.xml

# Check if DDS topics are visible
ros2 topic list | grep -E '(lowstate|sport)'
```

### No Topics Visible
```bash
# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Check RMW implementation
echo $RMW_IMPLEMENTATION  # Should be: rmw_cyclonedds_cpp

# Verify DDS discovery port
ss -tuln | grep 7400
```

### Build Errors
```bash
# Install missing dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Camera/LiDAR Not Working
```bash
# Check if nodes are running
ros2 node list

# Monitor driver logs
ros2 run rqt_console rqt_console

# Verify data rate
ros2 topic hz /go2_camera/color/image
ros2 topic hz /cloud
```

---

## Important Notes

### Connection Type: CycloneDDS vs WebRTC
- **CycloneDDS** (Ethernet): Used in this setup
  - Direct wired connection
  - Low latency
  - High bandwidth
  - More reliable

- **WebRTC** (WiFi): Alternative option
  - Wireless connection
  - Must close mobile app before connecting
  - Set `CONN_TYPE="webrtc"` to use

### Robot Safety
- Always have emergency stop ready (pick up the robot)
- Start with low speeds during testing
- Monitor battery levels
- Test navigation in open spaces first
- Keep robot software updated

### Performance Optimization
- LiDAR now runs at ~7 Hz (improved from 2 Hz)
- Joint states update at ~1 Hz (firmware limitation)
- Controller frequency: 3.0 Hz (conservative for stability)
- Planner frequency: 1.0 Hz (prevents overload)

---

## Additional Resources

### Documentation
- SDK Repository: https://github.com/abizovnuralem/go2_ros2_sdk
- ROS2 Humble Docs: https://docs.ros.org/en/humble/
- Nav2 Documentation: https://navigation.ros.org/
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

### Community Support
- Report issues: https://github.com/abizovnuralem/go2_ros2_sdk/issues
- Unitree Robotics: https://www.unitree.com/

---

## Quick Reference Commands

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Set robot connection
export ROBOT_IP="192.168.123.161"
export CONN_TYPE="cyclonedds"

# Launch full stack
ros2 launch go2_robot_sdk robot.launch.py

# View topics
ros2 topic list

# Echo robot state
ros2 topic echo /joint_states

# Check topic rates
ros2 topic hz /imu

# List available interfaces
ros2 interface list | grep go2

# Check active nodes
ros2 node list

# Kill all ROS processes
pkill -f ros
```

---

**Last Updated**: 2025-12-11
**SDK Version**: Latest from abizovnuralem/go2_ros2_sdk
**ROS2 Distro**: Humble
**Tested On**: Ubuntu 22.04 (Jetson Platform)
