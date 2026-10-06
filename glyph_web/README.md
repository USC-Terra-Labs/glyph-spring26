# Glyph Web Dashboard

`glyph_web` is a FastAPI + ROS2 web interface for controlling Glyph, managing navigation queues, and operating a UART-driven drink dispenser system.

It acts as the bridge between:
- the browser-based dashboard
- ROS2 navigation + perception
- dispenser subsystem (via UART topics)

---

## Core Features

### Navigation & Mapping
- Live occupancy grid visualization (`/map`)
- Robot pose tracking with TF transforms
- Click-to-go navigation
- Queue visualization (ACTIVE, Q1, Q2, ...)
- Goal cancelation by clicking near queued points
- Automatic goal clearing on completion or clock reset

---

### Landmark System
- Click-to-create landmarks
- Drag to reposition landmarks
- Persistent storage (`landmarks.json`)
- Automatic ID compaction and renumbering
- Goal snapping to nearby landmarks

---

### Order & Queue System
- Create drink orders tied to landmarks
- Orders automatically follow navigation queue
- Order states:
  - `submitted`
  - `queued`
  - `executing`
  - `completed`
  - `canceled`
  - `failed`
- Orders are reconciled against live ROS queue markers

---

### Dispenser / UART Integration
- Flavor selection publishing (`/flavor_selection`)
- UART command + event handling
- Live liquid level parsing: LEVELS:W=... A=... B=... C=...
- Automatic UART trigger for flavor selection when order becomes ACTIVE
- Safety checks:
  - blocks movement if dispensing
  - blocks navigation, flavor selection, and manual gate-open actions if empty
  - treats both `EMPTY` and `FLAVOR_EMPTY` as dispenser-empty conditions
  - keeps the robot at home in idle behavior until refill is reported

---

### Movement Gate Control
- Controls robot motion permission via:
- `/currently_dispensing`
- `/return_home_trigger` (legacy)
- Automatically syncs gate with dispensing state:
  - `currently_dispensing=True` → gate closed
  - `currently_dispensing=False` → gate open
- `dispense_empty=True` overrides normal serving flow by preventing new orders from leaving home

---

### Real-Time Web UI
- WebSocket-based live updates
- Live telemetry
- Robot pose
- Goals
- Queue state
- Dispenser state
- Activity log

---

## Architecture Overview
Browser (index.html)
↕ WebSocket
FastAPI (app.py)
↕
ROS2 Node (rclpy)
↕
Nav2 + Behavior Supervisor + UART nodes


---

## ROS Topics

### Publishes
- `/target_location` (`geometry_msgs/msg/PoseStamped`)
- `/behavior_supervisor_cancel_goal` (`geometry_msgs/msg/PoseStamped`)
- `/flavor_selection` (`std_msgs/msg/UInt8`)
- `/dispense_uart_command` (`std_msgs/msg/String`)
- `/set_home_here` (`std_msgs/msg/Bool`)
- `/currently_dispensing` (`std_msgs/msg/Bool`)
- Movement gate topics (`std_msgs/msg/Bool`)

### Subscribes
- `/odom` (`nav_msgs/msg/Odometry`)
- `/map` (`nav_msgs/msg/OccupancyGrid`)
- `/tf`, `/tf_static` (`tf2_msgs/msg/TFMessage`)
- `/navigate_to_pose/_action/status` (`action_msgs/msg/GoalStatusArray`)
- `/behavior_supervisor_queue` (`visualization_msgs/msg/MarkerArray`)
- `/dispense_uart_event` (`std_msgs/msg/String`)
- `/currently_dispensing` (`std_msgs/msg/Bool`)
- `/dispense_empty` (`std_msgs/msg/Bool`)

`/dispense_empty=True` is raised for either `EMPTY` or `FLAVOR_EMPTY` from the dispenser bridge.
It is cleared when the dispenser reports refilled levels again.

---

## API Endpoints

### Landmarks
- `GET /api/landmarks`
- `POST /api/landmarks`
- `DELETE /api/landmarks/{id}`

### Orders
- `POST /api/orders`
- `GET /api/orders/{id}`

### Queue
- `GET /api/queue`

---

## WebSocket Messages

### Client → Server
- `xy` → send goal
- `cancel_xy` → cancel goal
- `flavor_selection`
- `currently_dispensing`
- `set_home`
- `dispense_query`

### Server → Client
- `state`
- `goal`
- `queue`
- `uart_event`
- `activity_log`

---

## Environment Variable Defaults

TARGET_TOPIC: `/target_location`
HOME_TARGET_TOPIC: `/return_home_target_location`
MOVEMENT_GATE_TOPIC: `/return_home_trigger`
MAP_TOPIC: `/map`
ODOM_TOPIC: `/odom`
PORT: `8000`

---

## Running

cd ~/glyph_web

\# Activate venv

source venv/bin/activate

\# Source ROS2

source /opt/ros/humble/setup.bash

source ~/ros2_ws/install/setup.bash # if needed

\# Run server

python3 app.py
