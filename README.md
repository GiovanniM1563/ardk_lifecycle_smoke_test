# ARDK - Appliance Robot Development Kit

A ROS 2 orchestration layer that manages mode transitions between SLAM mapping and Nav2 navigationm with support for more stacks on the way.

**Status: Work in Progress** - Core functionality is implemented and tested on a single platform. Expect rough edges.

## Why Use This

If you're building a robot with ROS 2 and want to:
- Switch between mapping and navigation without restarting nodes or writing shell scripts
- Control your robot's nav stack from a web app, mobile app, or external system
- Have a consistent API layer between your application code and ROS internals
- Avoid dealing with Nav2/SLAM lifecycle management directly

This handles those pieces so you can focus on your application logic.

## What Problems It Solves

**The lifecycle problem**: Nav2 and slam_toolbox use ROS 2 lifecycle nodes. Getting them to start, configure, activate, and deactivate in the right order—especially when switching between modes—requires careful sequencing. This handles that.

**The exclusivity problem**: Enforces mutual exclusivity and handles the teardown/startup sequence.

**The integration problem**: ROS 2 command-line tools work fine for development, but integrating with external systems (web dashboards, fleet managers, custom apps) requires an API layer. This provides one.

## What This Is

A state manager and REST API for switching a robot between Mapping, Navigation, and Idle modes (as of now). It handles the lifecycle management of `slam_toolbox` and `nav2` so you don't have to write custom scripts or actively manage state.

## Limitations

- Not hardened for production (no authentication, limited error recovery)
- Requires your hardware layer (odometry, LiDAR) to be configured on your own 


---

## Requirements

**ROS 2 Jazzy** with these packages installed:
- `nav2_bringup`
- `slam_toolbox`
- `nav2_msgs`
- `lifecycle_msgs`

**Hardware layer** must be running separately:
- Publishes `odom → base_link` TF (e.g., from EKF or wheel odometry)
- LiDAR providing `/scan` topic

**Configuration files** (not included):
- `nav2_params.yaml` - Nav2 configuration
- `slam_params.yaml` - SLAM Toolbox configuration
- These must be passed as launch parameters

---

## Capabilities

### Modes
| Mode | Value | What It Does |
|------|-------|--------------|
| IDLE | 0 | Nothing running, motion stopped |
| MAPPING | 1 | slam_toolbox active, teleop allowed |
| NAVIGATION | 2 | Nav2 stack active, goal navigation enabled |

### REST API Endpoints

**Mode Control**
```
POST /ardk/mode         Switch modes
GET  /ardk/status       Current state, health, errors
POST /ardk/clear_fault  Clear latched fault
```

**Map Catalog**
```
POST /maps/save         Save current map with name
GET  /maps              List all saved maps
POST /maps/load         Load map by name (switches to NAV)
```

**Navigation**
```
POST /nav/goal          Send x,y goal (requires NAV mode)
POST /nav/cancel        Cancel active goal
GET  /nav/state         idle/active/succeeded/failed/canceled
POST /nav/map           Hot-swap map while in NAV
```

### Stable Topics
These topics always exist and switch sources based on mode:
- `/ardk/map` - Current map (SLAM or static)
- `/ardk/pose` - Current pose (slam_toolbox or AMCL)
- `/ardk_status` - Mode, health, errors

### Health Monitoring
- TF chain freshness check (odom→base_link)
- Required service presence check
- nav_stack_state: inactive/active/faulted

---

## Usage

### 1. Start Your Hardware Layer First
```bash
# Whatever launches your robot's odometry, LiDAR, etc.
bash ~/your_robot/hardware_start.sh
```

### 2. Launch State Manager
```bash
ros2 launch ardk_lifecycle system.launch.py \
  nav_config_path:=/path/to/nav2_params.yaml \
  slam_config_path:=/path/to/slam_params.yaml \
  default_map_path:=/path/to/default_map
```

### 3. Start API Server
```bash
ros2 run ardk_api server
# Runs on http://0.0.0.0:8000
```

### 4. Use It
```bash
# Switch to mapping
curl -X POST http://localhost:8000/ardk/mode -H "Content-Type: application/json" \
  -d '{"mode": 1}'

# Save map
curl -X POST http://localhost:8000/maps/save -H "Content-Type: application/json" \
  -d '{"name": "kitchen"}'

# Load map and navigate
curl -X POST http://localhost:8000/maps/load -H "Content-Type: application/json" \
  -d '{"name": "kitchen"}'

# Send goal
curl -X POST http://localhost:8000/nav/goal -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 0.5}'
```

---

## Sample Frontend

A web-based control panel is included in `frontend/index.html`.

**Features:**
- Live map visualization from `/ardk/map`
- Robot pose overlay (green triangle showing position and heading)
- Mode control buttons with transition locking
- Status panel with health indicators
- Map catalog (save, list, load)
- Navigation goal input (click on map or enter coordinates)

**To run:**
```bash
# 1. Start rosbridge (required for map/pose streaming)
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090

# 2. Serve frontend
python3 -m http.server 8080 -d frontend

# 3. Open in browser
http://<robot-ip>:8080
```

The frontend auto-detects the robot host from the URL.

## Testing

### Integration Test
Runs mode transitions, map save/load, status checks:
```bash
bash src/ardk_api/test_integration.sh
```

### Lifecycle Cycle Test
Stress tests mode cycling:
```bash
python3 src/ardk_lifecycle/tests/cycle_test.py
```

---

## Known Issues / Limitations

1. **Hardware must be running first** - Nav2 will fail to activate if `odom→base_link` TF isn't being published

2. **DDS service caching** - We rely on process termination rather than service disappearance for exclusivity checks

3. **Lifecycle manager node list** - `nav2_params.yaml` must list all 10 nodes that Nav2 Jazzy's `bringup_launch.py` actually launches, or the lifecycle manager will hang

4. **No security** - The API has no authentication; add your own if exposing to a network

5. **Tested only on RPi 5** - Other hardware may need tuning (timeouts, bond_timeout, etc.)

---

## File Structure

```
src/ardk_lifecycle/
├── scripts/state_manager.py    # Main orchestrator
├── ardk/
│   ├── runners/                # SLAM and Nav2 process control
│   └── core/                   # Readiness checks, health monitor
├── msg/ARDKStatus.msg
├── srv/SetMode.srv, ClearFault.srv
└── launch/system.launch.py

src/ardk_api/
├── ardk_api/
│   ├── main.py                 # FastAPI endpoints
│   ├── ros_bridge.py           # ROS 2 clients
│   └── map_catalog.py          # Named map storage
└── test_integration.sh
```

---

## Future Work

1. **Multiple stacks/Modularity** - Support for more stacks than just Nav2 and SLAM Toolbox or "mix and match" of supported modules. 
2. **Simulator Integration** - Support for ROS 2 simulators like Gazebo or Ignition.
3. **Control MUX** - Support for multiple control inputs and authorities.
4. **Security** - Add authentication and authorization to the API.
5. **Testing** - Add more tests and test coverage.
6. **Reliability** - Add more reliability and fault tolerance/handling. 
7. **Frontend Example** - Add a frontend example.


