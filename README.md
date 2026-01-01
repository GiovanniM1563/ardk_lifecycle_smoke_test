# ARDK - Appliance Robot Development Kit

A ROS 2 orchestration layer that manages mode transitions between SLAM mapping and Nav2 navigation.

## What This Is

A state manager and REST API for switching a robot between Mapping, Navigation, and Idle modes. It handles the lifecycle management of `slam_toolbox` and `nav2` so you don't have to write custom launch scripts.

## What This Is Not

- Not a complete robot framework
- Not hardened for production use
- Not a security solution (the API is completely open)
- Not tested beyond Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy

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

2. **DDS service caching** - ROS 2 DDS caches service registrations for ~60s after process death, so we rely on process termination rather than service disappearance for exclusivity checks

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

## Verified On

- Raspberry Pi 5
- Ubuntu 24.04
- ROS 2 Jazzy
