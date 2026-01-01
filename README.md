# ARDK Lifecycle & Mode Management System

**"Appliance Robot Development Kit"**

This project implements a lightweight deterministic orchestration layer designed to simplify the complex task of managing robot states in ROS 2. It provides a robust, fail-safe mechanism for switching between Mapping (SLAM) and Navigation modes without the fragility often associated with custom scripts or manual CLI commands.

Future work will involve creating a command mux for orchestrated movement control.

This project aims to treat the robot's navigation stack as a reliable "black box" appliance, focusing on their unique application logic rather than the low-level intricacies of ROS 2 lifecycle management.

---

##  Key Capabilities

### 1. Instant Mode Switching
- **Fast Transitions**: Switches between Mapping and Navigation modes in seconds.
- **Optimized Performance**: Drastically reduces CPU and memory overhead compared to standard launch-and-kill approaches (AKA using the CLI!!!).

### 2. High Reliability & Fault Tolerance
- **Atomic Operations**: The system ensures the robot is never left in an undefined or broken state. If a transition cannot complete successfully, it automatically reverts to a safe idle state.
- **Clean Shutdowns**: Guarantees no lingering processes or "zombie" nodes, ensuring long-term system stability on edge hardware.

### 3. Safety-First Design
- **Readiness**: Transitions only occur when the system has verified that all necessary data (Sensor Transforms, Map Topics, etc.) is valid and available.
- **Optimized**: Tuned specifically for the constraints of edge compute devices like the Raspberry Pi, handling variable boot times and resource constraints gracefully.

Disclaimer: These systems do not claim to be 100 percent reliable or Safe, caution should still be exercised! Do your own Due Diligence!  

### 4. Fast API Service
- **External Control**: Send commands to your navigation stack via a web API, allowing external control via a custom web app!
- **SLAM and NAV Support**: Basic control over your navigation stack, such as map saving, map selecting, parameters, and more.

*Disclaimer: The API is open—you need to design your own security solution for a production machine!*

---

## Architecture

### On-Demand Nav2 Pattern
The state manager uses an **on-demand launch strategy** for Nav2 to avoid lifecycle state issues:
- Nav2 stack is **only launched** when transitioning to NAVIGATION mode
- This prevents nodes from entering the unrecoverable `finalized` state
- SLAM and Nav2 stacks are mutually exclusive—never running simultaneously

### SLAM Lifecycle Activation
When entering MAPPING mode, the system explicitly activates the `slam_toolbox` lifecycle node:
1. Launch SLAM process
2. Wait for node to appear in the ROS graph
3. Call `configure` → `activate` lifecycle transitions
4. Verify map topic availability before completing transition

---

## Usage Guide

### 1. Launch the System
```bash
ros2 launch ardk_lifecycle system.launch.py
```

### 2. Start the API Server
```bash
ros2 run ardk_api server
```
*Runs on `http://0.0.0.0:8000`*

### 3. API Control (Recommended)

**Switch to Mapping:**
```bash
curl -X POST "http://localhost:8000/ardk/mode" -d '{"mode": 1}'
```

**Save Map:**
```bash
curl -X POST "http://localhost:8000/slam/save" -d '{"name": "/home/gio/ARDK_2/maps/my_map"}'
```

**Switch to Navigation:**
```bash
# Switches mode to NAV and loads the specified map
curl -X POST "http://localhost:8000/ardk/mode" -d '{"mode": 2, "map_path": "/home/gio/ARDK_2/maps/my_map.yaml"}'
```

**Hot-Swap Map (While in Nav):**
```bash
curl -X POST "http://localhost:8000/nav/map" -d '{"path": "/home/gio/ARDK_2/maps/another_map.yaml"}'
```

---

## Verification

The system includes verification tools to ensure stability on your hardware:

1.  **Lifecycle Cycle Test**:
    ```bash
    python3 src/ardk_lifecycle/tests/cycle_test.py
    ```

2.  **Full API Integration Test**:
    ```bash
    bash src/ardk_api/test_integration.sh
    ```

**Verified Platforms:**
- Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy
