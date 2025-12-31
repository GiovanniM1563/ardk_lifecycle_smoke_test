# ARDK Lifecycle & Mode Management System

**"Appliance Robot Development Kit"**


The point of this project is to create a readymade, easy to use system that allows individuals to accelerate the arbitrary setup of ROS 2 robots, to allow them to focus on their own unique applications or additions. 

Ideally, this system should be able to solve the "boring" parts of ROS2 integration such as switching and orchestrating between different stacks on the fly, without the need to use ROS 2 CLI tools or create your own scripts to do so.


This repository contains the hardened lifecycle management system for ROS2 robots. It solves a problem I faced, switching between `MAPPING` (SLAM_Toolbox) and `NAVIGATION` (Nav2) actively and frequently.

---

##  Key Features

### 1. Resident Nav2 Architecture
- **Problem**: Launching Nav2 from scratch takes 30-45 seconds on RPi and consumes massive CPU/DDS resources.
- **Solution**: The `nav2_bringup` process remains resident in memory. We use `LifecycleServiceClients` to purely toggle nodes between `ACTIVE` and `INACTIVE` states.
- **Result**: Transition time reduced to **< 5 seconds**.

### 2. Strict "No Partial Startups"
- **Atomic Rollback**: If ANY step of a transition fails (e.g., Map Load timeout, Service unavailability), the system **automatically** rolls back to `IDLE`.
- **Zero Zombie Processes**: signal handling ensures all subprocesses (SLAM, Nav2 container) are cleanly killed on exit.

### 3. Smooth & Safe Transitions
- **Event-Driven Gates**: active checks of real conditions before transition:
    - `wait_for_topic('/map')`: Ensures SLAM is actually publishing.
    - `wait_for_tf_chain(...)`: Ensures the full `map -> odom -> base_link` tree is valid before handing control to Nav2.
- **Motion Guard**: Every mode transition automatically forces `cmd_vel` to zero to prevent runaway motion.

### 4. Robustness for Edge Hardware
- **Retry Logic**: Critical services (like `load_map`) utilize a 3-try backoff mechanism to handle disk/DDS latency on SD cards.
- **State Tracking**: Intelligent tracking prevents redundant `shutdown` calls, avoiding "hanging" issues common with ROS 2 Lifecycle Managers on slow DDS networks.

---

## System Architecture

### Core Components

1.  **`state_manager` (Node)**: The central orchestrator.
    -   Manages the high-level Finite State Machine (`IDLE`, `MAPPING`, `NAVIGATION`).
    -   Holds persistent connections to Lifecycle Managers (`/lifecycle_manager_navigation/manage_nodes`).
    -   Publishes `/ardk_status` for observability.

2.  **`ardk` (Python Package)**:
    -   **`runners.nav_runner`**: Handles interaction with the Resident Nav2 stack (Composition enabled).
    -   **`runners.slam_runner`**: Manages SLAM Toolbox lifecycle.
    -   **`core.readiness`**: Contains the logic for "Readiness Gates" (Topic/TF checks).

3.  **Communication**:
    -   **Service**: `/set_mode` (`ardk_lifecycle/srv/SetMode`)
    -   **Topic**: `/ardk_status` (`ardk_lifecycle/msg/ARDKStatus`)

### The 10 Principles of Hardening
This architecture strictly adheres to:
1.  **Lifecycle-based Transitions** (Not Process-based)
2.  **Single-Authority Invariants** (SLAM xor AMCL)
3.  **Correct Ordering** (Loc -> Map -> Nav)
4.  **Readiness Gates** (No Sleeps)
5.  **Warm Service Clients** (Reuse connections)
6.  **Operational Purity** (No CLI tools)
7.  **Motion Safety** (Stop before switch)
8.  **Reduced Participant Churn** (Composition)
9.  **Parameterized Configuration**
10. **Measurable "Done" Criteria** (ARDKStatus)

---

## Usage Guide

### 1. Launch the System
```bash
ros2 launch ardk_lifecycle system.launch.py
```

### 2. Switch Modes (Service Call)
**To Mapping (SLAM):**
```bash
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 1}"
```
*Starts SLAM Toolbox. Robot is ready to map.*

**To Navigation (AMCL + Nav2):**
```bash
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 2, map_yaml_path: '/path/to/my_map.yaml'}"
```
*Saves current map (if mapping), stops SLAM, loads map, activates AMCL/Nav2.*

**To Idle:**
```bash
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 0}"
```
*Cleanly deactivates all stacks.*

### 3. Monitor Status
```bash
ros2 topic echo /ardk_status
```
Example Output:
```yaml
mode: 2 (NAVIGATION)
map_source: "map_server"
tf_authority: "amcl"
motion_authority: "nav2"
```
### 4. Finer Usage

Nav2 and SLAM Toolbox are both controlled through their own APIs, and can be used independently of the state manager to get desired functionalities. You can for example read the /ardk_status to check the mode to ensure the right stack is operational, and then use its corresponding APIs (I.E. map saving, Pose Routing etc.) 


## Future WIP

The system is currently in a proof of concept stage and is not yet ready for production use. Future work includes:

- Building a ROS 2 package for the state manager
- Adding support for more operation modes and modular configurations to add your own modes
- Adding rosbridge support to allow remote triggers of mode changes
- adding helpers for Nav2 and SLAM Toolbox native APIs to allow for more fine-grained control





## Verification

The system includes a verification script (`tests/cycle_test.py`) that performs a full operational cycle:

```bash
# Run the automated verification
python3 src/ardk_lifecycle/tests/cycle_test.py
```

**Success Criteria:**
- [x] **IDLE -> MAPPING**: < 1.0s. Map topic available.
- [x] **MAPPING -> NAVIGATION**: Map saved, SLAM stopped, Nav2 Activated, TF Tree valid.
- [x] **NAVIGATION -> IDLE**: Clean shutdown, no lingering processes.

*Current Status: **Tested on RPi 5/Ubuntu 24.04 Server/Jazzy/CycloneDDS.***
