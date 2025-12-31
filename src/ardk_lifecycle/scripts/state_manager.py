#!/usr/bin/env python3

import sys
import threading
import time
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

from ardk_lifecycle.srv import SetMode
from ardk_lifecycle.msg import ARDKStatus
from ardk.runners import slam_runner, nav_runner
from ardk.core.readiness import wait_for_services, wait_for_tf_chain, wait_for_topic
from nav2_msgs.srv import ManageLifecycleNodes, LoadMap
from slam_toolbox.srv import SaveMap

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        
        self.state = SetMode.Request.IDLE
        self.lock = threading.Lock()
        
        # Process Handles
        self.slam_proc: Optional[subprocess.Popen] = None
        self.nav_proc: Optional[subprocess.Popen] = None

        # Publishers / TF
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(ARDKStatus, 'ardk_status', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Persistent Service Clients (Principle 5)
        self.cli_loc = self.create_client(ManageLifecycleNodes, "/lifecycle_manager_localization/manage_nodes")
        self.cli_nav = self.create_client(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")
        self.cli_load_map = self.create_client(LoadMap, "/map_server/load_map")
        self.cli_save_map = self.create_client(SaveMap, "/slam_toolbox/save_map")

        # Service
        self.srv = self.create_service(
            SetMode, 
            'set_mode', 
            self.handle_set_mode,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("ARDK State Manager Ready. Current State: IDLE")
        
        # Resident Nav2: Start once, keep alive. (Principle 1)
        # Use Composition to save DDS resource/participants
        self._nav2_cmd = (
            "ros2 launch nav2_bringup bringup_launch.py "
            "use_sim_time:=false autostart:=false use_composition:=True "
            "params_file:=/home/gio/rpi_robot/config/nav2_params.yaml map:=/home/gio/ARDK_2/maps/ardk_smoke_map.yaml"
        )
        self.nav_proc = nav_runner.start(self._nav2_cmd)
        
        # Initial wait for services (one time cost)
        wait_for_services(self, 120.0, 
                          "/lifecycle_manager_localization/manage_nodes",
                          "/lifecycle_manager_navigation/manage_nodes")

        # Status Loop
        self.create_timer(1.0, self._publish_status)

    def _publish_status(self):
        msg = ARDKStatus()
        msg.mode = self.state
        msg.transition_step = "IDLE" # Todo: track step
        msg.last_error = "" # Todo: track error
        
        # Determine Authority
        if self.state == SetMode.Request.MAPPING:
            msg.map_source = "slam"
            msg.tf_authority = "slam"
            msg.motion_authority = "teleop"
        elif self.state == SetMode.Request.NAVIGATION:
            msg.map_source = "map_server"
            msg.tf_authority = "amcl"
            msg.motion_authority = "nav2"
        else:
            msg.map_source = "none"
            msg.tf_authority = "none"
            msg.motion_authority = "none"
            
        self.status_pub.publish(msg)

    def handle_set_mode(self, req, resp):
        with self.lock:
            self.get_logger().info(f"Transition Request: {self.state} -> {req.target_mode}")
            try:
                success, msg = self._execute_transition(req)
                resp.success = success
                resp.message = msg
                if success:
                    self.state = req.target_mode
            except Exception as e:
                self.get_logger().error(f"Transition Failed: {e}")
                self._enter_fault()
                resp.success = False
                resp.message = str(e)
                self.state = 99 # FAULT/Unknown
                
        return resp

    def _execute_transition(self, req) -> (bool, str):
        target = req.target_mode
        
        if target == SetMode.Request.IDLE:
            return self._to_idle()
        elif target == SetMode.Request.MAPPING:
            return self._to_mapping(req)
        elif target == SetMode.Request.NAVIGATION:
            return self._to_navigation(req)
        else:
            return False, "Unknown mode"

    def _stop_motion(self):
        """Principle 7: Motion Safe Primitive"""
        msg = Twist()
        # Publish zero velocity
        for _ in range(5):
            self.vel_pub.publish(msg)
            time.sleep(0.05)
        
        # Todo: Cancel Nav2 goals if we had that client accessible/needed
        # For now, just ensuring 0 cmd_vel is the priority

    def _enter_fault(self):
        self.get_logger().error("ENTERING FAULT STATE - KILLING EVERYTHING")
        self._stop_motion()
        if self.slam_proc:
            slam_runner.stop(self.slam_proc)
            self.slam_proc = None
        if self.nav_proc:
            nav_runner.stop(self.nav_proc)
            self.nav_proc = None

    # --- Transitions ---

    def _to_idle(self):
        self._stop_motion()
        
        if self.slam_proc:
            self.get_logger().info("Stopping SLAM...")
            slam_runner.stop(self.slam_proc)
            self.slam_proc = None
            
        if self.nav_proc:
            self.get_logger().info("Shutting down Nav stacks (Cycle Down)...")
            nav_runner.shutdown_loc_nav(self, timeout_sec=20.0)
            # Do NOT stop the process
            
        return True, "Switched to IDLE"

    def _to_mapping(self, req):
        self._stop_motion()
        
        # Stop Nav stacks if active (but keep process)
        if self.nav_proc:
            self.get_logger().info("Ensuring Nav stacks are down for Mapping...")
            # Use persistent clients? For now use runner helper but mapped to persistent logic
            # To strictly follow Principle 5, runners should take clients. 
            # For this pass, we rely on runners creating temp clients or refactor runners later.
            # To save time, we let runners do it but acknowledge warm client optimization is for LoadMap/Save primarily.
            nav_runner.shutdown_loc_nav(self, timeout_sec=20.0, client_loc=self.cli_loc, client_nav=self.cli_nav)
            
        if not self.slam_proc:
            self.get_logger().info("Starting SLAM Toolbox...")
            params = "/home/gio/rpi_robot/config/slam_params.yaml"
            self.slam_proc = slam_runner.start(params)
            
            # Principle 4: Readiness Gate (No Sleep)
            self.get_logger().info("Waiting for Map Topic...")
            # Polling instead of sleep
            wait_for_topic(self, 10.0, '/map')
            
        return True, "Switched to MAPPING"

    def _to_navigation(self, req):
        self._stop_motion()
        
        # 1. Save Map if coming from Mapping
        if self.slam_proc:
            self.get_logger().info(f"Saving map...")
            path = req.map_yaml_path if req.map_yaml_path else "/home/gio/ARDK_2/maps/autosave"
            
            # Principle 4: Check map freshness (replaces sleep)
            # Simple check: wait for /map to be published (we did at start of mapping)
            # Better check: wait for one more message? 
            # For now, we assume if we are in mapping, map is publishing.
            
            # Use persistent client? Runners handle clients currently.
            slam_runner.save_map(self, path)
            
            self.get_logger().info("Stopping SLAM...")
            slam_runner.stop(self.slam_proc)
            self.slam_proc = None
            
            # Principle 2: Exclusivity Invariant
            # Verify SLAM is dead? 
            time.sleep(1.0) # Short micro-sleep to ensure process death (Principle 2 best effort for now)
            
        # 2. Resident Nav2 Check
        if not self.nav_proc:
             # Principle 1: This should not happen in resident mode
             self.get_logger().warn("Nav2 process missing during transition!")
             self.nav_proc = nav_runner.start(self._nav2_cmd)
             wait_for_services(self, 60.0, "/lifecycle_manager_localization/manage_nodes")

        # 3. Startup Sequence (Principle 3)
        # A) Localization (brings up map_server)
        self.get_logger().info("Starting Localization...")
        # Pass persistent client (Principle 5)
        nav_runner.startup_localization(self, timeout_sec=30.0, client=self.cli_loc)
        
        # B) Load Map
        map_path = req.map_yaml_path if req.map_yaml_path else "/home/gio/ARDK_2/maps/autosave.yaml"
        self.get_logger().info(f"Loading map: {map_path}")
        wait_for_services(self, 20.0, "/map_server/load_map")
        nav_runner.load_map(self, map_path, client=self.cli_load_map)
        
        # C) Navigation
        self.get_logger().info("Starting Navigation...")
        nav_runner.startup_navigation(self, timeout_sec=30.0, client=self.cli_nav)
        
        # 4. Wait Readiness (tf chain)
        self.get_logger().info("Verifying Readiness...")
        wait_for_tf_chain(self, self.tf_buffer, 10.0, "map", "odom", "base_link")
        
        return True, "Switched to NAVIGATION"

def main():
    rclpy.init()
    node = StateManager()
    
    # Run spin in separate thread so we can block in service callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node._enter_fault()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
