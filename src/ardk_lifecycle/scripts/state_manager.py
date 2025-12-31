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
from ardk.runners import slam_runner, nav_runner
from ardk.core.readiness import wait_for_services, wait_for_tf_chain, wait_for_topic

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service
        self.srv = self.create_service(
            SetMode, 
            'set_mode', 
            self.handle_set_mode,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("ARDK State Manager Ready. Current State: IDLE")
        
        # Resident Nav2: Start once, keep alive.
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
        msg = Twist()
        # Send a few times to be sure
        for _ in range(3):
            self.vel_pub.publish(msg)
            time.sleep(0.05)

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
        # Stop Nav stacks if active (but keep process)
        if self.nav_proc:
            self.get_logger().info("Ensuring Nav stacks are down for Mapping...")
            nav_runner.shutdown_loc_nav(self, timeout_sec=20.0)
            
        if not self.slam_proc:
            self.get_logger().info("Starting SLAM Toolbox...")
            # We could assume params are in a default location or passed via some other way, 
            # here we use default launch
            # Use user-specified SLAM params
            params = "/home/gio/rpi_robot/config/slam_params.yaml"
            self.slam_proc = slam_runner.start(params)
            
            # Wait readiness
            # For SLAM, maybe wait for /scan and ensure /map appears?
            wait_for_services(self, 10.0, '/slam_toolbox/save_map')
            wait_for_topic(self, 10.0, '/map')
            
        return True, "Switched to MAPPING"

    def _to_navigation(self, req):
        self._stop_motion()
        
        # 1. Save Map if coming from Mapping (or if requested/configured)
        # The user API says: "MAPPING -> NAV: save map -> stop slam"
        if self.slam_proc:
            self.get_logger().info(f"Saving map to {req.map_yaml_path or 'autosave'}...")
            # Use request path or specific default
            # Use request path or specific default
            path = req.map_yaml_path if req.map_yaml_path else "/home/gio/ARDK_2/maps/autosave"
            
            # Give SLAM a moment to settle/generate map before saving
            self.get_logger().info("Waiting 5s for map generation...")
            time.sleep(5.0)
            
            slam_runner.save_map(self, path)
            
            self.get_logger().info("Stopping SLAM...")
            slam_runner.stop(self.slam_proc)
            self.slam_proc = None
            
            # Verify stopped? subprocesswait is called in stop()
            
        # 2. Resident Nav2: Ensure process is running
        if not self.nav_proc:
            self.get_logger().warn("Nav2 process was missing, restarting...")
            self.nav_proc = nav_runner.start(self._nav2_cmd)
            wait_for_services(self, 120.0, 
                              "/lifecycle_manager_localization/manage_nodes",
                              "/lifecycle_manager_navigation/manage_nodes")

        # 3. Startup Sequence
        # A) Localization (brings up map_server)
        self.get_logger().info("Starting Localization...")
        nav_runner.startup_localization(self, timeout_sec=30.0)
        
        # B) Load Map
        map_path = req.map_yaml_path if req.map_yaml_path else "/home/gio/ARDK_2/maps/autosave.yaml"
        self.get_logger().info(f"Loading map: {map_path}")
        # Now that map_server is active (from step A), we can use it
        wait_for_services(self, 20.0, "/map_server/load_map")
        nav_runner.load_map(self, map_path)
        
        # C) Navigation
        self.get_logger().info("Starting Navigation...")
        nav_runner.startup_navigation(self, timeout_sec=30.0)
        
        # 4. Wait Readiness
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
