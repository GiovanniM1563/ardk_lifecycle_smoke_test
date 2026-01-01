import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_msgs.msg import String
from ardk.core.process_group import start_process, stop_process_group
import subprocess
from typing import Optional
import time


def start(params_yaml: Optional[str] = None) -> subprocess.Popen:
    """
    Launch slam_toolbox with autostart:=false for explicit lifecycle control.
    ARDK will manually configure/activate the node after launch.
    Returns the Popen handle.
    """
    cmd = "ros2 launch slam_toolbox online_sync_launch.py autostart:=false"
    if params_yaml:
        cmd += f" slam_params_file:={params_yaml}"
    
    return start_process(cmd)


def stop(handle: subprocess.Popen) -> None:
    """
    Stop the slam_toolbox process group.
    """
    stop_process_group(handle)


def activate(node: Node, timeout_sec: float = 10.0) -> None:
    """
    Activate the slam_toolbox lifecycle node (configure + activate).
    Must be called after start() and after the node appears in the graph.
    """
    change_state_srv = "/slam_toolbox/change_state"
    client = node.create_client(ChangeState, change_state_srv)
    
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Timeout waiting for {change_state_srv}")
    
    # Configure transition (1)
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_CONFIGURE
    _call_sync(node, client, req, timeout_sec)
    
    # Activate transition (3)
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_ACTIVATE
    _call_sync(node, client, req, timeout_sec)
    
    node.get_logger().info("SLAM Toolbox activated.")


def deactivate(node: Node, timeout_sec: float = 10.0) -> None:
    """
    Gracefully deactivate and cleanup the slam_toolbox lifecycle node.
    Should be called before stop() for production-grade shutdown.
    """
    change_state_srv = "/slam_toolbox/change_state"
    client = node.create_client(ChangeState, change_state_srv)
    
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn("SLAM lifecycle service not available, skipping graceful shutdown")
        return
    
    try:
        # Deactivate transition (4)
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        _call_sync(node, client, req, timeout_sec)
        
        # Cleanup transition (2)
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CLEANUP
        _call_sync(node, client, req, timeout_sec)
        
        node.get_logger().info("SLAM Toolbox gracefully shutdown.")
    except Exception as e:
        node.get_logger().warn(f"SLAM graceful shutdown failed: {e}, will force stop")


def save_map(node: Node, out_prefix: str, timeout_sec: float = 5.0) -> None:
    """
    Call slam_toolbox/save_map service. 
    out_prefix: Absolute path without extension.
    """
    client = node.create_client(SaveMap, "/slam_toolbox/save_map")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError("Timeout waiting for /slam_toolbox/save_map")

    req = SaveMap.Request()
    # Handle version differences: some slam_toolbox use string, others use std_msgs/String
    if isinstance(req.name, str):
        req.name = out_prefix
    else:
        name_msg = String()
        name_msg.data = out_prefix
        req.name = name_msg

    fut = client.call_async(req)
    if node.executor:
        start_time = time.time()
        while not fut.done():
            if time.time() - start_time > timeout_sec:
                break
            time.sleep(0.1)
    else:
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)

    if not fut.done():
        raise RuntimeError("Timed out calling SaveMap")
    
    resp = fut.result()
    if hasattr(resp, "result"):
        if resp.result > 1:
            raise RuntimeError(f"SaveMap returned error code: {resp.result}")


def _call_sync(node: Node, client, req, timeout_sec: float) -> None:
    """Helper to call a service synchronously."""
    fut = client.call_async(req)
    if node.executor:
        start_time = time.time()
        while not fut.done():
            if time.time() - start_time > timeout_sec:
                raise RuntimeError("Timeout calling lifecycle service")
            time.sleep(0.1)
    else:
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
    
    if not fut.done():
        raise RuntimeError("Timeout calling lifecycle service")
    
    resp = fut.result()
    if not resp.success:
        raise RuntimeError(f"Lifecycle transition failed: {resp}")
