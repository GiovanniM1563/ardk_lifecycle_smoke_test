"""
SLAM Toolbox runner for ARDK.

Provides start/stop/activate/deactivate/save_map operations.
"""
import subprocess
from typing import Optional

from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_msgs.msg import String

from ardk.core.process_group import start_process, stop_process_group
from ardk.core.service_utils import call_service, Timeouts
from ardk.core.robot_config import RobotConfig, get_config


def start(params_yaml: Optional[str] = None, config: Optional[RobotConfig] = None) -> subprocess.Popen:
    """
    Launch slam_toolbox with autostart:=false for explicit lifecycle control.
    ARDK will manually configure/activate the node after launch.
    Returns the Popen handle.
    """
    cfg = config or get_config()
    cmd = cfg.slam_launch_cmd
    
    # Ensure autostart is false for lifecycle control
    if "autostart:=" not in cmd:
        cmd += " autostart:=false"
    
    if params_yaml:
        cmd += f" slam_params_file:={params_yaml}"
    elif cfg.slam_params_file:
        cmd += f" slam_params_file:={cfg.slam_params_file}"
    
    return start_process(cmd)


def stop(handle: subprocess.Popen) -> None:
    """Stop the slam_toolbox process group."""
    stop_process_group(handle)


def activate(node: Node, timeout_sec: float = 30.0, config: Optional[RobotConfig] = None) -> None:
    """
    Activate the slam_toolbox lifecycle node (configure + activate).
    Must be called after start() and after the node appears in the graph.
    """
    cfg = config or get_config()
    change_state_srv = cfg.slam_change_state
    client = node.create_client(ChangeState, change_state_srv)
    
    # Configure transition (1)
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_CONFIGURE
    resp = call_service(
        node, client, req,
        wait_service_sec=Timeouts.DISCOVERY_BOOT,
        call_timeout_sec=timeout_sec,
        service_name=f"{change_state_srv}/configure"
    )
    if not resp.success:
        raise RuntimeError(f"SLAM configure failed: {resp}")
    
    # Activate transition (3)
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_ACTIVATE
    resp = call_service(
        node, client, req,
        wait_service_sec=5.0,  # Already discovered
        call_timeout_sec=timeout_sec,
        service_name=f"{change_state_srv}/activate"
    )
    if not resp.success:
        raise RuntimeError(f"SLAM activate failed: {resp}")
    
    node.get_logger().info("SLAM Toolbox activated.")


def deactivate(node: Node, timeout_sec: float = 15.0, config: Optional[RobotConfig] = None) -> None:
    """
    Gracefully deactivate and cleanup the slam_toolbox lifecycle node.
    Should be called before stop() for production-grade shutdown.
    """
    cfg = config or get_config()
    change_state_srv = cfg.slam_change_state
    client = node.create_client(ChangeState, change_state_srv)
    
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn("SLAM lifecycle service not available, skipping graceful shutdown")
        return
    
    try:
        # Deactivate transition (4)
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        call_service(
            node, client, req,
            wait_service_sec=2.0,
            call_timeout_sec=timeout_sec,
            retries=0,  # Don't retry on shutdown
            service_name=f"{change_state_srv}/deactivate"
        )
        
        # Cleanup transition (2)
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CLEANUP
        call_service(
            node, client, req,
            wait_service_sec=2.0,
            call_timeout_sec=timeout_sec,
            retries=0,
            service_name=f"{change_state_srv}/cleanup"
        )
        
        node.get_logger().info("SLAM Toolbox gracefully shutdown.")
    except Exception as e:
        node.get_logger().warn(f"SLAM graceful shutdown failed: {e}, will force stop")


def save_map(node: Node, out_prefix: str, timeout_sec: float = None, config: Optional[RobotConfig] = None) -> None:
    """
    Call slam_toolbox/save_map service.
    out_prefix: Absolute path without extension.
    """
    cfg = config or get_config()
    if timeout_sec is None:
        timeout_sec = Timeouts.SAVE_MAP
        
    client = node.create_client(SaveMap, cfg.slam_save_map)

    req = SaveMap.Request()
    # Handle version differences: some slam_toolbox use string, others use std_msgs/String
    if isinstance(req.name, str):
        req.name = out_prefix
    else:
        name_msg = String()
        name_msg.data = out_prefix
        req.name = name_msg

    resp = call_service(
        node, client, req,
        wait_service_sec=Timeouts.DISCOVERY_NORMAL,
        call_timeout_sec=timeout_sec,
        service_name=cfg.slam_save_map
    )
    
    if hasattr(resp, "result"):
        if resp.result > 1:
            raise RuntimeError(f"SaveMap returned error code: {resp.result}")
