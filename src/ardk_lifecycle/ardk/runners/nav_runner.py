"""
Nav2 runner for ARDK.

Provides start/stop/load_map and lifecycle management operations.
"""
import subprocess
from typing import Optional

from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes, LoadMap

from ardk.core.process_group import start_process, stop_process_group
from ardk.core.service_utils import call_service, Timeouts
from ardk.core.robot_config import RobotConfig, get_config


def start(nav2_cmd: Optional[str] = None, config: Optional[RobotConfig] = None, map_path: str = "") -> subprocess.Popen:
    """
    Launch Nav2 stack (bringup).
    
    If nav2_cmd is provided, use it directly.
    Otherwise, build command from config.
    """
    cfg = config or get_config()
    
    if nav2_cmd:
        cmd = nav2_cmd
    else:
        cmd = cfg.get_nav2_launch_cmd(map_path)
    
    return start_process(cmd, stdout=None, stderr=None)


def stop(handle: subprocess.Popen) -> None:
    """Stop the Nav2 process group."""
    stop_process_group(handle)


def load_map(node: Node, map_yaml: str, timeout_sec: float = None, 
             client=None, config: Optional[RobotConfig] = None) -> None:
    """
    Call map_server/load_map with robust timeout handling.
    """
    cfg = config or get_config()
    if timeout_sec is None:
        timeout_sec = Timeouts.LOAD_MAP
        
    if client is None:
        client = node.create_client(LoadMap, cfg.map_server_load_map)
    
    req = LoadMap.Request()
    req.map_url = map_yaml
    
    resp = call_service(
        node, client, req,
        wait_service_sec=Timeouts.DISCOVERY_NORMAL,
        call_timeout_sec=timeout_sec,
        service_name=cfg.map_server_load_map
    )
    
    if hasattr(resp, "result"):
        if isinstance(resp.result, bool) and not resp.result:
            raise RuntimeError("LoadMap failed (bool=False)")
        if isinstance(resp.result, int) and resp.result != 0:
            raise RuntimeError(f"LoadMap failed with code {resp.result}")


def startup_localization(node: Node, timeout_sec: float = None, 
                         client=None, config: Optional[RobotConfig] = None) -> None:
    """Start localization lifecycle nodes."""
    cfg = config or get_config()
    _manage_lifecycle(node, cfg.lifecycle_manager_loc, ManageLifecycleNodes.Request.STARTUP, 
                      timeout_sec, client, is_startup=True)


def shutdown_localization(node: Node, timeout_sec: float = None, 
                          client=None, config: Optional[RobotConfig] = None) -> None:
    """Shutdown localization lifecycle nodes."""
    cfg = config or get_config()
    _manage_lifecycle(node, cfg.lifecycle_manager_loc, ManageLifecycleNodes.Request.SHUTDOWN, 
                      timeout_sec, client, is_startup=False)


def startup_navigation(node: Node, timeout_sec: float = None, 
                       client=None, config: Optional[RobotConfig] = None) -> None:
    """Start navigation lifecycle nodes."""
    cfg = config or get_config()
    _manage_lifecycle(node, cfg.lifecycle_manager_nav, ManageLifecycleNodes.Request.STARTUP, 
                      timeout_sec, client, is_startup=True)


def shutdown_navigation(node: Node, timeout_sec: float = None, 
                        client=None, config: Optional[RobotConfig] = None) -> None:
    """Shutdown navigation lifecycle nodes."""
    cfg = config or get_config()
    _manage_lifecycle(node, cfg.lifecycle_manager_nav, ManageLifecycleNodes.Request.SHUTDOWN, 
                      timeout_sec, client, is_startup=False)


def startup_loc_nav(node: Node, timeout_sec: float = None, 
                    client_loc=None, client_nav=None, config: Optional[RobotConfig] = None) -> None:
    """Start localization then navigation lifecycle managers."""
    startup_localization(node, timeout_sec, client=client_loc, config=config)
    startup_navigation(node, timeout_sec, client=client_nav, config=config)


def shutdown_loc_nav(node: Node, timeout_sec: float = None, 
                     client_loc=None, client_nav=None, config: Optional[RobotConfig] = None) -> None:
    """Shutdown navigation then localization lifecycle managers."""
    shutdown_navigation(node, timeout_sec, client=client_nav, config=config)
    shutdown_localization(node, timeout_sec, client=client_loc, config=config)


def _manage_lifecycle(
    node: Node, 
    service_name: str, 
    command: int, 
    timeout_sec: float = None,
    client=None,
    is_startup: bool = True
) -> None:
    """
    Internal helper for lifecycle management calls.
    Uses robust service call patterns with appropriate timeouts.
    """
    if client is None:
        client = node.create_client(ManageLifecycleNodes, service_name)
    
    req = ManageLifecycleNodes.Request()
    req.command = command
    
    # Use lifecycle-specific timeouts
    if is_startup:
        discovery_timeout = Timeouts.DISCOVERY_BOOT
        exec_timeout = timeout_sec if timeout_sec else Timeouts.LIFECYCLE_STARTUP
    else:
        discovery_timeout = Timeouts.DISCOVERY_NORMAL
        exec_timeout = timeout_sec if timeout_sec else Timeouts.LIFECYCLE_SHUTDOWN
    
    command_name = "STARTUP" if command == ManageLifecycleNodes.Request.STARTUP else "SHUTDOWN"
    node.get_logger().info(f"Calling {service_name} ({command_name})...")
    
    resp = call_service(
        node, client, req,
        wait_service_sec=discovery_timeout,
        call_timeout_sec=exec_timeout,
        service_name=f"{service_name}/{command_name}"
    )
    
    if not resp.success:
        raise RuntimeError(f"{service_name} returned success=False")
    
    node.get_logger().info(f"{service_name} ({command_name}) completed.")
