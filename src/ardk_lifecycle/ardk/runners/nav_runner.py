import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes, LoadMap
from ardk.core.process_group import start_process, stop_process_group
import subprocess
import time

# Service Names Verification from Smoke Test
SRV_LOC = "/lifecycle_manager_localization/manage_nodes"
SRV_NAV = "/lifecycle_manager_navigation/manage_nodes"
SRV_LOC = "/lifecycle_manager_localization/manage_nodes"
SRV_NAV = "/lifecycle_manager_navigation/manage_nodes"
SRV_LOAD_MAP = "/map_server/load_map"

def start(nav2_cmd: str) -> subprocess.Popen:
    """
    Launch Nav2 stack (bringup).
    Must contain 'autostart:=true' if you want it to come up fully, 
    OR 'autostart:=false' if you want manual control (which we support via startup_loc_nav).
    """
    return start_process(nav2_cmd)

def load_map(node: Node, map_yaml: str, timeout_sec: float = 5.0) -> None:
    """
    Call map_server/load_map.
    """
    client = node.create_client(LoadMap, SRV_MAP)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service not available: {SRV_MAP}")
    
    req = LoadMap.Request()
    req.map_url = map_yaml
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
        raise RuntimeError("Timed out calling LoadMap")
    
    resp = fut.result()
    if hasattr(resp, "result"):
        # enum: 0=SUCCESS, 1=MAP_DOES_NOT_EXIST, 2=INVALID_MAP_METADATA, 3=INVALID_MAP_DATA
        if resp.result != 0: 
             # Be permissive if response type varies, but smoke test showed explicit LoadMap srv usage works.
             # If result is boolean-like in some versions, check that too.
             if isinstance(resp.result, bool) and not resp.result:
                 raise RuntimeError("LoadMap failed (bool=False)")
             if isinstance(resp.result, int) and resp.result != 0:
                 raise RuntimeError(f"LoadMap failed with code {resp.result}")

def startup_localization(node: Node, timeout_sec: float = 20.0) -> None:
    _manage_lifecycle(node, SRV_LOC, ManageLifecycleNodes.Request.STARTUP, timeout_sec)

def shutdown_localization(node: Node, timeout_sec: float = 20.0) -> None:
    _manage_lifecycle(node, SRV_LOC, ManageLifecycleNodes.Request.SHUTDOWN, timeout_sec)

def startup_navigation(node: Node, timeout_sec: float = 20.0) -> None:
    _manage_lifecycle(node, SRV_NAV, ManageLifecycleNodes.Request.STARTUP, timeout_sec)

def shutdown_navigation(node: Node, timeout_sec: float = 20.0) -> None:
    _manage_lifecycle(node, SRV_NAV, ManageLifecycleNodes.Request.SHUTDOWN, timeout_sec)

def startup_loc_nav(node: Node, timeout_sec: float = 10.0) -> None:
    """
    Start localization then navigation lifecycle managers.
    """
    startup_localization(node, timeout_sec)
    startup_navigation(node, timeout_sec)

def shutdown_loc_nav(node: Node, timeout_sec: float = 10.0) -> None:
    """
    Shutdown navigation then localization lifecycle managers.
    """
    shutdown_navigation(node, timeout_sec)
    shutdown_localization(node, timeout_sec)

def stop(handle: subprocess.Popen) -> None:
    """
    Stop the Nav2 process group.
    """
    stop_process_group(handle)

def _manage_lifecycle(node: Node, service_name: str, command: int, timeout_sec: float) -> None:
    client = node.create_client(ManageLifecycleNodes, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        # We might log this but for a runner primitive, raising is safer to indicate failure
        raise RuntimeError(f"Lifecycle service not available: {service_name}")
        
    req = ManageLifecycleNodes.Request()
    req.command = command
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
         raise RuntimeError(f"Timed out calling {service_name}")
    
    resp = fut.result()
    if not resp.success:
         raise RuntimeError(f"{service_name} returned success=False")
