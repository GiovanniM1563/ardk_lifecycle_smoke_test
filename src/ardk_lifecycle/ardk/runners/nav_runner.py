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

def load_map(node: Node, map_yaml: str, timeout_sec: float = 5.0, client=None) -> None:
    """
    Call map_server/load_map.
    """
    if client is None:
        client = node.create_client(LoadMap, SRV_LOAD_MAP)
        
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service not available: {SRV_LOAD_MAP}")
    
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

def startup_localization(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_LOC, ManageLifecycleNodes.Request.STARTUP, timeout_sec, client)

def shutdown_localization(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_LOC, ManageLifecycleNodes.Request.SHUTDOWN, timeout_sec, client)

def startup_navigation(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_NAV, ManageLifecycleNodes.Request.STARTUP, timeout_sec, client)

def shutdown_navigation(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_NAV, ManageLifecycleNodes.Request.SHUTDOWN, timeout_sec, client)

def startup_loc_nav(node: Node, timeout_sec: float = 10.0, client=None) -> None:
    """
    Start localization then navigation lifecycle managers.
    """
    # Note: If passing a single client, it might be ambiguous which service it is for
    # unless 'client' here means a dict or we just don't support passing unified client to unified wrapper.
    # However, for shutdown/startup loc_nav, we mostly use this for "process lifecycle", 
    # and the individual calls use distinct services (loc vs nav).
    # So we should probably NOT support a single 'client' arg for this dual-function wrapper 
    # unless we pass 'client_loc' and 'client_nav'.
    # BUT, the error was "unexpected keyword argument 'client'" because state_manager called it with client=None.
    # So we must support the signature.
    
    # We will accept it but ignore it if it's just 'None' to fix the error, 
    # or better: StateManager shouldn't pass 'client' to this dual-wrapper if it holds distinct clients.
    
    # Correction: StateManager called: nav_runner.shutdown_loc_nav(self, timeout_sec=20.0, client=None)
    # The 'client' passed was None.
    # If we want to support passing real clients, we need client_loc and client_nav.
    # For now, let's fix the signature to accept **kwargs or specific args to unbreak the call.
    startup_localization(node, timeout_sec, client=client)  # Using one client for both? Risky if they are different services.
    startup_navigation(node, timeout_sec, client=client)    # Wait, client IS fixed to a service name in create_client?
    # YES. persistent clients are bound to a specific service name.
    # So we CANNOT pass one client to both localization and navigation startups.
    # We must chang the signature to client_loc=None, client_nav=None.
    pass

def startup_loc_nav(node: Node, timeout_sec: float = 10.0, client_loc=None, client_nav=None) -> None:
    """
    Start localization then navigation lifecycle managers.
    """
    startup_localization(node, timeout_sec, client=client_loc)
    startup_navigation(node, timeout_sec, client=client_nav)

def shutdown_loc_nav(node: Node, timeout_sec: float = 10.0, client_loc=None, client_nav=None, client=None) -> None:
    """
    Shutdown navigation then localization lifecycle managers.
    Accepts client_loc/nav, and 'client' for backward compat/erroneous calls (ignored or warned).
    """
    shutdown_navigation(node, timeout_sec, client=client_nav)
    shutdown_localization(node, timeout_sec, client=client_loc)

def stop(handle: subprocess.Popen) -> None:
    """
    Stop the Nav2 process group.
    """
    stop_process_group(handle)

def _manage_lifecycle(node: Node, service_name: str, command: int, timeout_sec: float, client=None) -> None:
    if client is None:
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
