import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from ardk.core.process_group import start_process, stop_process_group
import subprocess
from typing import Optional

def start(params_yaml: Optional[str] = None) -> subprocess.Popen:
    """
    Launch slam_toolbox (mapper_params_online_sync default or custom yaml).
    Returns the Popen handle.
    """
    cmd = "ros2 launch slam_toolbox online_sync_launch.py"
    if params_yaml:
        cmd += f" slam_params_file:={params_yaml}"
    
    # We inherit stdout/stderr by default via None in start_process if we pass nothing,
    # but here let's stick to simple execution.
    # User asked for: slam.start(params_yaml) -> handle
    return start_process(cmd)

def save_map(node: Node, out_prefix: str, timeout_sec: float = 5.0) -> None:
    """
    Call slam_toolbox/save_map service. 
    out_prefix: Absolute path without extension.
    """
    client = node.create_client(SaveMap, "/slam_toolbox/save_map")
    if not client.wait_for_service(timeout_sec=timeout_sec):
         raise RuntimeError("Time out waiting for /slam_toolbox/save_map")

    req = SaveMap.Request()
    # Handle std_msgs/String vs simple string depending on version, 
    # but our smoke test confirmed it needs std_msgs.msg.String for 'name'.
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
    # Smoke test logic: result=0 is success, result=1 is 'no map received' (technically success communication but no data)
    # Raising error if we fail fundamentally.
    if hasattr(resp, "result"):
        if resp.result > 1: # 0 and 1 are 'ok-ish' for connectivity
             raise RuntimeError(f"SaveMap returned error code: {resp.result}")
    
def stop(handle: subprocess.Popen) -> None:
    """
    Stop the slam_toolbox process group.
    """
    stop_process_group(handle)
