import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
import threading
import asyncio
from concurrent.futures import ThreadPoolExecutor

# Msgs
from ardk_lifecycle.srv import SetMode
from ardk_lifecycle.msg import ARDKStatus
from slam_toolbox.srv import SaveMap
from nav2_msgs.srv import LoadMap
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class ARDKRosBridge(Node):
    def __init__(self):
        super().__init__('ardk_api_bridge')
        
        # --- Clients ---
        self.cli_mode = self.create_client(SetMode, '/set_mode')
        # We create these on demand or persistent? Persistent is faster.
        self.cli_save_map = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.cli_load_map = self.create_client(LoadMap, '/map_server/load_map')
        
        # --- Action Clients ---
        self.act_nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.act_plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        
        # --- Status Subscription ---
        self._latest_status = None
        self.sub_status = self.create_subscription(
            ARDKStatus, '/ardk_status', self._status_callback, 10)

        self.get_logger().info("ARDK API Bridge initialized.")

    async def set_mode(self, mode: int, map_path: str = ""):
        if not self.cli_mode.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("ARDK SetMode service not available")
            
        req = SetMode.Request()
        req.target_mode = mode
        req.map_yaml_path = map_path
        
        return await self._call_service(self.cli_mode, req)

    def _status_callback(self, msg: ARDKStatus):
        self._latest_status = msg

    def get_status(self):
        """Returns the latest cached status from /ardk_status topic."""
        if self._latest_status is None:
            return None
        s = self._latest_status
        return {
            "mode": s.mode,
            "transition_step": s.transition_step,
            "last_error": s.last_error,
            "map_source": s.map_source,
            "tf_authority": s.tf_authority,
            "motion_authority": s.motion_authority
        }

    async def save_map(self, name: str):
        # slam_toolbox requires std_msgs/String name wrapping
        # Actually in recent versions it might just be string, but let's follow our smoke test which used String message
        # Wait, slam_runner.py used String() for name.
        if not self.cli_save_map.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("SLAM SaveMap service not available")
            
        req = SaveMap.Request()
        # Handle version differences: some slam_toolbox use string, others use std_msgs/String
        if isinstance(req.name, str):
            req.name = name
        else:
            s = String()
            s.data = name
            req.name = s
        
        return await self._call_service(self.cli_save_map, req)

    async def load_map(self, path: str):
        if not self.cli_load_map.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Nav2 LoadMap service not available")
            
        req = LoadMap.Request()
        req.map_url = path
        
        return await self._call_service(self.cli_load_map, req)
        
    async def navigate_to_pose(self, pose: PoseStamped):
        if not self.act_nav.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Nav2 NavigateToPose action not available")
            
        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        # Action calls are 2-stage: Send Goal -> Get Handle -> Get Result
        # For simplicity here, we will just send goal and return the UUID or simple confirmation, 
        # or wait for result? User said "send goal poses". Usually implies firing it off.
        # But let's try to at least confirm it was accepted.
        
        send_goal_future = self.act_nav.send_goal_async(goal)
        goal_handle = await self._wrap_future(send_goal_future)
        
        if not goal_handle.accepted:
            raise RuntimeError("Goal was rejected")
            
        return {"goal_id": goal_handle.goal_id.uuid.tolist(), "status": "accepted"}

    async def compute_path(self, pose: PoseStamped, start: PoseStamped = None):
        if not self.act_plan.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Nav2 ComputePathToPose action not available")
            
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        if start:
            goal.start = start
            goal.use_start = True
            
        send_goal_future = self.act_plan.send_goal_async(goal)
        goal_handle = await self._wrap_future(send_goal_future)
        
        if not goal_handle.accepted:
             raise RuntimeError("Plan request rejected")
             
        result_future = goal_handle.get_result_async()
        result = await self._wrap_future(result_future)
        
        # Result has 'path' field
        return result.result.path

    # --- Helpers ---

    def _call_service(self, client, req):
        future = client.call_async(req)
        return self._wrap_future(future)

    def _wrap_future(self, ros_future: Future):
        """
        Adapts a rclpy Future into an asyncio Future.
        """
        loop = asyncio.get_event_loop()
        aio_future = loop.create_future()
        
        def _done_callback(fut):
            # This runs in ROS thread/executor
            if fut.cancelled():
                loop.call_soon_threadsafe(aio_future.cancel)
            elif fut.exception():
                loop.call_soon_threadsafe(aio_future.set_exception, fut.exception())
            else:
                loop.call_soon_threadsafe(aio_future.set_result, fut.result())
        
        ros_future.add_done_callback(_done_callback)
        return aio_future

# Singleton manager
class ROSManager:
    def __init__(self):
        self.node = None
        self.thread = None
        self.executor = None

    def start(self):
        rclpy.init()
        self.node = ARDKRosBridge()
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
        if self.thread:
            self.thread.join()

ros_manager = ROSManager()
