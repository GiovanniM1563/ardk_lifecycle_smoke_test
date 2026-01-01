import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
import threading
import asyncio
from concurrent.futures import ThreadPoolExecutor

# Msgs
from ardk_lifecycle.srv import SetMode, ClearFault
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
        self.cli_clear_fault = self.create_client(ClearFault, '/clear_fault')
        # We create these on demand or persistent? Persistent is faster.
        self.cli_save_map = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.cli_load_map = self.create_client(LoadMap, '/map_server/load_map')
        
        # --- Action Clients ---
        self.act_nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.act_plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        
        # --- Goal State Tracking (Milestone 4) ---
        self._goal_handle = None
        self._goal_state = "idle"  # idle, active, succeeded, failed, canceled
        self._goal_message = ""
        
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
            "last_error_time": {"sec": s.last_error_time.sec, "nanosec": s.last_error_time.nanosec},
            "map_source": s.map_source,
            "tf_authority": s.tf_authority,
            "motion_authority": s.motion_authority,
            "nav_stack_state": s.nav_stack_state,
            "tf_valid": s.tf_valid,
            "services_valid": s.services_valid
        }

    async def clear_fault(self):
        """Clear a latched fault state."""
        if not self.cli_clear_fault.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("ClearFault service not available")
        req = ClearFault.Request()
        return await self._call_service(self.cli_clear_fault, req)

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
        """Send a navigation goal and track state."""
        if not self.act_nav.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Nav2 NavigateToPose action not available")
            
        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        # Send goal with feedback callback
        send_goal_future = self.act_nav.send_goal_async(
            goal, 
            feedback_callback=self._nav_feedback_callback
        )
        goal_handle = await self._wrap_future(send_goal_future)
        
        if not goal_handle.accepted:
            self._goal_state = "failed"
            self._goal_message = "Goal was rejected"
            raise RuntimeError("Goal was rejected")
        
        # Track the goal
        self._goal_handle = goal_handle
        self._goal_state = "active"
        self._goal_message = "Navigating to goal"
        
        # Set up result callback (non-blocking)
        goal_handle.get_result_async().add_done_callback(self._nav_result_callback)
        
        return {"goal_id": goal_handle.goal_id.uuid.tolist(), "accepted": True}

    def _nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        fb = feedback_msg.feedback
        self._goal_message = f"Distance remaining: {fb.distance_remaining:.2f}m"

    def _nav_result_callback(self, future):
        """Handle navigation result."""
        try:
            result = future.result()
            status = result.status
            # Status codes: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if status == 4:
                self._goal_state = "succeeded"
                self._goal_message = "Goal reached"
            elif status == 5:
                self._goal_state = "canceled"
                self._goal_message = "Goal canceled"
            else:
                self._goal_state = "failed"
                self._goal_message = f"Navigation failed (status={status})"
        except Exception as e:
            self._goal_state = "failed"
            self._goal_message = str(e)
        finally:
            self._goal_handle = None

    async def cancel_goal(self):
        """Cancel the active navigation goal."""
        if self._goal_handle is None:
            return {"success": False, "message": "No active goal"}
        
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_response = await self._wrap_future(cancel_future)
        
        self._goal_state = "canceled"
        self._goal_message = "Goal canceled by user"
        self._goal_handle = None
        
        return {"success": True, "message": "Goal canceled"}

    def get_nav_state(self):
        """Get current navigation goal state."""
        return {
            "state": self._goal_state,
            "message": self._goal_message,
            "has_active_goal": self._goal_handle is not None
        }

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
