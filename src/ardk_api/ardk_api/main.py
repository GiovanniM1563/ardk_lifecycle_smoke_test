from fastapi import FastAPI, HTTPException
from contextlib import asynccontextmanager
from pydantic import BaseModel
from typing import Optional
import uvicorn

from ardk_api.ros_bridge import ros_manager
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

# --- Models ---
class ModeRequest(BaseModel):
    mode: int
    map_path: Optional[str] = ""

class SaveMapRequest(BaseModel):
    name: str

class LoadMapRequest(BaseModel):
    path: str

class PoseRequest(BaseModel):
    x: float
    y: float
    theta: float = 0.0
    frame_id: str = "map"

# --- Logic ---

def create_pose_stamped(req: PoseRequest, now_func) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = req.frame_id
    msg.header.stamp = now_func().to_msg()
    msg.pose.position.x = req.x
    msg.pose.position.y = req.y
    msg.pose.position.z = 0.0
    # Simple Euler to Quat (Z-axis rotation only)
    import math
    msg.pose.orientation.z = math.sin(req.theta / 2.0)
    msg.pose.orientation.w = math.cos(req.theta / 2.0)
    return msg

# --- Lifecycle ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Starting ROS 2 Bridge...")
    ros_manager.start()
    yield
    print("Stopping ROS 2 Bridge...")
    ros_manager.stop()

# --- App ---
app = FastAPI(title="ARDK API", lifespan=lifespan)

@app.post("/ardk/mode")
async def set_mode(req: ModeRequest):
    try:
        res = await ros_manager.node.set_mode(req.mode, req.map_path)
        return {"success": res.success, "message": res.message}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/ardk/status")
async def get_status():
    status = ros_manager.node.get_status()
    if status is None:
        raise HTTPException(status_code=503, detail="Status not yet available")
    return status

@app.post("/slam/save")
async def save_map(req: SaveMapRequest):
    try:
        res = await ros_manager.node.save_map(req.name)
        # Check result
        if res.result != 0: # 0 is success in some versions, or <2
             return {"result": res.result, "status": "Maybe Error (Check codes)"}
        return {"result": res.result, "status": "Success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/nav/map")
async def load_map(req: LoadMapRequest):
    try:
        res = await ros_manager.node.load_map(req.path)
        return {"result": res.result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/nav/goal")
async def send_goal(req: PoseRequest):
    try:
        pose = create_pose_stamped(req, ros_manager.node.get_clock().now)
        res = await ros_manager.node.navigate_to_pose(pose)
        return res
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/nav/route")
async def compute_route(req: PoseRequest):
    try:
        pose = create_pose_stamped(req, ros_manager.node.get_clock().now)
        path = await ros_manager.node.compute_path(pose)
        # Convert path to JSON friendly
        points = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in path.poses]
        return {"count": len(points), "path": points}
    except Exception as e:
        # 500 or 400? 500 for ROS errors
        raise HTTPException(status_code=500, detail=str(e))

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
