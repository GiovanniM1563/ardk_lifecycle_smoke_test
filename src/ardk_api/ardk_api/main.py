from fastapi import FastAPI, HTTPException
from contextlib import asynccontextmanager
from pydantic import BaseModel
from typing import Optional
import uvicorn
import os

from ardk_api.ros_bridge import ros_manager
from ardk_api import map_catalog
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

# Configurable map storage
MAP_STORAGE_DIR = os.environ.get("ARDK_MAP_DIR", os.path.expanduser("~/ARDK_2/maps"))

# --- Models ---
class ModeRequest(BaseModel):
    mode: int
    map_path: Optional[str] = ""

class SaveMapRequest(BaseModel):
    name: str

class LoadMapRequest(BaseModel):
    path: str

class MapSaveRequest(BaseModel):
    name: str
    overwrite: bool = False

class MapLoadRequest(BaseModel):
    name: str

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

@app.post("/ardk/clear_fault")
async def clear_fault():
    try:
        res = await ros_manager.node.clear_fault()
        return {"success": res.success, "message": res.message}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

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
    """Send a navigation goal. Must be in NAVIGATION mode."""
    try:
        # Check if in NAVIGATION mode
        status = ros_manager.node.get_status()
        if status is None or status["mode"] != 2:
            raise HTTPException(status_code=400, detail="Must be in NAVIGATION mode to send goals")
        
        pose = create_pose_stamped(req, ros_manager.node.get_clock().now)
        res = await ros_manager.node.navigate_to_pose(pose)
        return res
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/nav/cancel")
async def cancel_goal():
    """Cancel the active navigation goal."""
    try:
        res = await ros_manager.node.cancel_goal()
        return res
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/nav/state")
async def get_nav_state():
    """Get current navigation goal state (idle/active/succeeded/failed/canceled)."""
    return ros_manager.node.get_nav_state()

@app.post("/nav/route")
async def compute_route(req: PoseRequest):
    try:
        pose = create_pose_stamped(req, ros_manager.node.get_clock().now)
        path = await ros_manager.node.compute_path(pose)
        # Convert path to JSON friendly
        points = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in path.poses]
        return {"count": len(points), "path": points}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# --- Map Catalog API (Milestone 3) ---

@app.post("/maps/save")
async def save_map_to_catalog(req: MapSaveRequest):
    """Save current map to catalog with name."""
    try:
        # Validate name first
        if not map_catalog.validate_name(req.name):
            raise HTTPException(status_code=400, detail="Invalid name. Use alphanumeric and underscore only (1-50 chars).")
        
        # Save to temp location first via SLAM
        temp_prefix = f"/tmp/ardk_map_{req.name}"
        res = await ros_manager.node.save_map(temp_prefix)
        if res.result > 1:
            raise HTTPException(status_code=500, detail=f"SLAM save_map failed with code {res.result}")
        
        # Move to catalog with metadata
        metadata = map_catalog.save_map_to_catalog(
            name=req.name,
            source_prefix=temp_prefix,
            base_dir=MAP_STORAGE_DIR,
            overwrite=req.overwrite
        )
        return {"success": True, "map": metadata}
    except FileExistsError as e:
        raise HTTPException(status_code=409, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/maps")
async def list_maps_catalog():
    """List all maps in catalog."""
    return {"maps": map_catalog.list_maps(MAP_STORAGE_DIR)}

@app.post("/maps/load")
async def load_map_from_catalog(req: MapLoadRequest):
    """Load a map by name and switch to NAVIGATION."""
    try:
        if not map_catalog.validate_name(req.name):
            raise HTTPException(status_code=400, detail="Invalid name format.")
        
        map_path = map_catalog.get_map_path(req.name, MAP_STORAGE_DIR)
        if not map_path:
            raise HTTPException(status_code=404, detail=f"Map '{req.name}' not found in catalog.")
        
        # Switch to NAVIGATION with this map
        res = await ros_manager.node.set_mode(2, map_path)  # 2 = NAVIGATION
        return {"success": res.success, "message": res.message, "map_path": map_path}
    except Exception as e:
        if isinstance(e, HTTPException):
            raise e
        raise HTTPException(status_code=500, detail=str(e))

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
