"""
Centralized robot configuration for ARDK.

All robot-specific values in one place with sane defaults.
Supports override via environment variables or YAML file.

Loading precedence:
1. YAML file (ARDK_CONFIG_FILE env var)
2. Individual env vars (ARDK_*)
3. Defaults
"""
import os
from dataclasses import dataclass, field, fields
from typing import Optional
import yaml


@dataclass
class RobotConfig:
    """
    Centralized configuration for robot-specific parameters.
    
    All values have sane defaults for a typical ROS 2 / Nav2 / slam_toolbox setup.
    Override via environment variables or YAML config file.
    """
    
    # ==========================================================================
    # Frame IDs
    # ==========================================================================
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    
    # ==========================================================================
    # Topics
    # ==========================================================================
    scan_topic: str = "/scan"
    cmd_vel_topic: str = "/cmd_vel"
    
    # SLAM mode topics
    slam_map_topic: str = "/map"
    slam_pose_topic: str = "/pose"  # slam_toolbox publishes PoseWithCovarianceStamped
    
    # Navigation mode topics
    nav_map_topic: str = "/map"
    amcl_pose_topic: str = "/amcl_pose"
    
    # Stable output topics (ARDK republishes to these)
    ardk_map_topic: str = "/ardk/map"
    ardk_pose_topic: str = "/ardk/pose"
    ardk_status_topic: str = "/ardk_status"
    
    # ==========================================================================
    # Service Names
    # ==========================================================================
    slam_change_state: str = "/slam_toolbox/change_state"
    slam_save_map: str = "/slam_toolbox/save_map"
    map_server_load_map: str = "/map_server/load_map"
    lifecycle_manager_loc: str = "/lifecycle_manager_localization/manage_nodes"
    lifecycle_manager_nav: str = "/lifecycle_manager_navigation/manage_nodes"
    
    # ==========================================================================
    # Launch Commands
    # ==========================================================================
    slam_launch_cmd: str = "ros2 launch slam_toolbox online_sync_launch.py autostart:=false"
    nav2_launch_cmd: str = "ros2 launch nav2_bringup bringup_launch.py"
    
    # ==========================================================================
    # File Paths (empty = not set, will be required by caller)
    # ==========================================================================
    slam_params_file: str = ""
    nav2_params_file: str = ""
    default_map_path: str = ""
    map_storage_dir: str = ""  # Where to save/load maps
    
    # ==========================================================================
    # Behavior Flags
    # ==========================================================================
    slam_autostart: bool = False  # If True, skip explicit lifecycle activation
    nav2_autostart: bool = True   # Nav2 default uses autostart:=true
    
    @classmethod
    def from_env(cls) -> "RobotConfig":
        """
        Create config from environment variables.
        
        Env var names: ARDK_<FIELD_NAME> (uppercase)
        Example: ARDK_MAP_FRAME=my_map
        """
        cfg = cls()
        
        for f in fields(cfg):
            env_name = f"ARDK_{f.name.upper()}"
            env_val = os.environ.get(env_name)
            
            if env_val is not None:
                # Type conversion
                if f.type == bool:
                    setattr(cfg, f.name, env_val.lower() in ("true", "1", "yes"))
                else:
                    setattr(cfg, f.name, env_val)
        
        return cfg
    
    @classmethod
    def from_yaml(cls, path: str) -> "RobotConfig":
        """
        Create config from YAML file.
        
        Example YAML:
        ```yaml
        map_frame: my_map
        odom_frame: my_odom
        slam_params_file: /path/to/slam.yaml
        ```
        """
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        
        cfg = cls()
        for f in fields(cfg):
            if f.name in data:
                val = data[f.name]
                if f.type == bool and isinstance(val, str):
                    val = val.lower() in ("true", "1", "yes")
                setattr(cfg, f.name, val)
        
        return cfg
    
    @classmethod
    def load(cls) -> "RobotConfig":
        """
        Load config with precedence:
        1. YAML file (if ARDK_CONFIG_FILE is set)
        2. Environment variables
        3. Defaults
        """
        config_file = os.environ.get("ARDK_CONFIG_FILE")
        
        if config_file and os.path.exists(config_file):
            cfg = cls.from_yaml(config_file)
            # Also apply env overrides on top of YAML
            for f in fields(cfg):
                env_name = f"ARDK_{f.name.upper()}"
                env_val = os.environ.get(env_name)
                if env_val is not None:
                    if f.type == bool:
                        setattr(cfg, f.name, env_val.lower() in ("true", "1", "yes"))
                    else:
                        setattr(cfg, f.name, env_val)
            return cfg
        else:
            return cls.from_env()
    
    def get_slam_launch_cmd(self) -> str:
        """Get full SLAM launch command with params file if set."""
        cmd = self.slam_launch_cmd
        if self.slam_params_file:
            cmd += f" slam_params_file:={self.slam_params_file}"
        return cmd
    
    def get_nav2_launch_cmd(self, map_path: str = "") -> str:
        """Get full Nav2 launch command with params and map."""
        cmd = self.nav2_launch_cmd
        if self.nav2_params_file:
            cmd += f" params_file:={self.nav2_params_file}"
        if map_path:
            cmd += f" map:={map_path}"
        elif self.default_map_path:
            cmd += f" map:={self.default_map_path}"
        if self.nav2_autostart:
            cmd += " autostart:=true"
        else:
            cmd += " autostart:=false"
        return cmd


# Global config instance (loaded once at import)
# Use RobotConfig.load() to refresh
_config: Optional[RobotConfig] = None


def get_config() -> RobotConfig:
    """Get the global robot config, loading it if necessary."""
    global _config
    if _config is None:
        _config = RobotConfig.load()
    return _config


def reload_config() -> RobotConfig:
    """Force reload the global config."""
    global _config
    _config = RobotConfig.load()
    return _config
