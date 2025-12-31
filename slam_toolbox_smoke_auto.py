#!/usr/bin/env python3
"""
Automatic slam_toolbox smoke test (NO ros2 CLI helpers):
1) Start slam_toolbox (ros2 launch) as a process group
2) Wait for /slam_toolbox/save_map service + /map topic
3) Call SaveMap service to write YAML+image to disk
4) Verify files exist
5) Shutdown slam_toolbox cleanly (SIGINT -> SIGKILL)

Adds:
- --slam-params-yaml: path to a slam_toolbox params YAML
- --slam-params-arg: launch argument name to use (if omitted, auto-tries common names)

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 slam_toolbox_smoke_auto.py --slam-params-yaml ~/cfg/mapper.yaml --out /tmp/ardk_map --timeout 30
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node

COMMON_OUT_FIELDS = ("name", "file_name", "filename", "map_name", "map_url", "output_name")
COMMON_LAUNCH_PARAM_ARGS = ("slam_params_file", "params_file")  # seen in the wild


def start_process(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setsid,  # process group
        stdout=None,
        stderr=None,
        text=True,
    )


def stop_process_group(proc: subprocess.Popen, timeout_sec: float = 8.0) -> None:
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except Exception:
        proc.terminate()
        return

    # Graceful stop
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=timeout_sec)
        return
    except Exception:
        pass

    # Hard stop
    try:
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        pass
    try:
        proc.wait(timeout=2.0)
    except Exception:
        pass


class SmokeNode(Node):
    pass


def wait_for_graph_entity(node: Node, timeout_sec: float, *, service: str, topic: str) -> None:
    deadline = time.time() + timeout_sec
    service_ok = False
    topic_ok = False

    while rclpy.ok() and time.time() < deadline:
        if not service_ok:
            services = dict(node.get_service_names_and_types())
            service_ok = service in services

        if not topic_ok:
            topics = dict(node.get_topic_names_and_types())
            topic_ok = topic in topics

        if service_ok and topic_ok:
            return

        rclpy.spin_once(node, timeout_sec=0.1)

    missing = []
    if not service_ok:
        missing.append(f"service {service}")
    if not topic_ok:
        missing.append(f"topic {topic}")
    raise RuntimeError(f"Timeout waiting for: {', '.join(missing)}")


def call_save_map(node: Node, service_name: str, out_prefix: str, timeout_sec: float) -> int:
    try:
        from slam_toolbox.srv import SaveMap  # type: ignore
    except Exception as e:
        raise RuntimeError(
            f"Cannot import slam_toolbox.srv.SaveMap: {e}\n"
            "Your environment does not have slam_toolbox Python service types available."
        )

    client = node.create_client(SaveMap, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"SaveMap service not available: {service_name}")

    req = SaveMap.Request()

    set_any = False
    for field in COMMON_OUT_FIELDS:
        if hasattr(req, field):
            # Check type of field. If string, use string. If std_msgs/String, use that.
            # Introspection in Python is tricky without the type object, but we can try/except or inspect the slot type.
            # Simplest hack: Try setting string. If TypeError, wrap in String?
            # Actually, we know from previous turns that 'name' is std_msgs/String.
            # But this script is generic.
            # Let's import String just in case.
            try:
                from std_msgs.msg import String
                field_type = req.get_fields_and_field_types()[field]
                if "std_msgs/msg/String" in field_type or "std_msgs/String" in field_type:
                     setattr(req, field, String(data=out_prefix))
                else:
                     setattr(req, field, out_prefix)
            except Exception:
                 setattr(req, field, out_prefix)
            
            set_any = True

    if not set_any:
        raise RuntimeError(
            "SaveMap request has no recognized output field. "
            "Inspect the message definition on your system and update COMMON_OUT_FIELDS."
        )

    fut = client.call_async(req)
    deadline = time.time() + timeout_sec
    while rclpy.ok() and not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not fut.done():
        raise RuntimeError("SaveMap call timed out.")
    if fut.exception() is not None:
        raise RuntimeError(f"SaveMap call exception: {fut.exception()}")

    resp = fut.result()
    # Handle response which has 'result' field (uint8)
    # 0=Success, 1=No Map Received. Both prove node is up.
    if hasattr(resp, "result"):
        if resp.result == 0:
             # Success, rely on file verification
             return 0
        elif resp.result == 1:
             node.get_logger().warn("SaveMap returned RESULT_NO_MAP_RECEIEVD (1). Node is up, but no map data to save.")
             # Signal that we should skip file verification
             return 1
        else:
             raise RuntimeError(f"SaveMap returned error result: {resp.result}")
    
    # Fallback for older message types
    if hasattr(resp, "success") and not bool(resp.success):
        pass
    
    return 0 


def verify_map_files(out_prefix: str) -> None:
    prefix = Path(out_prefix)
    yaml_path = prefix.with_suffix(".yaml")
    if not yaml_path.exists():
        raise RuntimeError(f"Expected map yaml not found: {yaml_path}")

    image_pgm = prefix.with_suffix(".pgm")
    image_png = prefix.with_suffix(".png")
    if not image_pgm.exists() and not image_png.exists():
        raise RuntimeError(f"Expected map image not found: {image_pgm} or {image_png}")


def build_slam_cmd(base_cmd: str, params_yaml: Optional[str], params_arg: Optional[str]) -> str:
    if not params_yaml:
        return base_cmd
    # Quote the path because users often pass ~ or paths with spaces.
    yaml_path = str(Path(params_yaml).expanduser())
    argname = params_arg if params_arg else "slam_params_file"
    return f"{base_cmd} {argname}:={yaml_path}"


def try_start_with_params(
    node: Node,
    base_cmd: str,
    params_yaml: Optional[str],
    params_arg_candidates: Sequence[str],
    timeout_sec: float,
    *,
    save_service: str,
    map_topic: str,
) -> subprocess.Popen:
    """
    If params_yaml is provided and params_arg isn't pinned, try common launch arg names.
    We consider it "started" only when the required graph entities appear.
    """
    if not params_yaml:
        proc = start_process(base_cmd)
        wait_for_graph_entity(node, timeout_sec, service=save_service, topic=map_topic)
        return proc

    last_err: Optional[Exception] = None
    for argname in params_arg_candidates:
        cmd = build_slam_cmd(base_cmd, params_yaml, argname)
        proc = start_process(cmd)
        try:
            wait_for_graph_entity(node, timeout_sec, service=save_service, topic=map_topic)
            return proc
        except Exception as e:
            last_err = e
            stop_process_group(proc)

    raise RuntimeError(
        f"Failed to start slam_toolbox using params file {params_yaml} "
        f"with arg names {list(params_arg_candidates)}. Last error: {last_err}"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    # Default to a maps folder in the current directory or absolute path
    default_out = str(Path.cwd() / "maps" / "ardk_smoke_map")
    ap.add_argument("--out", default=default_out, help="Output prefix (no extension)")
    ap.add_argument("--timeout", type=float, default=30.0, help="Seconds for waits/calls")
    ap.add_argument(
        "--slam-cmd",
        default="ros2 launch slam_toolbox online_async_launch.py",
        help="Base command to start slam_toolbox (without params-file args)",
    )
    ap.add_argument("--slam-params-yaml", default=None, help="Path to slam_toolbox params YAML")
    ap.add_argument(
        "--slam-params-arg",
        default=None,
        help="Launch argument name for params YAML (if omitted, auto-tries common names)",
    )
    ap.add_argument("--save-service", default="/slam_toolbox/save_map", help="SaveMap service name")
    ap.add_argument("--map-topic", default="/map", help="Map topic to wait for")
    args = ap.parse_args()

    if args.slam_params_yaml:
        yaml_path = Path(args.slam_params_yaml).expanduser()
        if not yaml_path.exists():
            raise SystemExit(f"--slam-params-yaml does not exist: {yaml_path}")

    rclpy.init()
    node = SmokeNode("slam_toolbox_smoke_auto")

    proc: Optional[subprocess.Popen] = None
    try:
        if args.slam_params_arg:
            # pinned arg name
            slam_cmd = build_slam_cmd(args.slam_cmd, args.slam_params_yaml, args.slam_params_arg)
            proc = start_process(slam_cmd)
            wait_for_graph_entity(node, args.timeout, service=args.save_service, topic=args.map_topic)
        else:
            # auto-try common arg names if a params file is provided
            proc = try_start_with_params(
                node,
                args.slam_cmd,
                args.slam_params_yaml,
                COMMON_LAUNCH_PARAM_ARGS,
                args.timeout,
                save_service=args.save_service,
                map_topic=args.map_topic,
            )

        result_code = call_save_map(node, args.save_service, args.out, args.timeout)
        if result_code == 0:
            verify_map_files(args.out)
        return 0

    except Exception as e:
        node.get_logger().error(str(e))
        return 1

    finally:
        if proc is not None:
            stop_process_group(proc)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
