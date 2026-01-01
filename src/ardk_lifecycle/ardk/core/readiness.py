import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException

def wait_for_services(node: Node, timeout_sec: float, *service_names: str) -> None:
    """
    Wait until all services in 'service_names' appear in the graph.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        available = set(name for (name, _) in node.get_service_names_and_types())
        if all(s in available for s in service_names):
            return
        # rclpy.spin_once(node, timeout_sec=0.1)  # Warning: Dangerous inside callbacks
        # We should just sleep if we are in a threaded executor context, as discovery runs in RMW background
        # But to be safe vs legacy single threaded, spin_once is "okay" usually.
        # However, for robustness on RPi with MultiThreadedExecutor, sleep is safer.
        time.sleep(0.1)
    
    available = set(name for (name, _) in node.get_service_names_and_types())
    missing = [s for s in service_names if s not in available]
    node.get_logger().error(f"Timeout waiting for services: {missing}. Available: {sorted(list(available))}")
    raise RuntimeError(f"Timeout waiting for services: {missing}")

def wait_for_service_loss(node: Node, timeout_sec: float, service_name: str) -> None:
    """
    Wait until 'service_name' disappears from the graph.
    Used for exclusivity checks (e.g., ensuring SLAM is fully gone before starting AMCL).
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        available = set(name for (name, _) in node.get_service_names_and_types())
        if service_name not in available:
            return
        time.sleep(0.1)
    raise RuntimeError(f"Timeout waiting for service to disappear: {service_name}")

def wait_for_node(node: Node, timeout_sec: float, node_name: str) -> None:
    """
    Wait until 'node_name' appears in the ROS graph.
    Used to replace fixed sleeps when waiting for a node to launch.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        node_names = [n for n, _ in node.get_node_names_and_namespaces()]
        if node_name in node_names:
            return
        time.sleep(0.1)
    raise RuntimeError(f"Timeout waiting for node: {node_name}")

def wait_for_topic(node: Node, timeout_sec: float, topic_name: str) -> None:
    """
    Wait until 'topic_name' appears in the graph.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        topics = dict(node.get_topic_names_and_types())
        if topic_name in topics:
            return
        time.sleep(0.1)
    raise RuntimeError(f"Timeout waiting for topic: {topic_name}")

def wait_for_tf_chain(node: Node, tf_buffer, timeout_sec: float, frame_map: str, frame_odom: str, frame_base: str) -> None:
    """
    Wait for map->odom AND odom->base_link transforms to be available.
    Requires the node to hold a Buffer/Listener.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        try:
            tf_buffer.lookup_transform(frame_map, frame_odom, Time())
            tf_buffer.lookup_transform(frame_odom, frame_base, Time())
            return
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f"Timeout waiting for TF chain {frame_map}->{frame_odom}->{frame_base}")
